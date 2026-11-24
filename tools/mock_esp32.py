#!/usr/bin/env python3
# Copyright 2026 Vision Guided Control contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Mock ESP32 - pretends to be the pan/tilt controller for hardware-free tests.

Reads ``X,Y\\n`` packets from a serial port, runs the same pixel->angle mapping
as the real firmware, and prints the resulting servo angles. With ``--ack`` it
answers each packet with ``pan=<P> tilt=<T>`` so the real round trip can be
inspected.

Typical use with a virtual serial pair created by ``socat``::

    socat -d -d pty,raw,echo=0 pty,raw,echo=0
    python tools/mock_esp32.py --port /dev/pts/1
    # then point serial_bridge_node at the other end, e.g. /dev/pts/2
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# Allow running straight from a source checkout without a ROS install.
sys.path.insert(
    0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)), os.pardir,
        'ros2_ws', 'src', 'vision_guided_control',
    )
)

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print('pyserial is not installed. Install it with:\n  pip install pyserial')
    raise

from vision_guided_control.config import ServoConfig
from vision_guided_control.serial_protocol import parse_packet
from vision_guided_control.servo_math import ServoSmoother, clamp, map_point_to_angles


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Emulate the ESP32 pan/tilt controller over serial.'
    )
    parser.add_argument('--port', help='Serial port to listen on.')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate.')
    parser.add_argument('--width', type=int, default=640, help='Frame width.')
    parser.add_argument('--height', type=int, default=480, help='Frame height.')
    parser.add_argument('--ack', action='store_true',
                        help='Reply with the computed angles.')
    parser.add_argument('--list', action='store_true', dest='list_ports',
                        help='List available serial ports and exit.')
    args = parser.parse_args(argv)

    if args.list_ports:
        for port in list_ports.comports():
            print(f'  {port.device:<20} {port.description}')
        return 0

    if not args.port:
        parser.error('--port is required (use --list to see ports).')

    config = ServoConfig()
    smoother = ServoSmoother(config)
    buffer = ''
    last_packet_time = time.monotonic()
    lost_timeout = 1.5
    target_visible = False

    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.1) as ser:
            print(f'Mock ESP32 listening on {args.port} @ {args.baud} baud '
                  f'(frame {args.width}x{args.height}). Ctrl-C to quit.')
            while True:
                chunk = ser.read(64)
                if chunk:
                    buffer += chunk.decode('ascii', errors='replace')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            x, y = parse_packet(line)
                        except ValueError as exc:
                            print(f'  REJECT "{line}": {exc}')
                            continue
                        raw_pan, raw_tilt = map_point_to_angles(
                            x, y, args.width, args.height, config
                        )
                        pan, tilt = smoother.update(raw_pan, raw_tilt)
                        last_packet_time = time.monotonic()
                        target_visible = True
                        print(f'  x={x:>3} y={y:>3} -> '
                              f'pan={pan:6.1f} tilt={tilt:6.1f}')
                        if args.ack:
                            ser.write(
                                f'pan={pan:.0f} tilt={tilt:.0f}\n'.encode('ascii')
                            )
                if target_visible and (
                    time.monotonic() - last_packet_time > lost_timeout
                ):
                    target_visible = False
                    smoother.reset(config.pan_home, config.tilt_home)
                    print('  target lost -> home position')
    except serial.SerialException as exc:
        print(f'Serial error: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\nShutting down mock ESP32.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
