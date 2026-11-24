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

"""Serial test utility - exercise the ESP32 pan/tilt controller WITHOUT ROS.

This tool needs only ``pyserial``, so it is the fastest way to verify the
physical servo system before trusting the full vision pipeline.

List the ports the ESP32 might be on::

    python tools/serial_test.py --list

Send a sweep of test coordinates to the pan/tilt controller::

    python tools/serial_test.py --port /dev/ttyUSB0

Send exact coordinates (frame size defaults to 640x480, use --width/--height
to match the tracker)::

    python tools/serial_test.py --port COM3 --points 320,240 100,100 560,420

Interactive mode - type "x,y" and press Enter::

    python tools/serial_test.py --port /dev/ttyUSB0 --interactive
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print(
        'pyserial is not installed. Install it with:\n'
        '  pip install pyserial'
    )
    raise


def list_serial_ports() -> None:
    print('Available serial ports:')
    ports = list(list_ports.comports())
    if not ports:
        print('  (none found)')
        return
    for port in ports:
        marker = ''
        if port.vid is not None and port.pid is not None:
            marker = '  <-- USB device (likely ESP32)'
        print(f'  {port.device:<20} {port.description}{marker}')


def open_port(port: str, baud: int, timeout: float):
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def send_point(ser, x: float, y: float, args) -> None:
    packet = f'{int(round(x))},{int(round(y))}\n'
    ser.write(packet.encode('ascii'))
    print(f'  -> {packet.strip()}')
    if args.readback:
        time.sleep(0.05)
        while ser.in_waiting:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line:
                print(f'  <- {line}')


def interactive_mode(ser, args) -> None:
    width, height = args.width, args.height
    print(f'Interactive mode ({width}x{height}). Type "x,y" then Enter, '
          f'"c" for centre, "q" to quit.')
    while True:
        try:
            raw = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw or raw.lower() == 'q':
            break
        if raw.lower() == 'c':
            send_point(ser, width / 2.0, height / 2.0, args)
            continue
        try:
            x, y = (float(v) for v in raw.split(','))
        except ValueError:
            print('  expected "x,y", e.g. 320,240')
            continue
        send_point(ser, x, y, args)


def preset_sweep(ser, args) -> None:
    """Visits the centre then each corner so servo direction can be checked."""
    width, height = args.width, args.height
    cx, cy = width / 2.0, height / 2.0
    sequence = [
        ('centre', cx, cy),
        ('left', 0.0, cy),
        ('right', width - 1.0, cy),
        ('top', cx, 0.0),
        ('bottom', cx, height - 1.0),
        ('top-left', 0.0, 0.0),
        ('bottom-right', width - 1.0, height - 1.0),
        ('centre', cx, cy),
    ]
    for name, x, y in sequence:
        print(f'{name}:')
        send_point(ser, x, y, args)
        time.sleep(args.interval)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Send test coordinates to the ESP32 pan/tilt controller.'
    )
    parser.add_argument('--port', help='Serial port (e.g. /dev/ttyUSB0 or COM3).')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate.')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='Seconds between preset points.')
    parser.add_argument('--width', type=int, default=640,
                        help='Frame width the ESP32 firmware expects.')
    parser.add_argument('--height', type=int, default=480,
                        help='Frame height the ESP32 firmware expects.')
    parser.add_argument('--interactive', action='store_true',
                        help='Type coordinates manually instead of the sweep.')
    parser.add_argument('--points', nargs='+', metavar='X,Y',
                        help='Send exactly these coordinates, e.g. 320,240 100,100.')
    parser.add_argument('--readback', action='store_true',
                        help='Print any acknowledgement lines sent by the ESP32.')
    parser.add_argument('--list', action='store_true', dest='list_ports',
                        help='List available serial ports and exit.')
    args = parser.parse_args(argv)

    if args.list_ports:
        list_serial_ports()
        return 0

    if not args.port:
        parser.error('--port is required (use --list to see available ports).')

    try:
        with open_port(args.port, args.baud, timeout=0.5) as ser:
            print(f'Opened {args.port} @ {args.baud} baud '
                  f'(frame {args.width}x{args.height}).')
            if args.points:
                for raw in args.points:
                    try:
                        x, y = (float(v) for v in raw.split(','))
                    except ValueError:
                        print(f'  skipping malformed point "{raw}"')
                        continue
                    send_point(ser, x, y, args)
                    time.sleep(args.interval)
            elif args.interactive:
                interactive_mode(ser, args)
            else:
                preset_sweep(ser, args)
    except serial.SerialException as exc:
        print(f'Serial error: {exc}', file=sys.stderr)
        return 1
    except PermissionError as exc:
        print(f'Permission denied: {exc}\n'
              'On Linux, add your user to the dialout group and re-login, or '
              'run with sudo.', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
