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

"""ROS 2 node that forwards tracked positions to the ESP32 over USB serial.

Subscribed topic
----------------
``/object_position``  ``geometry_msgs/msg/Point`` (published by ``tracker_node``)

Each message is converted to the wire protocol ``X,Y\\n`` and written to the
serial port. The bridge never maps coordinates to servo angles itself -- the
ESP32 owns the control law -- so it stays a thin, replaceable transport layer.

The port is never hard-coded. If ``serial_port`` is empty the bridge auto-detects
the first USB/UART adapter it finds, and it retries forever instead of crashing
when the ESP32 is unplugged::

    ros2 run vision_guided_control serial_bridge_node --ros-args \\
        -p serial_port:=/dev/ttyUSB0 \\
        -p baud_rate:=115200
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .serial_protocol import format_packet

DEFAULT_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


def _list_candidate_ports():
    """Returns serial device paths likely to be an ESP32, most promising first."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    candidates = []
    for port in list_ports.comports():
        # USB-serial adapters (CP210x, CH340, FTDI) expose a USB VID:PID,
        # while built-in UARTs and Bluetooth SPP ports do not.
        if port.vid is None or port.pid is None:
            continue
        if 'bluetooth' in (port.description or '').lower():
            continue
        candidates.append(port.device)
    return candidates


class SerialBridgeNode(Node):
    """Bridges ``/object_position`` to a USB serial link."""

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('connect_retry_sec', 2.0)
        self.declare_parameter('send_raw_when_lost', False)
        self.declare_parameter('max_send_rate_hz', 30.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.retry_interval = float(self.get_parameter('connect_retry_sec').value)
        self.send_raw_when_lost = bool(
            self.get_parameter('send_raw_when_lost').value
        )
        self.max_send_rate_hz = float(
            self.get_parameter('max_send_rate_hz').value
        )

        self._serial = None
        self._lock = threading.Lock()
        self._connected = False
        self._last_send_time = 0.0

        self.create_subscription(
            Point, '/object_position', self._on_position, DEFAULT_QOS
        )
        self.create_timer(self.retry_interval, self._maintain_connection)

        self.get_logger().info(
            'serial_bridge_node ready: port={} baud={} (empty port = auto-detect)'.format(
                self.serial_port or 'auto', self.baud_rate
            )
        )
        self._maintain_connection()

    # ------------------------------------------------------------- connection

    def _resolve_port(self) -> Optional[str]:
        if self.serial_port:
            return str(self.serial_port)
        candidates = _list_candidate_ports()
        if candidates:
            self.get_logger().info(f'Auto-detected serial port: {candidates[0]}')
        return candidates[0] if candidates else None

    def _maintain_connection(self):
        with self._lock:
            if self._connected:
                return
        port = self._resolve_port()
        if not port:
            self.get_logger().warning(
                'No serial port found. Connect the ESP32 and it will be '
                'detected automatically.',
                throttle_duration_sec=5.0,
            )
            return
        try:
            import serial
        except ImportError:
            self.get_logger().error('pyserial is not installed.')
            return
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout,
            )
            with self._lock:
                self._connected = True
            self.get_logger().info(f'Connected to {port} @ {self.baud_rate} baud.')
        except Exception as exc:
            # Permission denied, port in use, device unplugged: retry later.
            self.get_logger().warning(
                f'Cannot open {port}: {exc}. Retrying in '
                f'{self.retry_interval}s.',
                throttle_duration_sec=5.0,
            )
            self._serial = None

    # --------------------------------------------------------------- messages

    def _on_position(self, msg: Point):
        if not msg.z and not self.send_raw_when_lost:
            # Target lost: withhold the command so the ESP32 keeps its current
            # position and later returns to its home angle on its own timeout.
            return
        try:
            packet = format_packet(msg.x, msg.y)
        except ValueError as exc:
            self.get_logger().warning(f'Dropping out-of-range coordinate: {exc}')
            return
        if self.max_send_rate_hz > 0.0:
            now = time.monotonic()
            min_interval = 1.0 / self.max_send_rate_hz
            if (
                self._last_send_time > 0.0
                and (now - self._last_send_time) < min_interval
            ):
                return
            self._last_send_time = now
        self._send(packet)

    def _send(self, packet: str):
        with self._lock:
            if not self._connected or self._serial is None:
                return
            try:
                self._serial.write(packet.encode('ascii'))
            except Exception as exc:
                # Broken pipe, device unplugged, USB suspend: drop the link and
                # let the reconnect timer bring it back.
                self.get_logger().warning(f'Serial write failed: {exc}')
                self._close_locked()

    def _close_locked(self):
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        finally:
            self._serial = None
            self._connected = False

    # -------------------------------------------------------------- shutdown

    def destroy_node(self):
        self.get_logger().info('Shutting down serial_bridge_node.')
        with self._lock:
            self._close_locked()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
