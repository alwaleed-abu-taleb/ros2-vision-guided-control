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

"""Bridge logic tests using a fake serial port - no ROS and no ESP32 required.

``serial_bridge_node`` is imported with ``rclpy`` stubbed out, so the pure
message-handling parts (packet formatting, lost-target withholding, write
failure recovery) can be exercised anywhere.
"""

import types

import pytest

rclpy_stub = types.ModuleType('rclpy')


class _NodeBase:
    """Minimal stand-in for ``rclpy.node.Node`` used by the bridge."""

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._log = []
        self.subscriptions = []
        self.timers = []

    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback))

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        logger = self

        class _Logger:
            def _log(self, level, msg, **kwargs):
                logger._log.append((level, msg))

            def info(self, msg, **kw):
                self._log('info', msg, **kw)

            def warning(self, msg, **kw):
                self._log('warning', msg, **kw)

            def error(self, msg, **kw):
                self._log('error', msg, **kw)

            def debug(self, msg, **kw):
                self._log('debug', msg, **kw)

        return _Logger()


node_stub = types.ModuleType('rclpy.node')
node_stub.Node = _NodeBase
rclpy_stub.node = node_stub
rclpy_stub.init = lambda *a, **kw: None
rclpy_stub.spin = lambda *a, **kw: None
rclpy_stub.try_shutdown = lambda *a, **kw: None

geometry_stub = types.ModuleType('geometry_msgs')
msg_stub = types.ModuleType('geometry_msgs.msg')


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


msg_stub.Point = Point
geometry_stub.msg = msg_stub

qos_stub = types.ModuleType('rclpy.qos')


class _QoS:
    def __init__(self, **kw):
        pass


qos_stub.QoSProfile = _QoS
qos_stub.QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
rclpy_stub.qos = qos_stub

sys_modules = {}
for name, module in (
    ('rclpy', rclpy_stub),
    ('rclpy.node', node_stub),
    ('geometry_msgs', geometry_stub),
    ('geometry_msgs.msg', msg_stub),
    ('rclpy.qos', qos_stub),
):
    sys_modules[name] = module

import sys

for name, module in sys_modules.items():
    sys.modules[name] = module

from vision_guided_control.serial_bridge_node import SerialBridgeNode  # noqa: E402


class FakeSerial:
    """Records everything written and can be made to fail on demand."""

    def __init__(self, *args, **kwargs):
        self.written = []
        self.fail_on_write = False
        self.closed = False

    def write(self, data):
        if self.fail_on_write:
            raise OSError('device disconnected')
        self.written.append(data)
        return len(data)

    def close(self):
        self.closed = True


@pytest.fixture
def bridge():
    node = SerialBridgeNode()
    fake = FakeSerial()
    node._serial = fake
    node._connected = True
    node.serial_port = '/dev/fake'
    return node, fake


def _dispatch(bridge_node, msg):
    topic, callback = bridge_node.subscriptions[0][1], bridge_node.subscriptions[0][2]
    assert topic == '/object_position'
    callback(msg)


def test_valid_message_is_formatted_and_sent(bridge):
    node, fake = bridge
    _dispatch(node, Point(320, 240, 1.0))
    assert fake.written == [b'320,240\n']


def test_lost_target_is_withheld(bridge):
    node, fake = bridge
    _dispatch(node, Point(320, 240, 0.0))
    assert fake.written == []


def test_out_of_range_message_is_dropped(bridge):
    node, fake = bridge
    _dispatch(node, Point(-10, 99999, 1.0))
    assert fake.written == []
    assert any(level == 'warning' for level, _ in node._log)


def test_write_failure_disconnects_without_raising(bridge):
    node, fake = bridge
    fake.fail_on_write = True
    _dispatch(node, Point(100, 100, 1.0))
    assert node._connected is False
    assert fake.closed is True


def test_dropped_link_buffers_messages_silently(bridge):
    node, fake = bridge
    fake.fail_on_write = True
    _dispatch(node, Point(100, 100, 1.0))
    # A second message while disconnected must not raise.
    _dispatch(node, Point(200, 200, 1.0))
    assert node._connected is False


def test_rate_limiting_drops_excess_messages(bridge):
    node, fake = bridge
    node.max_send_rate_hz = 10.0
    _dispatch(node, Point(100, 100, 1.0))
    _dispatch(node, Point(200, 200, 1.0))
    assert fake.written == [b'100,100\n']


def test_rate_limiting_can_be_disabled(bridge):
    node, fake = bridge
    node.max_send_rate_hz = 0.0
    _dispatch(node, Point(100, 100, 1.0))
    _dispatch(node, Point(200, 200, 1.0))
    assert fake.written == [b'100,100\n', b'200,200\n']
