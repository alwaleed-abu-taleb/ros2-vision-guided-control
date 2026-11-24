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

"""ROS 2 node that tracks a coloured object and publishes its position.

Published topic
---------------
``/object_position``  ``geometry_msgs/msg/Point``

+---+-------------------------------------------+
| x | object centroid X in pixels (image space) |
+---+-------------------------------------------+
| y | object centroid Y in pixels (image space) |
+---+-------------------------------------------+
| z | detection state: 1.0 = target found,      |
|   | 0.0 = no target in the current frame      |
+---+-------------------------------------------+

The tracker has no knowledge of the ESP32 or the serial port; it only publishes.
A separate ``serial_bridge_node`` consumes this topic, which keeps the vision
and actuation halves of the system loosely coupled.

Every configuration value below is a ROS 2 parameter, so nothing has to be
edited in source code::

    ros2 run vision_guided_control tracker_node --ros-args \\
        -p camera_index:=0 \\
        -p hsv_lower_h:=100 -p hsv_lower_s:=100 -p hsv_lower_v:=100 \\
        -p hsv_upper_h:=130 -p hsv_upper_s:=255 -p hsv_upper_v:=255 \\
        -p min_contour_area:=300 -p publish_rate:=30.0 -p show_debug:=true
"""

from __future__ import annotations

import math
import time
from typing import Optional

import cv2
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .config import TrackerConfig
from .tracker_core import ColorTracker, TrackerResult, make_test_frame

DEFAULT_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class TrackerNode(Node):
    """Captures frames, runs the OpenCV tracker and publishes ``/object_position``."""

    def __init__(self):
        super().__init__('tracker_node')

        self._declare_parameters()
        self._config = self._build_config()

        self._tracker = ColorTracker(self._config)
        self._cap: Optional[cv2.VideoCapture] = None
        self._camera_ok = False
        self._last_camera_retry = 0.0
        self._camera_retry_interval = 2.0

        # Synthetic-video state (test mode).
        self._test_phase = 0.0

        # FPS bookkeeping used only for the debug overlay.
        self._frame_count = 0
        self._fps_start = time.monotonic()
        self._fps = 0.0

        self._publisher = self.create_publisher(Point, '/object_position', DEFAULT_QOS)

        period = 1.0 / max(self.publish_rate, 1e-3)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            'tracker_node ready: camera_index={} resolution={}x{} '
            'min_area={} rate={}Hz test_video={}'.format(
                self._config.camera_index,
                self._config.frame_width,
                self._config.frame_height,
                self._config.min_contour_area,
                self.publish_rate,
                self.use_test_video,
            )
        )

    # ------------------------------------------------------------------ setup

    def _declare_parameters(self):
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('hsv_lower_h', 100)
        self.declare_parameter('hsv_lower_s', 100)
        self.declare_parameter('hsv_lower_v', 100)
        self.declare_parameter('hsv_upper_h', 130)
        self.declare_parameter('hsv_upper_s', 255)
        self.declare_parameter('hsv_upper_v', 255)
        self.declare_parameter('min_contour_area', 300.0)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('show_debug', False)
        self.declare_parameter('use_test_video', False)

    def _build_config(self) -> TrackerConfig:
        # Node-level switches are kept on the node itself; TrackerConfig only
        # holds the OpenCV algorithm parameters so it stays ROS-independent.
        self.use_test_video = bool(self.get_parameter('use_test_video').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        config = TrackerConfig(
            camera_index=int(self.get_parameter('camera_index').value),
            frame_width=int(self.get_parameter('frame_width').value),
            frame_height=int(self.get_parameter('frame_height').value),
            hsv_lower=(
                int(self.get_parameter('hsv_lower_h').value),
                int(self.get_parameter('hsv_lower_s').value),
                int(self.get_parameter('hsv_lower_v').value),
            ),
            hsv_upper=(
                int(self.get_parameter('hsv_upper_h').value),
                int(self.get_parameter('hsv_upper_s').value),
                int(self.get_parameter('hsv_upper_v').value),
            ),
            min_contour_area=float(self.get_parameter('min_contour_area').value),
            morph_kernel_size=int(self.get_parameter('morph_kernel_size').value),
        )
        if self.publish_rate <= 0 or self.publish_rate > 1000:
            raise ValueError('publish_rate must be in (0, 1000]')
        return config.validate()

    # ------------------------------------------------------------- camera I/O

    def _release_camera(self):
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._camera_ok = False

    def _open_camera(self) -> bool:
        self._release_camera()
        try:
            self._cap = cv2.VideoCapture(self._config.camera_index)
            if not self._cap.isOpened():
                self._release_camera()
                return False
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._config.frame_width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._config.frame_height)
            return True
        except (cv2.error, Exception) as exc:
            self.get_logger().error(f'Failed to open camera: {exc}')
            self._release_camera()
            return False

    def _grab_frame(self):
        if self.use_test_video:
            return self._synthetic_frame()
        return self._camera_frame()

    def _camera_frame(self):
        if self._cap is None or not self._camera_ok:
            now = time.monotonic()
            if now - self._last_camera_retry >= self._camera_retry_interval:
                self._last_camera_retry = now
                self._camera_ok = self._open_camera()
                if self._camera_ok:
                    self.get_logger().info('Camera reconnected.')
            return None
        try:
            ok, frame = self._cap.read()
        except (cv2.error, Exception) as exc:
            self.get_logger().warn(f'Camera read failed: {exc}')
            self._release_camera()
            return None
        if not ok or frame is None:
            self.get_logger().warn('Camera returned an empty frame.')
            self._release_camera()
            return None
        return frame

    def _synthetic_frame(self):
        """Generates an animated blue target so the pipeline runs without a camera."""
        width = self._config.frame_width
        height = self._config.frame_height
        self._test_phase += 0.02
        cx = width / 2.0 + (width / 3.0) * math.sin(self._test_phase)
        cy = height / 2.0 + (height / 4.0) * math.sin(self._test_phase * 0.7)
        return make_test_frame(width, height, cx, cy, radius=35.0)

    # ---------------------------------------------------------------- runtime

    def _on_timer(self):
        frame = self._grab_frame()
        result = self._tracker.process(frame)

        self._publish(result)
        if self.show_debug:
            self._show_debug(frame, result)

    def _publish(self, result: TrackerResult):
        msg = Point()
        msg.x = float(result.x)
        msg.y = float(result.y)
        # z carries the detection state so subscribers can distinguish a real
        # measurement from a stale/empty one without parsing coordinates.
        msg.z = 1.0 if result.detected else 0.0
        self._publisher.publish(msg)

    def _show_debug(self, frame, result: TrackerResult):
        if frame is None:
            return
        self._frame_count += 1
        elapsed = time.monotonic() - self._fps_start
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start = time.monotonic()
        annotated = self._tracker.draw_debug(frame, result)
        hud = (
            f'FPS: {self._fps:.1f}  detected: {result.detected}  '
            f'x: {result.x:.0f}  y: {result.y:.0f}  '
            f'area: {result.area:.0f}  frame: {result.frame_width}x{result.frame_height}'
        )
        cv2.putText(
            annotated, hud, (10, annotated.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
        )
        cv2.imshow('vision_guided_control tracker', annotated)
        cv2.waitKey(1)

    # -------------------------------------------------------------- shutdown

    def destroy_node(self):
        self.get_logger().info('Shutting down tracker_node.')
        self._release_camera()
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
