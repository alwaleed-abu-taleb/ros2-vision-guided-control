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

"""Hardware-independent colour tracker built on OpenCV.

This module deliberately contains no ROS code so it can be unit tested with a
plain ``pytest`` run and no camera::

    from vision_guided_control.tracker_core import ColorTracker, make_test_frame

    tracker = ColorTracker()
    result = tracker.process(make_test_frame(640, 480, 320, 240, 40))

The pipeline implemented here is::

    BGR frame -> HSV -> colour threshold -> morphology -> contours
              -> largest contour -> centroid (cx, cy)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .config import TrackerConfig


@dataclass
class TrackerResult:
    """Outcome of processing one frame.

    ``x``/``y`` are the object centroid in pixel coordinates; they are only
    meaningful when ``detected`` is True.  ``frame_width``/``frame_height`` are
    the dimensions of the frame that was actually processed (never an
    assumption), and are 0 for an invalid frame.
    """

    detected: bool = False
    x: float = 0.0
    y: float = 0.0
    area: float = 0.0
    frame_width: int = 0
    frame_height: int = 0

    def as_tuple(self) -> tuple:
        return (self.x, self.y)


class ColorTracker:
    """Detects the largest blob of a configured colour and returns its centroid."""

    def __init__(self, config: Optional[TrackerConfig] = None):
        self.config = config if config is not None else TrackerConfig()
        self._lower = np.array(self.config.hsv_lower, dtype=np.uint8)
        self._upper = np.array(self.config.hsv_upper, dtype=np.uint8)
        self._kernel = None
        if self.config.morph_kernel_size > 0:
            size = self.config.morph_kernel_size
            # An even kernel is invalid for morphology, fall back to odd sizes.
            if size % 2 == 0:
                size += 1
            self._kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (size, size)
            )

    def _threshold(self, hsv: np.ndarray) -> np.ndarray:
        mask = cv2.inRange(hsv, self._lower, self._upper)
        if self._kernel is not None:
            # Erode then dilate removes speckle; dilate then erode fills holes.
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        return mask

    def _largest_contour(self, mask: np.ndarray):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None
        best = None
        best_area = self.config.min_contour_area
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= best_area:
                best, best_area = contour, area
        return best

    def process(self, frame: Optional[np.ndarray]) -> TrackerResult:
        """Run the detection pipeline on a single BGR frame.

        Never raises on bad input: an invalid, empty or object-free frame
        simply yields a ``TrackerResult`` with ``detected=False``.
        """
        if not self._is_valid_frame(frame):
            return TrackerResult()

        height, width = frame.shape[0], frame.shape[1]
        result = TrackerResult(frame_width=width, frame_height=height)

        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = self._threshold(hsv)
            contour = self._largest_contour(mask)
        except cv2.error:
            return result

        if contour is None:
            return result

        moments = cv2.moments(contour)
        # Guard against division by zero for degenerate (zero-area) contours.
        if moments['m00'] == 0:
            return result

        result.detected = True
        result.x = float(moments['m10'] / moments['m00'])
        result.y = float(moments['m01'] / moments['m00'])
        result.area = float(moments['m00'])
        return result

    @staticmethod
    def _is_valid_frame(frame) -> bool:
        if frame is None:
            return False
        if not isinstance(frame, np.ndarray):
            return False
        if frame.size == 0 or frame.ndim != 3 or frame.shape[2] != 3:
            return False
        return True

    def draw_debug(self, frame: np.ndarray, result: TrackerResult) -> np.ndarray:
        """Returns an annotated copy of ``frame`` for the optional debug window."""
        annotated = frame.copy()
        if result.detected:
            cv2.circle(
                annotated, (int(result.x), int(result.y)), 12, (0, 255, 0), 2
            )
            cv2.line(
                annotated,
                (int(result.x) - 18, int(result.y)),
                (int(result.x) + 18, int(result.y)),
                (0, 255, 0),
                1,
            )
            cv2.line(
                annotated,
                (int(result.x), int(result.y) - 18),
                (int(result.x), int(result.y) + 18),
                (0, 255, 0),
                1,
            )
            label = f'({result.x:.0f}, {result.y:.0f}) area={result.area:.0f}'
            cv2.putText(
                annotated, label, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )
        else:
            cv2.putText(
                annotated, 'NO TARGET', (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
            )
        return annotated


def make_test_frame(
    width: int = 640,
    height: int = 480,
    cx: float = 320.0,
    cy: float = 240.0,
    radius: float = 30.0,
    color_bgr: tuple = (255, 0, 0),
    background_bgr: tuple = (30, 30, 30),
) -> np.ndarray:
    """Synthetic BGR frame containing one filled coloured circle.

    Used by the unit tests and by the tracker node's ``use_test_video`` mode,
    which lets the whole pipeline be exercised without a physical camera.
    """
    frame = np.full((height, width, 3), np.array(background_bgr), dtype=np.uint8)
    cv2.circle(frame, (int(cx), int(cy)), int(radius), color_bgr, thickness=-1)
    return frame
