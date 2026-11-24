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

"""Unit tests for the OpenCV tracker core. No camera and no ROS required."""

import numpy as np
import pytest

from vision_guided_control.config import TrackerConfig
from vision_guided_control.tracker_core import (
    ColorTracker,
    TrackerResult,
    make_test_frame,
)


BLUE_BGR = (255, 0, 0)
RED_BGR = (0, 0, 255)


@pytest.fixture
def tracker():
    return ColorTracker(TrackerConfig())


def test_detects_centered_object(tracker):
    frame = make_test_frame(640, 480, 320, 240, 40, color_bgr=BLUE_BGR)
    result = tracker.process(frame)
    assert result.detected
    assert result.frame_width == 640
    assert result.frame_height == 480
    assert abs(result.x - 320) < 3
    assert abs(result.y - 240) < 3
    assert result.area > 0


def test_detects_off_center_object(tracker):
    frame = make_test_frame(640, 480, 150, 400, 25, color_bgr=BLUE_BGR)
    result = tracker.process(frame)
    assert result.detected
    assert abs(result.x - 150) < 3
    assert abs(result.y - 400) < 3


def test_no_object_in_frame(tracker):
    frame = make_test_frame(640, 480, 320, 240, 40, color_bgr=RED_BGR)
    result = tracker.process(frame)
    assert not result.detected
    assert result.frame_width == 640
    assert result.frame_height == 480


def test_object_below_min_contour_area_is_rejected():
    config = TrackerConfig(min_contour_area=5000.0)
    tracker = ColorTracker(config)
    # A small blue dot is far below the 5000 px^2 threshold.
    frame = make_test_frame(640, 480, 320, 240, 5, color_bgr=BLUE_BGR)
    assert not tracker.process(frame).detected


def test_largest_contour_is_selected_when_multiple_present(tracker):
    frame = make_test_frame(640, 480, 100, 100, 15, color_bgr=BLUE_BGR)
    # Paint a second, much larger blob elsewhere.
    import cv2
    cv2.circle(frame, (500, 350), 60, BLUE_BGR, thickness=-1)
    result = tracker.process(frame)
    assert result.detected
    assert abs(result.x - 500) < 5
    assert abs(result.y - 350) < 5


def test_invalid_frames_are_handled(tracker):
    for bad in (None, np.array([]), np.zeros((10, 10)), np.zeros((10, 10, 4)), 'frame'):
        result = tracker.process(bad)
        assert isinstance(result, TrackerResult)
        assert not result.detected
        assert result.frame_width == 0


def test_zero_moment_does_not_divide_by_zero():
    # A single-pixel "blob" has a degenerate contour with m00 == 0; the tracker
    # must treat it as undetected rather than raising ZeroDivisionError.
    frame = np.zeros((20, 20, 3), dtype=np.uint8)
    frame[5, 5] = BLUE_BGR
    tracker = ColorTracker(TrackerConfig(min_contour_area=0.0))
    result = tracker.process(frame)
    assert isinstance(result, TrackerResult)


def test_centroid_uses_image_moments():
    result = ColorTracker().process(
        make_test_frame(640, 480, 400, 120, 30, color_bgr=BLUE_BGR)
    )
    # m10/m00 and m01/m00 of a filled circle equal its centre.
    assert result.detected
    assert abs(result.x - 400) < 3
    assert abs(result.y - 120) < 3


def test_actual_frame_dimensions_are_reported_not_configured():
    config = TrackerConfig(frame_width=1920, frame_height=1080)
    tracker = ColorTracker(config)
    frame = make_test_frame(320, 240, 160, 120, 20, color_bgr=BLUE_BGR)
    result = tracker.process(frame)
    assert result.frame_width == 320
    assert result.frame_height == 240


def test_custom_hsv_threshold_detects_different_colour():
    config = TrackerConfig(
        hsv_lower=(0, 100, 100), hsv_upper=(10, 255, 255), min_contour_area=100.0
    )
    tracker = ColorTracker(config)
    frame = make_test_frame(640, 480, 320, 240, 40, color_bgr=RED_BGR)
    result = tracker.process(frame)
    assert result.detected


def test_config_validation_rejects_bad_values():
    with pytest.raises(ValueError):
        TrackerConfig(camera_index=-1).validate()
    with pytest.raises(ValueError):
        TrackerConfig(hsv_lower=(300, 0, 0)).validate()
    with pytest.raises(ValueError):
        TrackerConfig(hsv_upper=(180, 255, 255)).validate()
    with pytest.raises(ValueError):
        TrackerConfig(hsv_lower=(130, 100, 100), hsv_upper=(100, 255, 255)).validate()
    with pytest.raises(ValueError):
        TrackerConfig(min_contour_area=-5).validate()


def test_debug_annotation_does_not_mutate_frame(tracker):
    frame = make_test_frame(640, 480, 320, 240, 40, color_bgr=BLUE_BGR)
    before = frame.copy()
    annotated = tracker.draw_debug(frame, result=tracker.process(frame))
    assert annotated.shape == frame.shape
    assert np.array_equal(frame, before)
