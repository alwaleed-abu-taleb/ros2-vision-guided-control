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

"""Configuration containers shared by the tracker, the serial bridge and the tools.

Every tunable value lives here so that behaviour can be changed without editing
algorithmic source code.  The ROS 2 nodes expose these same values as ROS 2
parameters; the pure-Python tools build these dataclasses directly.
"""

from __future__ import annotations

from dataclasses import dataclass


def _validate_range(name: str, value: float, low: float, high: float) -> None:
    if not low <= value <= high:
        raise ValueError(
            f'{name}={value} is outside the valid range [{low}, {high}]'
        )


@dataclass
class TrackerConfig:
    """Colour-tracker configuration (OpenCV side, no ROS dependency)."""

    # /dev/videoN index on Linux, USB camera index on macOS/Windows.
    camera_index: int = 0

    # Requested capture resolution. The tracker always reports the *actual*
    # frame dimensions read from the camera, so these are only a hint.
    frame_width: int = 640
    frame_height: int = 480

    # HSV colour threshold. OpenCV encodes hue in [0, 179], saturation and
    # value in [0, 255]. The default targets a saturated blue object.
    hsv_lower: tuple = (100, 100, 100)
    hsv_upper: tuple = (130, 255, 255)

    # Contours smaller than this (in pixels^2) are treated as noise.
    min_contour_area: float = 300.0

    # Morphological cleanup kernel size (pixels). 0 disables morphology.
    morph_kernel_size: int = 5

    def validate(self) -> 'TrackerConfig':
        _validate_range('camera_index', self.camera_index, 0, 63)
        _validate_range('frame_width', self.frame_width, 1, 4096)
        _validate_range('frame_height', self.frame_height, 1, 4096)
        for name in ('hsv_lower', 'hsv_upper'):
            values = getattr(self, name)
            if len(values) != 3:
                raise ValueError(f'{name} must have 3 channels (H, S, V)')
            if not 0 <= values[0] <= 179:
                raise ValueError(f'{name}[0] (Hue) must be in [0, 179]')
            if not (0 <= values[1] <= 255 and 0 <= values[2] <= 255):
                raise ValueError(f'{name} S/V channels must be in [0, 255]')
        if any(lo > hi for lo, hi in zip(self.hsv_lower, self.hsv_upper)):
            raise ValueError('hsv_lower must be <= hsv_upper on every channel')
        if self.min_contour_area < 0:
            raise ValueError('min_contour_area must be >= 0')
        if not 0 <= self.morph_kernel_size <= 31:
            raise ValueError('morph_kernel_size must be in [0, 31]')
        return self

    def as_dict(self) -> dict:
        return {
            'camera_index': self.camera_index,
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'hsv_lower': list(self.hsv_lower),
            'hsv_upper': list(self.hsv_upper),
            'min_contour_area': self.min_contour_area,
            'morph_kernel_size': self.morph_kernel_size,
        }


@dataclass
class ServoConfig:
    """Pan/tilt servo mapping limits shared by the firmware and the simulator.

    The pixel->angle mapping in ``servo_math`` and the equivalent code in
    ``esp32/pan_tilt_controller/src/main.cpp`` use exactly these same values so
    the simulated behaviour matches the physical hardware.
    """

    # Servo angles are expressed in degrees and must stay inside [0, 180].
    pan_min: float = 0.0
    pan_max: float = 180.0
    tilt_min: float = 0.0
    tilt_max: float = 180.0

    # Safe starting / lost-target position.
    pan_home: float = 90.0
    tilt_home: float = 90.0

    # Proportional gain: fraction of the servo half-range applied for a target
    # at the opposite edge of the frame. 1.0 = full travel frame edge to edge.
    gain_x: float = 1.0
    gain_y: float = 1.0

    # Some pan/tilt mounts are wired backwards; flip the sign to correct.
    invert_pan: bool = False
    invert_tilt: bool = False

    # Ignore target movement smaller than this many pixels (removes jitter).
    deadzone_pixels: float = 15.0

    # Exponential-moving-average factor in [0, 1]: 0 = frozen, 1 = no smoothing.
    smoothing_alpha: float = 0.35

    def validate(self) -> 'ServoConfig':
        for name, low, high in (
            ('pan_min', 0.0, 180.0),
            ('pan_max', 0.0, 180.0),
            ('tilt_min', 0.0, 180.0),
            ('tilt_max', 0.0, 180.0),
            ('pan_home', 0.0, 180.0),
            ('tilt_home', 0.0, 180.0),
        ):
            _validate_range(name, getattr(self, name), low, high)
        if self.pan_min > self.pan_max or self.tilt_min > self.tilt_max:
            raise ValueError('servo minimum angle must be <= maximum angle')
        if not 0.0 < self.gain_x <= 2.0 or not 0.0 < self.gain_y <= 2.0:
            raise ValueError('gains must be in (0, 2]')
        if not 0.0 <= self.smoothing_alpha <= 1.0:
            raise ValueError('smoothing_alpha must be in [0, 1]')
        if self.deadzone_pixels < 0:
            raise ValueError('deadzone_pixels must be >= 0')
        return self
