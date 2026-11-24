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

"""Pixel -> servo-angle mapping shared by the simulator and the ESP32 firmware.

The reference implementation of the control law lives here (and is unit
tested); ``esp32/pan_tilt_controller/src/main.cpp`` implements the same
equations in C++ so the physical hardware behaves like the simulation.

Control law (proportional, centred on the frame centre)::

    error_x = target_x - frame_width / 2
    error_y = target_y - frame_height / 2
    pan     = pan_home  + gain_x * (error_x / half_width)  * pan_half_range
    tilt    = tilt_home - gain_y * (error_y / half_height) * tilt_half_range

A target at the frame centre commands the home angle, a target at the left or
right edge commands the corresponding servo limit, and every result is clamped
to the configured safe angle range.
"""

from __future__ import annotations

from typing import Optional, Tuple

from .config import ServoConfig


def clamp(value: float, low: float, high: float) -> float:
    """Returns ``value`` restricted to ``[low, high]``."""
    return max(low, min(high, value))


def in_deadzone(
    x: float,
    y: float,
    frame_width: int,
    frame_height: int,
    deadzone_pixels: float,
) -> bool:
    """True when the target is close enough to centre that the servos should hold."""
    if deadzone_pixels <= 0:
        return False
    error_x = x - frame_width / 2.0
    error_y = y - frame_height / 2.0
    return abs(error_x) <= deadzone_pixels and abs(error_y) <= deadzone_pixels


def map_point_to_angles(
    x: float,
    y: float,
    frame_width: int,
    frame_height: int,
    config: Optional[ServoConfig] = None,
) -> Tuple[float, float]:
    """Maps a target pixel coordinate to (pan, tilt) servo angles in degrees.

    ``x``/``y`` are in image coordinates (origin top-left, y increasing
    downward). The result is always clamped to the configured safe range, so
    malformed input can never command an unsafe angle.
    """
    config = config if config is not None else ServoConfig()
    config.validate()

    if frame_width <= 0 or frame_height <= 0:
        return config.pan_home, config.tilt_home

    half_width = frame_width / 2.0
    half_height = frame_height / 2.0

    pan_half_range = (config.pan_max - config.pan_min) / 2.0
    tilt_half_range = (config.tilt_max - config.tilt_min) / 2.0

    # Normalised, signed offset from the optical centre: -1 .. 1 at the edges.
    offset_x = (x - half_width) / half_width
    offset_y = (y - half_height) / half_height

    pan = config.pan_home + config.gain_x * offset_x * pan_half_range
    # Image y grows downward, but "tilt up" means looking above the horizon,
    # so the tilt term is inverted by default.
    tilt = config.tilt_home - config.gain_y * offset_y * tilt_half_range

    if config.invert_pan:
        pan = config.pan_home - (pan - config.pan_home)
    if config.invert_tilt:
        tilt = config.tilt_home - (tilt - config.tilt_home)

    return (
        clamp(pan, config.pan_min, config.pan_max),
        clamp(tilt, config.tilt_min, config.tilt_max),
    )


def step_closed_loop_angles(
    x: float,
    y: float,
    frame_width: int,
    frame_height: int,
    current_pan: float,
    current_tilt: float,
    step_deg: float = 4.0,
    config: Optional[ServoConfig] = None,
) -> Tuple[float, float]:
    """Incremental visual-servoing step for a camera mounted on the pan/tilt head.

    Unlike :func:`map_point_to_angles` (open-loop fixed-camera mapping), this
    adjusts the current servo angles proportionally to the target offset from the
    optical centre and holds the current angles inside the deadzone.
    """
    config = config if config is not None else ServoConfig()
    config.validate()

    if frame_width <= 0 or frame_height <= 0 or step_deg <= 0:
        return (
            clamp(current_pan, config.pan_min, config.pan_max),
            clamp(current_tilt, config.tilt_min, config.tilt_max),
        )

    half_width = frame_width / 2.0
    half_height = frame_height / 2.0
    error_x = x - half_width
    error_y = y - half_height

    pan = current_pan
    tilt = current_tilt

    if abs(error_x) > config.deadzone_pixels:
        delta_pan = config.gain_x * (error_x / half_width) * step_deg
        if config.invert_pan:
            delta_pan = -delta_pan
        pan += delta_pan

    if abs(error_y) > config.deadzone_pixels:
        delta_tilt = -config.gain_y * (error_y / half_height) * step_deg
        if config.invert_tilt:
            delta_tilt = -delta_tilt
        tilt += delta_tilt

    return (
        clamp(pan, config.pan_min, config.pan_max),
        clamp(tilt, config.tilt_min, config.tilt_max),
    )


class ServoSmoother:
    """Exponential moving average filter for servo commands.

    Smoothing removes the small frame-to-frame jumps of a noisy centroid and
    keeps the mechanical movement slow enough to be safe and watchable.
    """

    def __init__(
        self,
        config: Optional[ServoConfig] = None,
        alpha: Optional[float] = None,
    ):
        self.config = config if config is not None else ServoConfig()
        self.alpha = self.config.smoothing_alpha if alpha is None else alpha
        self._pan = self.config.pan_home
        self._tilt = self.config.tilt_home
        self._initialised = False

    def reset(self, pan: Optional[float] = None, tilt: Optional[float] = None):
        self._pan = self.config.pan_home if pan is None else pan
        self._tilt = self.config.tilt_home if tilt is None else tilt
        self._initialised = True

    def update(self, pan: float, tilt: float) -> Tuple[float, float]:
        """Fuses a new raw command into the smoothed output."""
        if not self._initialised:
            self.reset(pan, tilt)
        self._pan += self.alpha * (pan - self._pan)
        self._tilt += self.alpha * (tilt - self._tilt)
        return self.state()

    def state(self) -> Tuple[float, float]:
        return (
            clamp(self._pan, self.config.pan_min, self.config.pan_max),
            clamp(self._tilt, self.config.tilt_min, self.config.tilt_max),
        )
