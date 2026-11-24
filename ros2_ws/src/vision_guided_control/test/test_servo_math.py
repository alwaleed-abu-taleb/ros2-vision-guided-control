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

"""Pixel -> servo-angle mapping tests. They mirror the checks documented in
docs/SERIAL_PROTOCOL.md and the behaviour of the ESP32 firmware."""

import pytest

from vision_guided_control.config import ServoConfig
from vision_guided_control.servo_math import (
    ServoSmoother,
    clamp,
    in_deadzone,
    map_point_to_angles,
    step_closed_loop_angles,
)


WIDTH, HEIGHT = 640, 480


def test_centre_maps_to_home():
    pan, tilt = map_point_to_angles(WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT)
    assert pan == pytest.approx(90.0)
    assert tilt == pytest.approx(90.0)


def test_left_and_right_pan_in_opposite_directions():
    centre_pan, _ = map_point_to_angles(WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT)
    left_pan, _ = map_point_to_angles(0, HEIGHT / 2, WIDTH, HEIGHT)
    right_pan, _ = map_point_to_angles(WIDTH, HEIGHT / 2, WIDTH, HEIGHT)
    assert left_pan < centre_pan < right_pan


def test_up_and_down_tilt_in_opposite_directions():
    _, centre_tilt = map_point_to_angles(WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT)
    _, up_tilt = map_point_to_angles(WIDTH / 2, 0, WIDTH, HEIGHT)
    _, down_tilt = map_point_to_angles(WIDTH / 2, HEIGHT, WIDTH, HEIGHT)
    # A target above the centre (small y) must tilt "up" away from the home angle
    # in the opposite sense from a target below the centre (large y).
    assert up_tilt != centre_tilt
    assert down_tilt != centre_tilt
    assert (up_tilt - centre_tilt) * (down_tilt - centre_tilt) < 0


def test_full_travel_reaches_servo_limits():
    pan, _ = map_point_to_angles(0, HEIGHT / 2, WIDTH, HEIGHT)
    assert pan == pytest.approx(0.0)
    pan, _ = map_point_to_angles(WIDTH, HEIGHT / 2, WIDTH, HEIGHT)
    assert pan == pytest.approx(180.0)


def test_angles_are_always_clamped_to_safe_range():
    config = ServoConfig(pan_min=30.0, pan_max=150.0, tilt_min=20.0, tilt_max=160.0,
                         gain_x=2.0, gain_y=2.0)
    for x in (0, WIDTH / 2, WIDTH):
        for y in (0, HEIGHT / 2, HEIGHT):
            pan, tilt = map_point_to_angles(x, y, WIDTH, HEIGHT, config)
            assert config.pan_min <= pan <= config.pan_max
            assert config.tilt_min <= tilt <= config.tilt_max


def test_out_of_frame_coordinates_stay_safe():
    # A misbehaving publisher must never command an angle outside the limits.
    pan, tilt = map_point_to_angles(-5000, 9000, WIDTH, HEIGHT)
    assert 0.0 <= pan <= 180.0
    assert 0.0 <= tilt <= 180.0


def test_invert_flags_mirror_the_mapping():
    plain_pan, plain_tilt = map_point_to_angles(0, 0, WIDTH, HEIGHT)
    config = ServoConfig(invert_pan=True, invert_tilt=True)
    inv_pan, inv_tilt = map_point_to_angles(0, 0, WIDTH, HEIGHT, config)
    assert inv_pan == pytest.approx(180.0 - plain_pan)
    assert inv_tilt == pytest.approx(180.0 - plain_tilt)


def test_reduced_gain_limits_travel():
    config = ServoConfig(gain_x=0.5, gain_y=0.5)
    pan, tilt = map_point_to_angles(0, 0, WIDTH, HEIGHT, config)
    # Half gain => a corner target reaches half-way to the servo limit.
    assert pan == pytest.approx(45.0)
    assert tilt == pytest.approx(135.0)


def test_deadzone_centre_is_neutral():
    assert in_deadzone(WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT, deadzone_pixels=15)
    assert in_deadzone(WIDTH / 2 + 10, HEIGHT / 2 - 10, WIDTH, HEIGHT, 15)
    assert not in_deadzone(WIDTH / 2 + 100, HEIGHT / 2, WIDTH, HEIGHT, 15)
    assert not in_deadzone(WIDTH / 2, HEIGHT / 2 + 100, WIDTH, HEIGHT, 15)


def test_deadzone_disabled_when_zero():
    assert not in_deadzone(WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT, 0)


def test_clamp():
    assert clamp(-5, 0, 100) == 0
    assert clamp(150, 0, 100) == 100
    assert clamp(42, 0, 100) == 42


def test_config_validation():
    with pytest.raises(ValueError):
        ServoConfig(pan_min=100, pan_max=10).validate()
    with pytest.raises(ValueError):
        ServoConfig(gain_x=0.0).validate()
    with pytest.raises(ValueError):
        ServoConfig(smoothing_alpha=2.0).validate()


class TestServoSmoother:
    def test_starts_at_home(self):
        smoother = ServoSmoother()
        assert smoother.state() == (90.0, 90.0)

    def test_converges_to_command(self):
        smoother = ServoSmoother(ServoConfig(smoothing_alpha=0.5))
        # The very first command snaps into place so the servos do not start
        # from an arbitrary position.
        assert smoother.update(0.0, 180.0) == (0.0, 180.0)

    def test_exponential_moving_average_blends_commands(self):
        smoother = ServoSmoother(ServoConfig(smoothing_alpha=0.5))
        smoother.reset(90.0, 90.0)
        # Half-way toward the commanded value on each update.
        assert smoother.update(0.0, 180.0) == (45.0, 135.0)
        assert smoother.update(0.0, 180.0) == (22.5, 157.5)
        for _ in range(60):
            pan, tilt = smoother.update(0.0, 180.0)
        assert pan == pytest.approx(0.0, abs=1e-3)
        assert tilt == pytest.approx(180.0, abs=1e-3)

    def test_output_stays_within_limits(self):
        smoother = ServoSmoother(ServoConfig(smoothing_alpha=0.9))
        for pan_cmd, tilt_cmd in ((-100.0, 900.0), (400.0, -300.0)):
            pan, tilt = smoother.update(pan_cmd, tilt_cmd)
            assert 0.0 <= pan <= 180.0
            assert 0.0 <= tilt <= 180.0


class TestClosedLoopVisualServoing:
    def test_holds_current_angle_inside_deadzone(self):
        pan, tilt = step_closed_loop_angles(
            WIDTH / 2 + 5, HEIGHT / 2 - 5, WIDTH, HEIGHT,
            current_pan=110.0, current_tilt=75.0,
        )
        assert pan == pytest.approx(110.0)
        assert tilt == pytest.approx(75.0)

    def test_steps_in_target_direction(self):
        # Target to the right and above centre should increase pan and tilt.
        pan, tilt = step_closed_loop_angles(
            WIDTH, 0, WIDTH, HEIGHT,
            current_pan=90.0, current_tilt=90.0, step_deg=4.0,
        )
        assert pan == pytest.approx(94.0)
        assert tilt == pytest.approx(94.0)

    def test_invert_flags_reverse_step_direction(self):
        config = ServoConfig(invert_pan=True, invert_tilt=True)
        pan, tilt = step_closed_loop_angles(
            WIDTH, 0, WIDTH, HEIGHT,
            current_pan=90.0, current_tilt=90.0, step_deg=4.0, config=config,
        )
        assert pan == pytest.approx(86.0)
        assert tilt == pytest.approx(86.0)

    def test_clamps_to_safe_limits(self):
        config = ServoConfig(pan_min=20.0, pan_max=160.0, tilt_min=20.0, tilt_max=160.0)
        pan, tilt = step_closed_loop_angles(
            WIDTH, 0, WIDTH, HEIGHT,
            current_pan=159.0, current_tilt=159.0, step_deg=4.0, config=config,
        )
        assert pan == pytest.approx(160.0)
        assert tilt == pytest.approx(160.0)
