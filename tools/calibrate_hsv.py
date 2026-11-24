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

"""Interactive HSV threshold calibrator for ``tracker_node``.

Opens the webcam and displays live trackbars for ``H_min/max``, ``S_min/max``,
and ``V_min/max`` alongside the detected contour centroid and binary mask.
Press ``p`` to print the ready-to-run ``ros2 run`` command at any time, or
``q`` to quit and print the final command.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np


def _noop(_value: int) -> None:
    pass


def format_ros2_command(
    h_min: int, s_min: int, v_min: int,
    h_max: int, s_max: int, v_max: int,
) -> str:
    """Returns a copy-pasteable ``ros2 run`` command with the calibrated HSV bounds."""
    return (
        'ros2 run vision_guided_control tracker_node --ros-args '
        f'-p hsv_lower_h:={h_min} -p hsv_lower_s:={s_min} -p hsv_lower_v:={v_min} '
        f'-p hsv_upper_h:={h_max} -p hsv_upper_s:={s_max} -p hsv_upper_v:={v_max} '
        '-p show_debug:=true'
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Calibrate HSV colour thresholds interactively with OpenCV.'
    )
    parser.add_argument('--camera', type=int, default=0, help='Camera index.')
    parser.add_argument('--width', type=int, default=640, help='Capture width.')
    parser.add_argument('--height', type=int, default=480, help='Capture height.')
    args = parser.parse_args(argv)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f'Error: cannot open camera index {args.camera}', file=sys.stderr)
        return 1

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    window_controls = 'HSV Calibrator'
    window_mask = 'Mask'
    cv2.namedWindow(window_controls)
    cv2.namedWindow(window_mask)

    cv2.createTrackbar('H_min', window_controls, 100, 179, _noop)
    cv2.createTrackbar('H_max', window_controls, 130, 179, _noop)
    cv2.createTrackbar('S_min', window_controls, 100, 255, _noop)
    cv2.createTrackbar('S_max', window_controls, 255, 255, _noop)
    cv2.createTrackbar('V_min', window_controls, 100, 255, _noop)
    cv2.createTrackbar('V_max', window_controls, 255, 255, _noop)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    bounds = (100, 100, 100, 130, 255, 255)

    print('Adjust sliders until only the target is white in the Mask window.')
    print('Press "p" to print the ROS 2 command, or "q" to quit.')

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print('Warning: empty frame from camera.', file=sys.stderr)
                break

            h_min = cv2.getTrackbarPos('H_min', window_controls)
            h_max = max(h_min, cv2.getTrackbarPos('H_max', window_controls))
            s_min = cv2.getTrackbarPos('S_min', window_controls)
            s_max = max(s_min, cv2.getTrackbarPos('S_max', window_controls))
            v_min = cv2.getTrackbarPos('V_min', window_controls)
            v_max = max(v_min, cv2.getTrackbarPos('V_max', window_controls))
            bounds = (h_min, s_min, v_min, h_max, s_max, v_max)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(
                hsv,
                np.array([h_min, s_min, v_min], dtype=np.uint8),
                np.array([h_max, s_max, v_max], dtype=np.uint8),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            annotated = frame.copy()
            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area >= 300.0:
                    m = cv2.moments(largest)
                    if m['m00'] > 0:
                        cx = int(m['m10'] / m['m00'])
                        cy = int(m['m01'] / m['m00'])
                        cv2.circle(annotated, (cx, cy), 12, (0, 255, 0), 2)
                        cv2.putText(
                            annotated,
                            f'({cx}, {cy}) area={area:.0f}',
                            (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )

            cv2.imshow(window_controls, annotated)
            cv2.imshow(window_mask, mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('p'):
                print(format_ros2_command(*bounds))
            elif key == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print('\nCalibrated command:')
    print(format_ros2_command(*bounds))
    return 0


if __name__ == '__main__':
    sys.exit(main())
