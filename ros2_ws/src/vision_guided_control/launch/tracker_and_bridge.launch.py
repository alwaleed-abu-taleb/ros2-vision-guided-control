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

"""Launches the tracker and the serial bridge together.

Examples::

    # Full system with a camera and an ESP32 on /dev/ttyUSB0
    ros2 launch vision_guided_control tracker_and_bridge.launch.py \\
        serial_port:=/dev/ttyUSB0

    # Bring everything up without any hardware (synthetic camera, no serial port)
    ros2 launch vision_guided_control tracker_and_bridge.launch.py \\
        use_test_video:=true show_debug:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='',
        description='Serial port of the ESP32 (empty = auto-detect).'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate.'
    )
    use_test_video_arg = DeclareLaunchArgument(
        'use_test_video', default_value='false',
        description='Use a synthetic animated target instead of a real camera.'
    )
    show_debug_arg = DeclareLaunchArgument(
        'show_debug', default_value='false',
        description='Show the OpenCV debug window with FPS and centroid.'
    )
    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='Camera index (0 = first camera).'
    )
    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area', default_value='300.0',
        description='Minimum contour area in pixels^2 to be considered a target.'
    )

    tracker_node = Node(
        package='vision_guided_control',
        executable='tracker_node',
        name='tracker_node',
        parameters=[{
            'camera_index': ParameterValue(
                LaunchConfiguration('camera_index'), value_type=int
            ),
            'use_test_video': ParameterValue(
                LaunchConfiguration('use_test_video'), value_type=bool
            ),
            'show_debug': ParameterValue(
                LaunchConfiguration('show_debug'), value_type=bool
            ),
            'min_contour_area': ParameterValue(
                LaunchConfiguration('min_contour_area'), value_type=float
            ),
        }],
        output='screen',
    )

    serial_bridge_node = Node(
        package='vision_guided_control',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': ParameterValue(
                LaunchConfiguration('baud_rate'), value_type=int
            ),
        }],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        use_test_video_arg,
        show_debug_arg,
        camera_index_arg,
        min_contour_area_arg,
        tracker_node,
        serial_bridge_node,
    ])
