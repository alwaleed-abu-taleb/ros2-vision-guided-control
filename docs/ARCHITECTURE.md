# Architecture

End-to-end pipeline:

```text
Camera ──► OpenCV colour tracker ──► /object_position ──► Serial bridge ──► USB serial ──► ESP32 ──► Pan/Tilt servos
 (BGR)      (HSV threshold +           geometry_msgs/     serial_bridge_node    "X,Y\n"      pan_tilt_controller
             contours)                 Point
```

## Design principles

1. **Loose coupling.** The tracker never mentions the ESP32 and the bridge never
   imports OpenCV. They only share the topic, so either half can be replaced,
   tested or replayed independently.
2. **No hardware knowledge in the algorithms.** Colour detection lives in
   `tracker_core.py` and the pixel->angle control law lives in `servo_math.py`.
   Both are plain Python with no ROS and no camera dependency, so they are unit
   tested with `pytest` alone.
3. **Configuration, not source edits.** Every tunable value is a ROS 2 parameter
   on the node that owns it, or a `#define` at the top of the firmware.
4. **Fail soft.** A missing camera, a missing serial port or a malformed packet
   is logged and tolerated; only genuinely unreachable hardware blocks operation.

## ROS 2 graph

```text
+---------------------+        /object_position        +---------------------+
|   tracker_node      | ----------------------------►  |  serial_bridge_node |
| (vision_guided_     |   geometry_msgs/msg/Point      |                     |
|  control)           |                                |  USB serial ──► ESP32|
+---------------------+                                +---------------------+
        ▲                                                       │
        │ frames from cv2.VideoCapture                          ▼
     camera                                                 pan/tilt servos
```

### `/object_position` semantics

| Field | Type  | Meaning                                                     |
|-------|-------|-------------------------------------------------------------|
| `x`   | float | Object centroid X in pixels (image space, left = 0).        |
| `y`   | float | Object centroid Y in pixels (image space, top = 0).         |
| `z`   | float | Detection state: `1.0` = target found, `0.0` = not found.  |

`z` lets subscribers tell a real measurement from an empty frame without
parsing coordinates. The serial bridge withholds messages whose `z` is `0.0`,
so the ESP32 keeps its last command until its own lost-target timer expires and
it returns to the home position.

## Module responsibilities

| Module                            | Responsibility                                             |
|-----------------------------------|------------------------------------------------------------|
| `vision_guided_control/config.py` | Dataclasses + validation for tracker and servo settings.   |
| `vision_guided_control/tracker_core.py` | OpenCV pipeline: HSV threshold, morphology, contours, centroid. Returns a `TrackerResult`. |
| `vision_guided_control/tracker_node.py` | ROS node: camera I/O, parameters, publishing, debug window, synthetic test mode. |
| `vision_guided_control/serial_protocol.py` | `format_packet` / `parse_packet` for the `X,Y\n` wire format. |
| `vision_guided_control/serial_bridge_node.py` | ROS node: subscribes, serialises, reconnects, writes safely. |
| `vision_guided_control/servo_math.py` | Reference pixel->angle control laws (open-loop mapping + closed-loop visual servoing), dead-zone, clamping, EMA smoothing. |
| `esp32/pan_tilt_controller/src/main.cpp` | Firmware: parses the same protocol, runs the same control law, drives the servos. |
| `tools/serial_test.py` | Sends test coordinates to the ESP32 with no ROS and no camera. |
| `tools/mock_esp32.py` | Emulates the ESP32 (parses packets, prints angles) so the whole chain can be tested without hardware. |
| `tools/calibrate_hsv.py` | Interactive OpenCV trackbar utility to tune HSV bounds for your lighting and print the `ros2 run` command. |

## Control law

Two control laws are supported depending on where the camera is mounted:

1. **Open-loop pointer (`CONTROL_MODE_CLOSED_LOOP = false`, default):** for a fixed camera watching the workspace while a separate pan/tilt head tracks the target. Implemented in `servo_math.map_point_to_angles()` and `moveServos()`:

```text
error_x = target_x - frame_width / 2
error_y = target_y - frame_height / 2

pan  = pan_home  + gain_x * (error_x / (frame_width/2))  * (pan_max  - pan_min)/2
tilt = tilt_home - gain_y * (error_y / (frame_height/2)) * (tilt_max - tilt_min)/2
```

2. **Closed-loop visual servoing (`CONTROL_MODE_CLOSED_LOOP = true`):** for an eye-in-hand setup where the camera is mounted directly on the pan/tilt head. Implemented in `servo_math.step_closed_loop_angles()` and `moveServos()`, adjusting the current angle incrementally until the target enters the optical-centre dead-zone (avoiding the oscillation that absolute mapping causes when the camera moves).

Both modes apply optional axis inversion, a dead-zone around the centre, EMA smoothing, and a final `clamp(angle, min, max)`.

## Repository layout

```text
.
├── LICENSE
├── README.md
├── requirements.txt             # host-side deps for tools/tests without ROS
├── .github/workflows/ci.yml
├── ros2_ws/src/vision_guided_control/
│   ├── package.xml setup.py setup.cfg conftest.py
│   ├── resource/vision_guided_control
│   ├── launch/tracker_and_bridge.launch.py
│   ├── vision_guided_control/{config,tracker_core,tracker_node,
│   │                            serial_protocol,serial_bridge_node,servo_math}.py
│   └── test/                    # hardware-free pytest suite
├── esp32/pan_tilt_controller/{platformio.ini, README.md, src/main.cpp}
├── tools/{serial_test,mock_esp32,calibrate_hsv}.py
└── docs/{ARCHITECTURE,HARDWARE_WIRING,PHYSICAL_SETUP,SERIAL_PROTOCOL,
          TROUBLESHOOTING}.md
```

## Testing strategy

```text
pytest (host, no ROS, no hardware)      unit suite tracker core / protocol / servo math / bridge logic
colcon build (ROS 2)                    1 package  package builds and installs
ros2 run / ros2 topic echo              runtime    nodes discoverable, topic publishes
socat PTY pair + tools/mock_esp32.py    end-to-end tracker ──► topic ──► serial ──► "ESP32"
tools/serial_test.py + real ESP32       hardware   servo direction and limits (manual, final step)
```
