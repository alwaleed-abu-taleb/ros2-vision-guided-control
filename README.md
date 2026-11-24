# ROS 2 & OpenCV Vision-Guided Control

A ROS 2 Python package that tracks a coloured object with OpenCV, publishes its
pixel position on a topic, and forwards it over USB serial to an ESP32 that
drives a pan/tilt servo mount — so the camera head follows the target.

```text
Camera
   ↓
OpenCV (HSV threshold + contours)
   ↓
tracker_node  ──►  /object_position (geometry_msgs/Point)
                        ↓
              serial_bridge_node
                        ↓
                 USB serial  "X,Y\n"
                        ↓
                    ESP32
                        ↓
              Pan/Tilt servos
```

> **Resume line:** *ROS 2 & OpenCV Vision-Guided Control: Developed a ROS 2
> Python package integrating OpenCV for real-time object tracking. Processed the
> video stream to detect targets and published motor control commands over ROS 2
> topics to an ESP32 microcontroller.*

---

## Project Status

- **Software & ROS 2 Pipeline:** Verified (`colcon build`, 73 automated `pytest` cases, synthetic video + `mock_esp32.py` end-to-end).
- **ESP32 Firmware:** PlatformIO build verified (`esp32dev` / `ESP32Servo`); firmware safety clamps, line-resync parsing, smooth home return, and Open-Loop / Closed-Loop modes implemented.
- **Physical Hardware Validation:** Pending final upload & bench test on physical ESP32 + servos + webcam (demo video/GIF to be added once recorded).

---

## Features

- **Real-time colour tracking** with configurable HSV thresholds, morphological
  cleanup, largest-contour selection, and an interactive `tools/calibrate_hsv.py` tuner.
- **Loosely coupled ROS 2 architecture** — the tracker publishes, the bridge
  subscribes; neither knows about the other.
- **Dual control modes** — supports both **Open-Loop** (fixed camera pointing a separate pan/tilt head, default) and **Closed-Loop** (camera mounted on the pan/tilt head using incremental visual servoing).
- **Everything configurable** via ROS 2 parameters or firmware `#define`s — no
  hard-coded ports, resolutions or colours.
- **Hardware-free development**: 73 automated tests plus a synthetic video source
  and a mock ESP32, so the entire chain runs with no camera and no ESP32.
- **Safety-first firmware**: dead-zone, EMA smoothing, rate-limited bridge,
  smooth lost-target home return, conservative mechanical limits (`20–160°`)
  clamped immediately before every servo write, and a dedicated servo power
  design (never from a GPIO).

---

## Requirements

| Component  | Requirement                     |
|------------|---------------------------------|
| OS         | Ubuntu 22.04 (or any ROS 2 host; Windows/macOS work for the tools) |
| ROS 2      | Humble (any distro with `rclpy`) |
| Python     | 3.10+                           |
| OpenCV     | `python3-opencv` ≥ 4.5          |
| NumPy      | `python3-numpy`                 |
| pyserial   | `python3-serial` ≥ 3.5          |
| Embedded   | ESP32 + 2× hobby servos + external 5V supply |
| Firmware   | [PlatformIO](https://platformio.org/) 6+ (VS Code extension or CLI). The ESP32 platform, toolchain and ESP32Servo library are fetched automatically — **no Arduino IDE or `arduino-cli` required**. |

Install the host-side Python dependencies (needed only to run the tools and the
hardware-free tests without ROS):

```bash
pip install -r requirements.txt
```

---

## Installation & build

```bash
# 1. ROS 2 prerequisites (Ubuntu 22.04 / Humble)
sudo apt update && sudo apt install -y \
    ros-humble-desktop python3-opencv python3-serial python3-numpy \
    python3-colcon-common-extensions python3-pytest

# 2. Build the workspace
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build
source install/setup.bash

# 3. Verify
ros2 pkg list | grep vision_guided_control
```

---

## ESP32 firmware (PlatformIO)

The firmware lives in a self-contained PlatformIO project. Open the folder
`esp32/pan_tilt_controller` in VS Code with the PlatformIO IDE extension, or use
the CLI:

```bash
cd esp32/pan_tilt_controller
pio run                # build
pio run -t upload      # build + upload to the connected ESP32
```

The board environment is `esp32dev` (ESP32 Dev Module). The ESP32Servo dependency
is declared in `platformio.ini`, so nothing has to be installed by hand.

Pins, servo limits and control gains are all `#define`s at the top of
`src/main.cpp`. Full wiring instructions and the (mandatory) servo power rules
are in **[docs/HARDWARE_WIRING.md](docs/HARDWARE_WIRING.md)**.

---

## Running

### Both nodes at once (recommended)

```bash
ros2 launch vision_guided_control tracker_and_bridge.launch.py \
    serial_port:=/dev/ttyUSB0 show_debug:=true
```

### Or one node per terminal

```bash
# Terminal 1 — the tracker
cd ros2_ws && source install/setup.bash
ros2 run vision_guided_control tracker_node --ros-args -p show_debug:=true

# Terminal 2 — inspect the topic
ros2 topic echo /object_position

# Terminal 3 — the serial bridge
ros2 run vision_guided_control serial_bridge_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

### Serial configuration

The port is **never hard-coded**. Leave `serial_port` empty (the default) and the
bridge auto-detects the first USB-serial adapter; or set it explicitly, including
a Windows `COM` port:

```bash
ros2 run vision_guided_control serial_bridge_node --ros-args -p serial_port:=COM3
```

### No hardware available?

The whole pipeline still runs:

```bash
# Tracker with an animated synthetic target instead of a camera
ros2 run vision_guided_control tracker_node --ros-args -p use_test_video:=true

# Fake ESP32 on the other end of a virtual serial pair
socat -d -d pty,raw,echo=0 pty,raw,echo=0
python tools/mock_esp32.py --port /dev/pts/1
ros2 run vision_guided_control serial_bridge_node --ros-args -p serial_port:=/dev/pts/2
```

---

## Configuration

### Tracker parameters

| Parameter          | Default | Meaning                                  |
|--------------------|---------|------------------------------------------|
| `camera_index`     | `0`     | `/dev/videoN` index.                     |
| `frame_width`      | `640`   | Requested capture width (a hint).        |
| `frame_height`     | `480`   | Requested capture height (a hint).       |
| `hsv_lower_h/s/v`  | `100/100/100` | Lower HSV bound of the target colour. |
| `hsv_upper_h/s/v`  | `130/255/255` | Upper HSV bound (default: bright blue). |
| `min_contour_area` | `300`   | Contours below this px² are noise.       |
| `morph_kernel_size`| `5`     | Morphological cleanup kernel (0 = off).  |
| `publish_rate`     | `30`    | Publish frequency in Hz.                 |
| `show_debug`       | `false` | Show the OpenCV debug window with FPS.   |
| `use_test_video`   | `false` | Use an animated synthetic target.        |

### Serial bridge parameters

| Parameter              | Default | Meaning                             |
|------------------------|---------|-------------------------------------|
| `serial_port`          | `""`    | Port; empty = auto-detect.          |
| `baud_rate`            | `115200`| Must match the firmware.            |
| `serial_timeout`       | `0.1`   | Serial read/write timeout (s).      |
| `connect_retry_sec`    | `2.0`   | Reconnect attempt interval.         |
| `send_raw_when_lost`   | `false` | Forward stale coordinates when the target is lost. |
| `max_send_rate_hz`     | `30.0`  | Maximum serial packet rate in Hz (`0` = unlimited). |

### ESP32 firmware `#define`s

`FRAME_WIDTH`/`FRAME_HEIGHT`, `PAN_SERVO_PIN`/`TILT_SERVO_PIN`, `PAN_MIN/MAX` (`20–160°`),
`TILT_MIN/MAX` (`20–160°`), `PAN_HOME`/`TILT_HOME`, `CONTROL_MODE_CLOSED_LOOP` (`false` = fixed camera, `true` = camera on head),
`CLOSED_LOOP_STEP_DEG`, `GAIN_X`/`GAIN_Y`, `INVERT_PAN`/`INVERT_TILT`, `DEADZONE_PX`,
`SMOOTHING`, `TARGET_LOST_TIMEOUT_MS`, `SEND_ACK`. See the top of `esp32/pan_tilt_controller/src/main.cpp`.

---

## Testing

```bash
# Hardware-free unit + integration tests (73 cases), run anywhere with pytest
python -m pytest ros2_ws/src/vision_guided_control/test -q

# ROS 2 layer (inside the built workspace)
colcon test --packages-select vision_guided_control
```

What is covered without any hardware:

- **Tracker core** — detection, centroid accuracy, largest-contour selection,
  small-contour rejection, invalid frames, zero-moment safety, reported
  dimensions, custom HSV ranges, Hue/bound validation, debug annotation.
- **Serial protocol** — every valid/boundary/malformed frame from
  `docs/SERIAL_PROTOCOL.md`, plus format→parse round-trips.
- **Servo mapping** — centre→home, direction of each axis, servo limits, gain,
  inversion, dead-zone, clamping, EMA smoothing, and closed-loop incremental stepping.
- **Serial bridge logic** — packet formatting, lost-target withholding,
  rate limiting, out-of-range dropping, write-failure recovery, with a fake serial port and a
  stubbed `rclpy`.

End-to-end with a virtual serial pair (`tools/mock_esp32.py`) validates
tracker → topic → bridge → serial → "ESP32".

---

## Hardware-free servo checkout & HSV calibration

Before trusting vision, verify the servos alone (needs only pyserial):

```bash
python tools/serial_test.py --list            # find the ESP32's port
python tools/serial_test.py --port /dev/ttyUSB0     # centre + compass + corners sweep
python tools/serial_test.py --port /dev/ttyUSB0 --interactive
```

To tune HSV colour bounds under your room lighting with live trackbars:

```bash
python tools/calibrate_hsv.py --camera 0
```

---

## Documentation

| Document | Contents |
|----------|----------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Pipeline, ROS graph, topic semantics, module responsibilities, control law, testing strategy. |
| [docs/HARDWARE_WIRING.md](docs/HARDWARE_WIRING.md) | Pin assignment, full wiring diagram, servo power rules, mechanical limits. |
| [docs/PHYSICAL_SETUP.md](docs/PHYSICAL_SETUP.md) | Step-by-step physical build and bring-up for a human. |
| [docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md) | Wire format, worked examples, accept/reject table, lost-target behaviour. |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Symptom → fix for every layer of the system. |

---

## Project structure

```text
.
├── .github/workflows/ci.yml        # GitHub Actions pytest + tool syntax CI
├── LICENSE                         # Apache License 2.0
├── README.md
├── requirements.txt
├── ros2_ws/src/vision_guided_control/
│   ├── package.xml setup.py setup.cfg conftest.py
│   ├── resource/vision_guided_control
│   ├── launch/tracker_and_bridge.launch.py
│   ├── vision_guided_control/
│   │   ├── config.py               # TrackerConfig + ServoConfig (validated)
│   │   ├── tracker_core.py         # OpenCV pipeline (no ROS)
│   │   ├── tracker_node.py         # camera + parameters + publisher
│   │   ├── serial_protocol.py      # "X,Y\n" format/parse
│   │   ├── serial_bridge_node.py   # topic → serial, auto-reconnect, rate-limit
│   │   └── servo_math.py           # open-loop & closed-loop control laws
│   └── test/                       # 73 hardware-free pytest cases
├── esp32/pan_tilt_controller/{platformio.ini, README.md, src/main.cpp}
├── tools/
│   ├── calibrate_hsv.py            # interactive HSV threshold tuner
│   ├── serial_test.py              # servo checkout without ROS/camera
│   └── mock_esp32.py               # fake ESP32 for end-to-end tests
└── docs/
    ├── ARCHITECTURE.md HARDWARE_WIRING.md PHYSICAL_SETUP.md
    └── SERIAL_PROTOCOL.md TROUBLESHOOTING.md
```

---

## Final workflow (the 12 steps)

1. Wire the servos per `docs/HARDWARE_WIRING.md` (external 5V, common GND).
2. Connect the ESP32 to the PC by USB.
3. Open `esp32/pan_tilt_controller` in VS Code (PlatformIO) and upload the
   firmware with `pio run -t upload`.
4. Identify the ESP32 serial port.
5. `python tools/serial_test.py --port /dev/ttyUSB0` — confirm the servos move
   in the right directions.
6. `cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash`
7. Start the tracker (`tracker_node`, or the launch file).
8. Start the serial bridge (`serial_bridge_node`, or the launch file).
9. Hold a bright blue target in front of the camera.
10. Move it left/right/up/down and watch the pan/tilt head follow it.
11. Confirm there is no jitter (dead-zone + EMA smoothing).
12. Hide the target and confirm it returns to the home position after ~1.5 s.

---

## Future improvements (kept out of the MVP on purpose)

PID or Kalman filtering, multi-target detection, camera calibration and distance
estimation, a `micro-ROS` / Wi-Fi transport instead of serial, RViz
visualisation, and a ROS 2 diagnostics feed.

## License

Apache License 2.0.
