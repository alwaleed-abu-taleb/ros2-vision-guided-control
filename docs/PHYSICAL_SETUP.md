# Physical Setup Guide

Follow these steps in order. Each step says exactly what to connect or type.
No software knowledge is required for the hardware steps.

---

### STEP 1 — Install the software prerequisites (once)

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt update && sudo apt install -y \
    ros-humble-geometry-msgs ros-humble-rclpy python3-opencv python3-serial \
    python3-numpy python3-colcon-common-extensions python3-pytest

# Host-side tools for testing without ROS
pip install -r requirements.txt
```

For the ESP32 firmware, install **PlatformIO** (no Arduino IDE required):

- **VS Code (recommended):** install the *PlatformIO IDE* extension from the
  marketplace. It bundles PlatformIO Core and the toolchains.
- **Command line:** see <https://platformio.org/install/cli>, or
  `python -m pip install platformio`.

The ESP32 platform, toolchain and the ESP32Servo library are all downloaded
automatically on the first build — nothing to install by hand.

---

### STEP 2 — Upload the ESP32 firmware (PlatformIO)

1. Open the folder `esp32/pan_tilt_controller` in VS Code (it is its own
   PlatformIO project — open *that folder*, not the repository root).
2. Connect the ESP32 by USB and confirm the status bar shows the board.
3. Build: the ✓ button in the PlatformIO status bar, or `pio run`.
4. Upload: the → button, or `pio run -t upload`.

Command line equivalent, from `esp32/pan_tilt_controller`:

```bash
pio run                # build
pio run -t upload      # build + upload
```

If the port is not auto-detected, specify it:

```bash
pio run -t upload --upload-port /dev/ttyUSB0     # Linux
pio run -t upload --upload-port COM3             # Windows
```

Watch the firmware at 115200 baud afterwards with `pio device monitor`.

---

### STEP 3 — Wire the pan servo

| Servo wire (colour)     | Connect to                    |
|-------------------------|-------------------------------|
| Signal (orange/yellow)  | ESP32 **GPIO 25**             |
| VCC (red)               | **External +5V**              |
| GND (brown/black)       | **Common GND**                |

---

### STEP 4 — Wire the tilt servo

| Servo wire (colour)     | Connect to                    |
|-------------------------|-------------------------------|
| Signal (orange/yellow)  | ESP32 **GPIO 26**             |
| VCC (red)               | **External +5V**              |
| GND (brown/black)       | **Common GND**                |

---

### STEP 5 — Connect external servo power

Plug in the dedicated 5V ≥ 2A supply. **Double-check before powering on:**

- [ ] Servo VCC is on the **external** supply, **not** the ESP32.
- [ ] The external supply's GND is connected to the **ESP32 GND**.
- [ ] Both servos' GND wires are on that same common ground.

Then power the external supply. See `docs/HARDWARE_WIRING.md` for the diagram.

---

### STEP 6 — Connect the ESP32 to the PC

Plug a **data-capable** USB cable between the ESP32 and the PC. This carries the
serial link (and powers the board).

---

### STEP 7 — Find the ESP32 serial port

```bash
# Linux: look for a USB-serial adapter (CP210x, CH340, FTDI)
ls -l /dev/ttyUSB*
# or let the bridge find it automatically (see STEP 10)

# Windows: note the COM port in Device Manager ▸ Ports (COM & LPT)
#          or run:
python tools/serial_test.py --list
```

If the port is missing: try a different USB cable (many are charge-only), a
different USB port, and check the driver for the ESP32's USB-serial chip.

---

### STEP 8 — Test the servos WITHOUT vision (do this first)

This isolates the hardware from the software. With the external power on:

```bash
python tools/serial_test.py --port /dev/ttyUSB0
```

The tool sends centre ▸ left ▸ right ▸ top ▸ bottom ▸ corners ▸ centre. Watch
the pan/tilt head: it should move smoothly and **always** stay inside the safe
range. If a servo moves the wrong way for a given direction, flip
`INVERT_PAN` / `INVERT_TILT` in `esp32/pan_tilt_controller/src/main.cpp` and
re-upload.

If nothing moves, stop here and check `docs/TROUBLESHOOTING.md` — most often it
is servo power or the common ground.

---

### STEP 9 — Build the ROS 2 workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Verify the package is discoverable:

```bash
ros2 pkg list | grep vision_guided_control
```

---

### STEP 10 — Start the tracker

In a first terminal:

```bash
cd ros2_ws && source install/setup.bash
ros2 run vision_guided_control tracker_node --ros-args -p show_debug:=true
```

A window should appear showing the camera feed. In a second terminal confirm the
position is being published:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /object_position
```

Hold a **bright blue** object (the default target colour) in front of the camera.
`z` should flip to `1.0` and `x`/`y` should follow the object. To track another
colour, set the HSV parameters — see `docs/TROUBLESHOOTING.md`.

> No camera yet? Run the synthetic source to validate the pipeline:
> `ros2 run vision_guided_control tracker_node --ros-args -p use_test_video:=true`

---

### STEP 11 — Start the serial bridge

In a third terminal:

```bash
cd ros2_ws && source install/setup.bash
ros2 run vision_guided_control serial_bridge_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

Leave `serial_port` empty (the default) and the bridge auto-detects the first
USB-serial device it finds.

Or launch both nodes at once:

```bash
ros2 launch vision_guided_control tracker_and_bridge.launch.py \
    serial_port:=/dev/ttyUSB0 show_debug:=true
```

---

### STEP 12 — Verify tracking

1. Hold the blue target in front of the camera.
2. Move it left/right: the **pan** servo follows.
3. Move it up/down: the **tilt** servo follows.
4. Move smoothly and confirm there is no jitter (the dead-zone and EMA smoothing
   suppress small noise).
5. Hide the target: after ~1.5 s the head should return to the home position.

If a servo moves the wrong way, invert it in the firmware (`INVERT_PAN` /
`INVERT_TILT` in `esp32/pan_tilt_controller/src/main.cpp`) and re-upload — see
`docs/TROUBLESHOOTING.md`.

---

## Quick reference — the four commands

```bash
# 0. build + upload the ESP32 firmware (from esp32/pan_tilt_controller)
pio run -t upload

# 1. servos only, no vision
python tools/serial_test.py --port /dev/ttyUSB0

# 2. tracker + debug window
ros2 run vision_guided_control tracker_node --ros-args -p show_debug:=true

# 3. serial bridge
ros2 run vision_guided_control serial_bridge_node --ros-args -p serial_port:=/dev/ttyUSB0

# 4. or both together
ros2 launch vision_guided_control tracker_and_bridge.launch.py show_debug:=true
```
