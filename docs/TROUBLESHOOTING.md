# Troubleshooting

Symptoms are grouped by layer. Start at the top and work down.

---

## ROS 2 / build

### `ros2: command not found`
ROS 2 is not installed or not sourced.
```bash
# Ubuntu 22.04 (Humble)
sudo apt update && sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash     # run in EVERY terminal, or add to ~/.bashrc
```

### `colcon: command not found`
```bash
sudo apt install -y python3-colcon-common-extensions
```

### `colcon build` fails with `package 'vision_guided_control' not found`
You are running `colcon build` from the wrong directory. It must run from
`ros2_ws`, where `src/vision_guided_control/package.xml` lives:
```bash
cd ros2_ws && colcon build
```

### `colcon build` fails with `ament_python` error
The `ament_python` build tool is missing:
```bash
sudo apt install -y ros-humble-ament-python-build-extensions
```

### `ros2 pkg list` does not show the package
You forgot to source the workspace overlay after building:
```bash
source ros2_ws/install/setup.bash
```

### `ros2 run vision_guided_control tracker_node` fails with `Package not found`
Same cause — source the overlay. Confirm first with
`ros2 pkg list | grep vision`.

---

## Camera / OpenCV

### `OpenCV not found` / `No module named 'cv2'`
```bash
sudo apt install -y python3-opencv      # inside ROS 2
# or, without ROS:
pip install opencv-python
```

### Camera not detected / `Camera returned an empty frame`
- Check the index: list devices and try `camera_index:=1`, `:=2`, etc.
- On Linux check permissions and that no other app (browser, Zoom, Cheese) holds
  the camera exclusively.
- USB cameras on WSL2 need `usbipd-win` to be forwarded; run the tracker on the
  Windows host or pass the camera through explicitly.
- Test without hardware first to confirm the software side:
  ```bash
  ros2 run vision_guided_control tracker_node --ros-args -p use_test_video:=true
  ```

### Object not detected at all
The HSV range does not match your object's colour.
1. Run the interactive calibrator to find exact HSV bounds for your lighting:
   ```bash
   python tools/calibrate_hsv.py
   ```
   Press `p` or `q` to print the ready-to-use `ros2 run` command.
2. Or run with the debug window on (`-p show_debug:=true`) and adjust the bounds manually:
   ```bash
   ros2 run vision_guided_control tracker_node --ros-args \
       -p hsv_lower_h:=40 -p hsv_lower_s:=80 -p hsv_lower_v:=80 \
       -p hsv_upper_h:=80 -p hsv_upper_s:=255 -p hsv_upper_v:=255
   ```
3. Light the object evenly; a shadowed object has low saturation and falls out
   of the default S/V bounds (raise `hsv_lower_v` only if you must, as that
   admits background noise).
4. **Red is a special case** — hue wraps around 0/179, so red needs two masks.
   Pick blue, green or yellow instead; it makes the threshold single-range.

### Object detection is unstable / flickers
- Raise `min_contour_area` to reject small false positives (e.g. `:=800`).
- Increase `morph_kernel_size` (e.g. `:=7`) to clean the mask.
- Improve lighting; a flickering lamp varies hue frame to frame.
- Use a larger, uniformly coloured object.

---

## Topics

### `ros2 topic echo /object_position` prints nothing
- Is `tracker_node` actually running? Check `ros2 node list`.
- Is the camera failing? The tracker logs `Camera returned an empty frame` and
  publishes `z: 0.0` — `echo` will show those messages, so a truly empty topic
  means the node is dead or the camera retry loop is stuck retrying.
- Confirm the topic exists and its type:
  ```bash
  ros2 topic list | grep object_position
  ros2 topic info /object_position
  ```

### `/object_position` shows `z: 0.0` constantly
Detection is failing — see *Object not detected at all* above. `x`/`y` in that
state are stale/zero, and the serial bridge correctly forwards nothing.

---

## Serial / ESP32

### ESP32 serial port missing (`/dev/ttyUSB*` does not appear)
- Try another USB cable — **charge-only cables** are the #1 cause.
- Check the chip driver: `lsusb` should show the ESP32's USB-serial bridge
  (CP210x, CH340, FTDI). On Linux, recent kernels include all three.
- On WSL2, USB devices are not forwarded by default; use `usbipd-win`, or run
  the bridge on the Windows host with the COM port.

### `Permission denied` on `/dev/ttyUSB0`
Your user is not in the `dialout` group:
```bash
sudo usermod -aG dialout $USER
# then log out and back in (a new terminal is not enough)
# verify:
groups | grep dialout
```
Quick one-off test: `sudo python tools/serial_test.py --port /dev/ttyUSB0`.

### Serial bridge cannot connect (logs `Cannot open ...`)
- Wrong port name — use `python tools/serial_test.py --list` to see real ports.
- Another program holds the port (PlatformIO's serial monitor, minicom, a
  previous bridge instance). Close it.
- On Windows, remember that the port is `COM3`, not `/dev/ttyUSB0`.
- The bridge retries forever by default; just plug the ESP32 in and it connects.

### Bridge connects but the servos do not move
Split the problem in half with the hardware-only tool:
```bash
python tools/serial_test.py --port /dev/ttyUSB0 --readback
```
- If the servos **do** move here, the tracker is the problem (topic empty or
  `z: 0.0`) — see the *Topics* section.
- If they **don't** move here either, it is hardware: servo power, common
  ground, or pin assignment — see `docs/HARDWARE_WIRING.md`.

### Servo does not move / only hums
- **Servo power** is almost always the cause: it must come from the external
  supply, not the ESP32. Verify the external supply is actually powered on.
- **Common ground** missing between the external supply and the ESP32 GND.
- Signal wire on the wrong GPIO (expected: pan = GPIO 25, tilt = GPIO 26).

### Servo jitters / snaps around
- Brown-out: the supply cannot deliver stall current. Use ≥ 1 A per servo and
  short, thick wires.
- Increase the firmware dead-zone (`DEADZONE_PX`) and smoothing (`SMOOTHING`).
- Too high a `publish_rate` floods the servo; 30 Hz is plenty, and the firmware
  already throttles writes to 50 Hz.

### Wrong servo direction (pan goes right when target goes left)
Invert that axis at the top of `esp32/pan_tilt_controller/src/main.cpp` and
re-upload with `pio run -t upload`:
```cpp
#define INVERT_PAN  true
#define INVERT_TILT false
```
(The Python simulator has the same flags in `ServoConfig`.)

### Servo hits the frame / grinding noise
The mechanical limits are wider than the bracket. Narrow them at the top of
`esp32/pan_tilt_controller/src/main.cpp`, re-upload, and verify again with
`tools/serial_test.py`:
```cpp
#define PAN_MIN  30
#define PAN_MAX  150
```

---

## ESP32 firmware (PlatformIO)

### Sketch does not compile: `ESP32Servo.h: No such file or directory`
PlatformIO should fetch this dependency automatically — it is declared in
`esp32/pan_tilt_controller/platformio.ini` (`lib_deps`). If it did not:

```bash
cd esp32/pan_tilt_controller
pio pkg update
pio run
```

If you are getting this error from the **Arduino IDE**, you are in the wrong
project: the firmware has migrated to PlatformIO and there is no `.ino` file.
Open `esp32/pan_tilt_controller` in VS Code with the PlatformIO IDE extension
instead.

### `pio: command not found` / PlatformIO not recognised
PlatformIO Core is not installed, or not on `PATH`. Easiest fix: install the
*PlatformIO IDE* extension in VS Code and open the `esp32/pan_tilt_controller`
folder. For the CLI, see <https://platformio.org/install/cli>.

### `Please specify upload_port` during `pio run -t upload`
No ESP32 was found on USB. Check the cable is data-capable and the board is
powered, then either let PlatformIO auto-detect it or pass it explicitly:

```bash
pio run -t upload --upload-port /dev/ttyUSB0     # Linux
pio run -t upload --upload-port COM3             # Windows
```

### Upload fails / times out
- Hold **BOOT** (or IO0) while the upload starts, release when you see
  `Connecting....`
- Check the selected port and board (`board = esp32dev` in `platformio.ini`).
- Try a lower upload speed (`upload_speed = 115200` in `platformio.ini`) with a
  long cable.
- If a previous upload is stuck in the bootloader, a single `pio run -t upload`
  usually resyncs; otherwise power-cycle the board.

### ESP32 resets when the servos move
Classic brown-out: servos are pulling the ESP32's supply down. Power the servos
from the dedicated external supply with a common ground — never from the board.

---

## Still stuck

Reduce the system to the smallest failing piece:

```bash
# 1. Pure Python, no ROS, no camera: does the tracker math work?
python -m pytest ros2_ws/src/vision_guided_control/test -q

# 2. No ROS, no camera: do the servos move on command?
python tools/serial_test.py --port /dev/ttyUSB0

# 3. No hardware at all: does the full ROS chain deliver packets?
socat -d -d pty,raw,echo=0 pty,raw,echo=0
python tools/mock_esp32.py --port /dev/pts/1
ros2 run vision_guided_control tracker_node --ros-args -p use_test_video:=true
ros2 run vision_guided_control serial_bridge_node --ros-args -p serial_port:=/dev/pts/2
```
