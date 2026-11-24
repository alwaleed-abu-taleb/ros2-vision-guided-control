# ESP32 Pan/Tilt Controller (PlatformIO)

Firmware for the ESP32 that receives `X,Y\n` pixel coordinates from the ROS 2
`serial_bridge_node` and drives two servos so the pan/tilt mount follows the
tracked object.

This is a **PlatformIO** project — no Arduino IDE or `arduino-cli` is required.

## Requirements

- [PlatformIO Core](https://platformio.org/install/cli) 6+, or the
  [PlatformIO IDE extension for VS Code](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).
- The `espressif32` platform and the `ESP32Servo` library are downloaded
  automatically by PlatformIO on first build.

## Build and upload

From this directory (`esp32/pan_tilt_controller`):

```bash
pio run                # build
pio run -t upload      # build + upload to the connected ESP32
pio device monitor     # serial monitor at 115200 baud
```

In VS Code with the PlatformIO extension installed, open this folder, then use
the ✓ (Build), → (Upload) and 🔌 (Monitor) buttons in the bottom status bar.

Select the serial port with `pio run -t upload --upload-port /dev/ttyUSB0`
(or `COM3` on Windows), or set `upload_port` in `platformio.ini`.

## Configuration

Everything is a `#define` at the top of `src/main.cpp`:

| Define | Default | Meaning |
|--------|---------|---------|
| `PAN_SERVO_PIN` / `TILT_SERVO_PIN` | 25 / 26 | Servo signal GPIOs. |
| `PAN_MIN/MAX`, `TILT_MIN/MAX` | 20/160 | Conservative safe mechanical angle range (clamp applied before every write). |
| `PAN_HOME` / `TILT_HOME` | 90 / 90 | Startup and lost-target position. |
| `FRAME_WIDTH` / `FRAME_HEIGHT` | 640 / 480 | Must match the tracker's `frame_width`/`frame_height` parameters. |
| `CONTROL_MODE_CLOSED_LOOP` | `false` | `false` = fixed camera (open-loop pointing); `true` = camera mounted on pan/tilt head (incremental visual servoing). |
| `CLOSED_LOOP_STEP_DEG` | `4.0` | Maximum step per update in closed-loop mode. |
| `GAIN_X` / `GAIN_Y` | 1.0 | Proportional gain (fraction of servo half-range in open-loop, or step multiplier in closed-loop). |
| `INVERT_PAN` / `INVERT_TILT` | false | Flip an axis if the servo moves the wrong way. |
| `DEADZONE_PX` | 15 | Ignore target motion smaller than this (suppresses jitter). |
| `SMOOTHING` | 0.35 | EMA factor: 0 = frozen, 1 = no smoothing. |
| `TARGET_LOST_TIMEOUT_MS` | 1500 | Return smoothly to home if no coordinate arrives for this long. |
| `SERIAL_BAUD` | 115200 | Must match the `serial_bridge_node` `baud_rate` parameter. |

## Protocol

```text
X,Y\n          e.g. 320,240\n
```

See `docs/SERIAL_PROTOCOL.md` for the full specification, including every
rejected input.

## Wiring

See `docs/HARDWARE_WIRING.md`. **Servos must be powered from a dedicated 5V
supply, never from an ESP32 GPIO pin or the ESP32 3.3V regulator.** The external
supply ground must be tied to the ESP32 GND.

## Relationship to the rest of the system

```text
Camera → OpenCV → tracker_node → /object_position → serial_bridge_node
                                                              ↓ USB serial
                                                        this firmware
                                                              ↓
                                                        Pan/Tilt servos
```

The control law in `moveServos()` mirrors `servo_math.py` in the ROS 2 package
so the simulated behaviour matches the hardware.
