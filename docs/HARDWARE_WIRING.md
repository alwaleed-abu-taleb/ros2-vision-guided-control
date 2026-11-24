# Hardware Wiring

> ## ⚠ READ THIS FIRST — SERVO POWER RULE
>
> **NEVER power a servo from an ESP32 GPIO pin, and never power two or more
> servos from the ESP32's onboard 3.3V regulator or from the USB 5V rail.**
>
> A standard hobby servo stalls around 0.5–1 A per servo. An ESP32 GPIO pin can
> source ~12 mA continuously (40 mA absolute maximum). Drawing servo current
> through a GPIO will **destroy the pin and very likely the whole ESP32**, and
> even the 5V USB rail will brown out, resetting the board or causing the servos
> to jitter and snap randomly.
>
> Always use a **dedicated 5V (or 4.8–6V, per the servo datasheet) power supply**
> rated for at least 1 A per servo, and **tie its ground to the ESP32 GND**.

## Components

| Part                 | Example                             | Notes                                    |
|----------------------|-------------------------------------|------------------------------------------|
| Microcontroller      | ESP32 DevKit V1 / NodeMCU-32S       | Any ESP32 board with ≥ 2 free GPIOs.     |
| Pan servo            | SG90 (or MG90S / SG5010)            | 180° rotation, 4.8–6V.                   |
| Tilt servo           | SG90 (or MG90S / SG5010)            | Same as pan.                             |
| Servo power supply   | 5V ≥ 2A wall adapter, or 4×AA pack  | Dedicate it to the servos.               |
| Camera               | USB webcam                          | Used by the PC side, not wired to ESP32. |
| USB-C / micro-USB cable | data-capable                    | Flashes firmware and carries the serial link. |

## ESP32 pin assignment

The pins are chosen once, defined once in the firmware, and documented here:

| Signal            | ESP32 GPIO | Servo wire colour (typical) | Notes                     |
|-------------------|:----------:|-----------------------------|---------------------------|
| Pan servo signal  | **GPIO 25**| Orange / Yellow             | PWM output, 50 Hz, 500–2400 µs. |
| Tilt servo signal | **GPIO 26**| Orange / Yellow             | PWM output, 50 Hz, 500–2400 µs. |
| Servo VCC         | —          | Red                         | **External supply only.** |
| Servo GND         | —          | Brown / Black               | Common ground with ESP32. |
| ESP32 USB         | —          | —                           | To the PC (serial + power). |

**Why GPIO 25/26:** they are general-purpose output pins on essentially every
ESP32 module — not strapping pins (0/2/12/15), not the USB-UART pins (1/3), not
SPI-flash pins (6–11), and they support the ESP32Servo library's hardware PWM
timers. If you move a servo, change `PAN_SERVO_PIN` / `TILT_SERVO_PIN` at the top
of `esp32/pan_tilt_controller/src/main.cpp` and update this table.

> GPIO 25/26 are also the ESP32's DAC pins; using them for PWM servo output is
> completely safe and is a common documented ESP32Servo configuration.

## Wiring diagram

```text
                          +----------------------+
                          |        ESP32          |
                          |                      |
   PC ────── USB ────────►| USB                  |
                          |                      |
   External 5V ──┬────────| VIN / 5V  (optional, |
        (≥2A)    │        |  only if you also want|
                 │        |  to power the board   |
                 │        |  from the same supply)|
                 │        |                      |
                 │   GPIO25 ──────────────┐      |
                 │                        │      |
                 │   GPIO26 ───────────┐  │      |
                 │                    │  │      |
                 │   GND ─────────────┼──┼──────┘
                 +----------------------+
                                             │
   External GND ─────────────────────────────┘
        │
        ├───► PAN SERVO  GND
        ├───► TILT SERVO GND
        │
   External +5V ──────────┬─────────────────────┘
                          │
                          ├───► PAN SERVO  VCC
                          └───► TILT SERVO VCC

   PAN SERVO:   Signal (orange) ──► ESP32 GPIO25
                VCC    (red)    ──► External +5V
                GND    (brown)  ──► Common GND

   TILT SERVO:  Signal (orange) ──► ESP32 GPIO26
                VCC    (red)    ──► External +5V
                GND    (brown)  ──► Common GND
```

The **single most important detail in that diagram is the common ground**: the
ESP32, the servo power supply and both servos must all share one ground
reference, otherwise the servo signal pulses have no reference and the motors
will twitch or refuse to move.

## Ground rules checklist

- [ ] Servo VCC comes from the **external** supply, never from the ESP32.
- [ ] External supply ground is wired to **ESP32 GND**.
- [ ] Both servos' GND wires are on that same common ground.
- [ ] Servo signal wires go to GPIO 25 (pan) and GPIO 26 (tilt).
- [ ] The external supply can deliver ≥ 1 A **per servo** (2 A for two SG90s).
- [ ] The USB cable between ESP32 and PC is data-capable (charge-only cables
      will not enumerate a serial port).

## Servo mechanical limits

Many pan/tilt brackets collide before a servo reaches 0° or 180°. Before powering
anything, set the firmware limits to what the mechanism physically allows (in
`esp32/pan_tilt_controller/src/main.cpp`):

```cpp
#define PAN_MIN   20     // conservative defaults; widen only after verifying clearance
#define PAN_MAX   160
#define TILT_MIN  20
#define TILT_MAX  160
#define PAN_HOME  90
#define TILT_HOME 90
```

Start narrow (e.g. `PAN_MIN 30 / PAN_MAX 150`) and widen only after verifying
free movement. The firmware clamps every command to this range **after** the
control law and immediately before writing to the servo, so no software bug can
command an out-of-range angle.

## Optional: single-supply wiring

If you have one 5V ≥ 2 A supply and want it to also run the ESP32, wire supply
+5V to **both** `VIN`/`5V` and the servo VCC rails, and supply GND to ESP32 GND.
Do **not** back-feed 5V into the ESP32 while USB is also connected — pick one
power source for the board. SG90 servos are fine on 5V; check your servo's
datasheet if using something larger.

## What does NOT get wired

- The **camera connects to the PC** over USB, never to the ESP32.
- No resistors, capacitors or level shifters are required for these servos —
  the ESP32's 3.3V logic high is accepted by standard servo signal inputs.

## Configuration summary

The values the firmware and the PC side must agree on. Firmware side is
`esp32/pan_tilt_controller/platformio.ini` + `src/main.cpp`; PC side is the ROS 2
parameters of the same names.

| Setting            | Firmware location                    | Value      |
|--------------------|---------------------------------------|------------|
| ESP32 board        | `platformio.ini` → `board`           | `esp32dev` (ESP32 Dev Module) |
| USB connection     | `upload_port` / auto-detect          | data-capable USB cable, carries serial + power |
| Pan servo GPIO     | `PAN_SERVO_PIN`                      | **25**     |
| Tilt servo GPIO    | `TILT_SERVO_PIN`                     | **26**     |
| Servo power        | hardware wiring (see diagram above)  | **External 5V ≥ 2A** — never from a GPIO pin or the ESP32 3.3V regulator |
| Common ground      | hardware wiring                      | external supply GND tied to ESP32 GND |
| Serial baud rate   | `SERIAL_BAUD` (`platformio.ini` build flag) | **115200** — must match the `serial_bridge_node` `baud_rate` parameter |
| Frame size         | `FRAME_WIDTH` / `FRAME_HEIGHT`       | 640 × 480 — must match the `tracker_node` `frame_width` / `frame_height` parameters |

> **PlatformIO, not Arduino IDE.** The firmware project is built and uploaded
> with PlatformIO (`pio run`, `pio run -t upload`) from VS Code or the command
> line. The ESP32Servo library is declared in `platformio.ini` and fetched
> automatically — no manual library installation is required.
