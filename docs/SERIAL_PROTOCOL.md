# Serial Protocol

The link between the ROS 2 serial bridge and the ESP32 is deliberately trivial:
ASCII text, one coordinate per line. Trivial to read with `cat`, to generate
with `echo`, and to parse on a microcontroller with no buffering library.

## Frame format

```text
<X>,<Y>\n
```

| Part      | Meaning                                              |
|-----------|------------------------------------------------------|
| `<X>`     | Decimal integer, target pixel X, range `0..65535`.   |
| `,`       | Field separator (single ASCII comma).                |
| `<Y>`     | Decimal integer, target pixel Y, range `0..65535`.   |
| `\n`      | Frame terminator (LF). `\r\n` is also accepted.      |

No whitespace, no quoting, no checksum, no headers. Example frame:

```text
320,240\n
```

Both sides agree on the **frame size**: the tracker publishes the resolution it
actually captures, and the ESP32's `FRAME_WIDTH`/`FRAME_HEIGHT` must match those
dimensions for the mapping to be correct. They are configuration values, never
an assumption: check them with `ros2 param get /tracker_node frame_width`.

## Worked examples

```text
320,240\n        centre of a 640x480 frame
0,0\n            top-left corner
640,480\n        bottom-right corner
100,100\n        near the top-left
```

## Validation rules (identical on both sides)

1. Exactly one comma, exactly two integer fields.
2. No sign character (`-`), no decimal point, no `0x` prefix, no spaces.
3. Both values inside `[0, 65535]` (fits an unsigned 16-bit integer).
4. Frames longer than 31 characters are dropped (line buffer resync).
5. Partial lines are accumulated until `\n`, never interpreted early.

The Python reference is `vision_guided_control/serial_protocol.py`
(`format_packet` / `parse_packet`); the C++ mirror is `processLine()` in
`esp32/pan_tilt_controller/src/main.cpp`.

## Accepted / rejected by the parser

| Input          | Verdict | Reason                                   |
|----------------|---------|------------------------------------------|
| `320,240\n`    | accept  | well-formed                              |
| `0,0\n`        | accept  | boundary                                 |
| `65535,65535`  | accept  | boundary                                 |
| `320,240\r\n`  | accept  | trailing CR/LF stripped                  |
| `320`          | reject  | no separator                             |
| `320,`         | reject  | empty field                              |
| `,240`         | reject  | empty field                              |
| `320,240,100`  | reject  | too many fields                          |
| `abc,def`      | reject  | non-numeric                              |
| `3.5,240`      | reject  | non-integer                              |
| `  320 , 240`  | reject  | internal whitespace                      |
| `-10,240`      | reject  | negative                                 |
| `65536,240`    | reject  | out of range                             |
| `hello`        | reject  | garbage                                  |
| `` (empty)     | reject  | empty line                               |

These verdicts were verified against both implementations: the Python
`parse_packet` unit tests and a compiled C++ harness of the firmware's
`processLine()` agree on every row.

Rejected frames are discarded and logged; they never move the servos.

## Lost-target behaviour

The tracker sets `Point.z = 0.0` when no target is found. The bridge then
**withholds** the message rather than forwarding the stale position, so no data
reaches the ESP32. The firmware notices the silence: after
`TARGET_LOST_TIMEOUT_MS` (1.5 s by default) it returns to `PAN_HOME` /
`TILT_HOME`.

## Transport parameters

| Parameter    | Default  | Notes                                   |
|--------------|----------|-----------------------------------------|
| Baud rate    | `115200` | Configurable on the bridge (`baud_rate`) and in the firmware (`SERIAL_BAUD`). Must match on both ends. |
| Framing      | 8N1      | Standard UART framing.                   |
| Flow control | none     | Only TX/RX/GND are used.                 |
| Direction    | half     | PC ──► ESP32 carries coordinates; ESP32 ──► PC is optional acknowledgements only (firmware `SEND_ACK`). |

## Manual testing

With an ESP32 running the firmware, no ROS needed:

```bash
# Send a centre coordinate, then the four compass points
echo -n "320,240
0,240
640,240
320,0
320,480" > /dev/ttyUSB0
```

Or use the bundled tool, which also lists ports and prints acknowledgements:

```bash
python tools/serial_test.py --list
python tools/serial_test.py --port /dev/ttyUSB0 --readback
```

See `docs/PHYSICAL_SETUP.md` for the full physical procedure.
