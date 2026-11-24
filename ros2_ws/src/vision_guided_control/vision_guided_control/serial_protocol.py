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

"""The wire protocol between the serial bridge and the ESP32.

Frame format (ASCII, terminated by a newline)::

    <X>,<Y>\n

Example::

    320,240\n

``X`` and ``Y`` are unsigned decimal integers in the range [0, 65535] so they
fit into a 16-bit integer on the microcontroller. Only one frame per line.
Malformed input is rejected rather than partially interpreted -- see
``docs/SERIAL_PROTOCOL.md`` for the full specification.
"""

from __future__ import annotations

from typing import Tuple

SEPARATOR = ','
TERMINATOR = '\n'

# Coordinates are pixel positions; anything above this is rejected.
MAX_COORDINATE = 65535


def format_packet(x: float, y: float) -> str:
    """Serialises a pixel coordinate into ``"X,Y\\n"``.

    Raises ``ValueError`` for NaN/infinite values or coordinates outside
    [0, MAX_COORDINATE]; the caller decides whether to drop or clamp them.
    """
    for name, value in (('x', x), ('y', y)):
        if value != value or value in (float('inf'), float('-inf')):
            raise ValueError(f'{name}={value} is not a finite number')
    if not 0 <= x <= MAX_COORDINATE or not 0 <= y <= MAX_COORDINATE:
        raise ValueError(
            f'coordinates ({x}, {y}) outside [0, {MAX_COORDINATE}]'
        )
    return f'{int(round(x))},{int(round(y))}{TERMINATOR}'


def parse_packet(line: str) -> Tuple[int, int]:
    """Parses one line of the form ``"X,Y"`` into ``(x, y)``.

    Leading/trailing whitespace and a trailing ``\\r`` are tolerated; anything
    else (missing separator, empty fields, internal spaces, non-numeric text,
    extra fields) raises ``ValueError``. This is the PC-side mirror of the
    parser running on the ESP32 and is used by ``tools/mock_esp32.py``.
    """
    if line is None:
        raise ValueError('empty line')
    text = line.strip()
    if not text:
        raise ValueError('empty line')

    parts = text.split(SEPARATOR)
    if len(parts) != 2:
        raise ValueError(f'expected 2 fields, got {len(parts)}: {text!r}')

    x_str, y_str = parts
    # int() silently tolerates surrounding whitespace ("  3 " -> 3), but the
    # ESP32's strtol does not, so reject it here to keep both parsers aligned.
    if x_str != x_str.strip() or y_str != y_str.strip():
        raise ValueError(f'internal whitespace: {text!r}')
    try:
        x = int(x_str)
        y = int(y_str)
    except ValueError:
        raise ValueError(f'non-integer coordinate: {text!r}') from None

    if not 0 <= x <= MAX_COORDINATE or not 0 <= y <= MAX_COORDINATE:
        raise ValueError(
            f'coordinates ({x}, {y}) outside [0, {MAX_COORDINATE}]'
        )
    return x, y


def is_valid_packet(line: str) -> bool:
    """Predicate form of :meth:`parse_packet`, never raises."""
    try:
        parse_packet(line)
    except ValueError:
        return False
    return True
