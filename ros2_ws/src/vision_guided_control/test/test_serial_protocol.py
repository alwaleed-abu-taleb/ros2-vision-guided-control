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

"""Wire-protocol tests for the PC <-> ESP32 link. No hardware required."""

import pytest

from vision_guided_control.serial_protocol import (
    format_packet,
    is_valid_packet,
    parse_packet,
)


class TestFormatPacket:
    def test_basic_format(self):
        assert format_packet(320, 240) == '320,240\n'

    def test_rounds_floats(self):
        assert format_packet(319.6, 240.4) == '320,240\n'

    @pytest.mark.parametrize('x,y', [(0, 0), (640, 480), (65535, 65535)])
    def test_accepts_boundary_coordinates(self, x, y):
        assert format_packet(x, y) == f'{x},{y}\n'

    @pytest.mark.parametrize('x,y', [(-1, 100), (100, -1), (65536, 100)])
    def test_rejects_out_of_range(self, x, y):
        with pytest.raises(ValueError):
            format_packet(x, y)

    @pytest.mark.parametrize('x,y', [(float('nan'), 100), (float('inf'), 100)])
    def test_rejects_non_finite(self, x, y):
        with pytest.raises(ValueError):
            format_packet(x, y)


class TestParsePacket:
    @pytest.mark.parametrize(
        'line,expected',
        [
            ('320,240', (320, 240)),
            ('320,240\n', (320, 240)),
            ('0,0', (0, 0)),
            ('65535,65535', (65535, 65535)),
        ],
    )
    def test_parses_valid_packets(self, line, expected):
        assert parse_packet(line) == expected

    @pytest.mark.parametrize(
        'line',
        [
            '',
            '   ',
            '320',
            'hello',
            ',240',
            '320,',
            '320,240,100',
            'abc,def',
            '3.5,240',
            '320 240',
            '  320 , 240  ',   # internal spaces: rejected, matching the ESP32 parser
            '0x10,20',
            '-10,20',
            '65536,20',
            '20,65536',
        ],
    )
    def test_rejects_malformed_packets(self, line):
        with pytest.raises(ValueError):
            parse_packet(line)

    def test_is_valid_packet_matches_parse(self):
        assert is_valid_packet('320,240\n') is True
        assert is_valid_packet('hello') is False


class TestRoundTrip:
    @pytest.mark.parametrize('x,y', [(0, 0), (320, 240), (639, 479), (65535, 1)])
    def test_format_then_parse(self, x, y):
        assert parse_packet(format_packet(x, y)) == (x, y)
