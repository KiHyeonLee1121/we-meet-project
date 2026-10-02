"""Unit tests for the TF-Luna binary stream parser."""

from da_daka_control.tf_luna_serial import TfLunaParser
import pytest


def make_frame(
    distance_cm: int,
    strength: int = 500,
    temperature_raw: int = 2368,
) -> bytes:
    """Build one checksum-valid TF-Luna frame."""
    frame = bytearray(
        [
            0x59,
            0x59,
            distance_cm & 0xFF,
            distance_cm >> 8,
            strength & 0xFF,
            strength >> 8,
            temperature_raw & 0xFF,
            temperature_raw >> 8,
        ]
    )
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


def test_parser_decodes_distance_strength_and_temperature():
    parser = TfLunaParser()

    frames = parser.feed(make_frame(123, strength=456, temperature_raw=2368))

    assert len(frames) == 1
    assert frames[0].distance_m == pytest.approx(1.23)
    assert frames[0].strength == 456
    assert frames[0].temperature_c == pytest.approx(40.0)


def test_parser_accepts_fragmented_input():
    parser = TfLunaParser()
    frame = make_frame(87)

    assert parser.feed(frame[:4]) == []
    frames = parser.feed(frame[4:])

    assert len(frames) == 1
    assert frames[0].distance_m == pytest.approx(0.87)


def test_parser_recovers_after_noise_and_bad_checksum():
    parser = TfLunaParser()
    bad_frame = bytearray(make_frame(50))
    bad_frame[-1] ^= 0xFF

    frames = parser.feed(b'\x00\x01\x59' + bytes(bad_frame) + make_frame(75))

    assert [frame.distance_m for frame in frames] == [pytest.approx(0.75)]
    assert parser.checksum_errors >= 1
    assert parser.discarded_bytes >= 1


def test_parser_returns_multiple_frames_from_one_chunk():
    parser = TfLunaParser()

    frames = parser.feed(make_frame(20) + make_frame(30) + make_frame(40))

    assert [frame.distance_m for frame in frames] == [
        pytest.approx(0.2),
        pytest.approx(0.3),
        pytest.approx(0.4),
    ]
