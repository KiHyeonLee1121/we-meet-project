"""Tests for the laptop-to-Pi perception protocol."""

import json

from da_daka_control.perception_protocol import (
    decode_perception_packet,
    PerceptionProtocolError,
    ProtocolConfig,
    SequenceTracker,
)
import pytest


def packet(**updates):
    payload = {
        'protocol_version': 2,
        'source_id': 'laptop-ai-01',
        'session_id': 'test-session',
        'frame_id': 1,
        'sequence': 1,
        'capture_timestamp_ns': 100,
        'inference_timestamp_ns': 110,
        'send_timestamp_ns': 120,
        'mode': 'survey',
        'image_width': 1280,
        'image_height': 720,
        'valid': True,
        'panel_visible': True,
        'panels': [
            {
                'candidate_id': 1,
                'center_x_norm': 0.5,
                'center_y_norm': 0.5,
                'width_norm': 0.4,
                'height_norm': 0.3,
                'confidence': 0.9,
            }
        ],
        'active_panel_id': -1,
        'dirt_found': False,
        'dirt_centroid_x_norm': 0.0,
        'dirt_centroid_y_norm': 0.0,
        'dirt_bbox_x_norm': 0.0,
        'dirt_bbox_y_norm': 0.0,
        'dirt_bbox_w_norm': 0.0,
        'dirt_bbox_h_norm': 0.0,
        'dirt_confidence': 0.0,
        'inference_time_ms': 12.0,
        'invalid_reason': '',
        'model_name': 'test-model',
    }
    payload.update(updates)
    return json.dumps(payload)


def test_valid_survey_packet_is_decoded():
    result = decode_perception_packet(
        packet(), ProtocolConfig('laptop-ai-01')
    )
    assert result.mode == 'survey'
    assert len(result.panels) == 1
    assert not result.dirt_found


def test_source_allowlist_is_enforced():
    with pytest.raises(PerceptionProtocolError) as error:
        decode_perception_packet(packet(source_id='attacker'), ProtocolConfig('laptop-ai-01'))
    assert error.value.code == 'source_id'


def test_clean_packet_cannot_carry_nonzero_dirt_coordinates():
    with pytest.raises(PerceptionProtocolError) as error:
        decode_perception_packet(
            packet(dirt_centroid_x_norm=0.2), ProtocolConfig('laptop-ai-01')
        )
    assert error.value.code == 'dirt_detection'


def test_sequence_tracker_rejects_duplicate_result():
    config = ProtocolConfig('laptop-ai-01')
    result = decode_perception_packet(packet(), config)
    tracker = SequenceTracker()
    tracker.accept(result)
    with pytest.raises(PerceptionProtocolError) as error:
        tracker.accept(result)
    assert error.value.code == 'sequence_order'
