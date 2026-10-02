"""Tests for laptop control and result payload helpers."""

import json

from laptop_ai.result_protocol import encode_result, ZERO_DIRT


def test_result_encoder_emits_protocol_v2_and_no_nan():
    payload = json.loads(
        encode_result(
            source_id='laptop-ai-01',
            session_id='session',
            frame_id=1,
            sequence=1,
            capture_timestamp_ns=1,
            inference_timestamp_ns=2,
            send_timestamp_ns=3,
            mode='idle',
            image_width=1280,
            image_height=720,
            valid=False,
            panel_visible=False,
            panels=[],
            active_panel_id=-1,
            dirt_found=False,
            inference_time_ms=0.0,
            invalid_reason='idle',
            model_name='model.onnx',
            **ZERO_DIRT,
        )
    )
    assert payload['protocol_version'] == 2
    assert payload['mode'] == 'idle'
    assert not payload['dirt_found']
