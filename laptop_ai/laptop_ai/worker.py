"""Receive Pi video and return typed panel/dirt perception results."""

import logging
from pathlib import Path
import socket
import time
import uuid

import av
import cv2

from laptop_ai.control_protocol import ControlReceiver
from laptop_ai.panel_detector import PanelDetector, select_panel_nearest_target
from laptop_ai.result_protocol import encode_result, ZERO_DIRT
from laptop_ai.runtime_tuning import (
    RuntimeTuning,
    configure_cuda_environment,
    configure_opencv,
)


class MpegTsVideoReceiver:
    """Decode the Pi's MPEG-TS UDP stream through PyAV/FFmpeg."""

    def __init__(self, port: int) -> None:
        if not 1 <= port <= 65535:
            raise ValueError('video port must be within [1, 65535]')
        url = (
            f'udp://0.0.0.0:{port}'
            '?fifo_size=1000000&overrun_nonfatal=1'
        )
        self.container = av.open(
            url,
            format='mpegts',
            options={'fflags': 'nobuffer', 'flags': 'low_delay'},
        )
        self.frames = self.container.decode(video=0)

    def read(self):
        """Return the next decoded BGR frame."""
        try:
            frame = next(self.frames)
        except StopIteration as exc:
            raise RuntimeError('Pi video stream ended') from exc
        return frame.to_ndarray(format='bgr24')

    def close(self) -> None:
        """Close the network decoder."""
        self.container.close()


def translate_dirt_to_frame(dirt, panel, frame_width, frame_height):
    """Translate panel-ROI normalized dirt coordinates to full-frame values."""
    return {
        'dirt_centroid_x_norm': (
            panel.x + dirt.centroid_x_norm * panel.width
        ) / frame_width,
        'dirt_centroid_y_norm': (
            panel.y + dirt.centroid_y_norm * panel.height
        ) / frame_height,
        'dirt_bbox_x_norm': (
            panel.x + dirt.bbox_x_norm * panel.width
        ) / frame_width,
        'dirt_bbox_y_norm': (
            panel.y + dirt.bbox_y_norm * panel.height
        ) / frame_height,
        'dirt_bbox_w_norm': dirt.bbox_w_norm * panel.width / frame_width,
        'dirt_bbox_h_norm': dirt.bbox_h_norm * panel.height / frame_height,
        'dirt_confidence': dirt.confidence,
    }


class LaptopAiWorker:
    """Single process for video decode, perception and UDP output."""

    def __init__(
        self,
        config: dict,
        *,
        viewer=None,
        panel_only: bool = False,
    ) -> None:
        tuning = RuntimeTuning.from_mapping(config.get('performance'))
        self.panel_only = bool(panel_only)
        if not self.panel_only:
            configure_cuda_environment(tuning)
        configure_opencv(tuning)

        network = config['network']
        video = config['video']
        panel = config['panel_detector']
        model = config['dirt_model']
        self.source_id = str(network['source_id'])
        self.pi_ip = str(network['pi_ip'])
        self.pi_address = (self.pi_ip, int(network['result_port']))
        self.viewer = viewer
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control_socket.bind((str(network['control_bind']), int(network['control_port'])))
        self.control_socket.setblocking(False)
        self.control = ControlReceiver(
            self.control_socket,
            allowed_source_id=str(network['pi_source_id']),
            allowed_remote_ip=str(network['pi_ip']),
            timeout_s=float(network['control_timeout_s']),
        )
        self.result_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video = MpegTsVideoReceiver(int(video['port']))
        self.panel_detector = PanelDetector(
            minimum_area_ratio=float(panel['minimum_area_ratio']),
            maximum_area_ratio=float(panel['maximum_area_ratio']),
            minimum_aspect_ratio=float(panel['minimum_aspect_ratio']),
            maximum_aspect_ratio=float(panel['maximum_aspect_ratio']),
            maximum_panels=int(panel['maximum_panels']),
            polygon_epsilon_ratio=float(panel['polygon_epsilon_ratio']),
            blue_fallback_enabled=bool(
                panel.get('blue_fallback_enabled', True)
            ),
            blue_minimum_b_minus_r=int(
                panel.get('blue_minimum_b_minus_r', 12)
            ),
            blue_minimum_b_minus_g=int(
                panel.get('blue_minimum_b_minus_g', 0)
            ),
            blue_minimum_brightness=int(
                panel.get('blue_minimum_brightness', 20)
            ),
            blue_minimum_rectangularity=float(
                panel.get('blue_minimum_rectangularity', 0.50)
            ),
            blue_minimum_landscape_ratio=float(
                panel.get('blue_minimum_landscape_ratio', 1.20)
            ),
            blue_grid_validation_enabled=bool(
                panel.get('blue_grid_validation_enabled', True)
            ),
            blue_grid_minimum_vertical_lines=int(
                panel.get('blue_grid_minimum_vertical_lines', 3)
            ),
            blue_grid_minimum_horizontal_lines=int(
                panel.get('blue_grid_minimum_horizontal_lines', 1)
            ),
            blue_grid_minimum_edge_density=float(
                panel.get('blue_grid_minimum_edge_density', 0.008)
            ),
            border_margin_px=int(panel.get('border_margin_px', 2)),
            maximum_width_ratio=float(
                panel.get('maximum_width_ratio', 0.85)
            ),
            maximum_height_ratio=float(
                panel.get('maximum_height_ratio', 0.85)
            ),
        )
        selection = config.get('target_selection', {})
        self.target_x_norm = float(selection.get('target_x_norm', 0.5))
        self.target_y_norm = float(selection.get('target_y_norm', 0.5))
        self.maximum_target_distance_norm = float(
            selection.get('maximum_center_distance_norm', 0.45)
        )
        self.dirt_detector = None
        self.model_name = 'opencv-panel-cpu'
        if not self.panel_only:
            from laptop_ai.onnx_dirt_detector import OnnxDirtSegmenter

            self.dirt_detector = OnnxDirtSegmenter(
                str(model['path']),
                input_width=int(model['input_width']),
                input_height=int(model['input_height']),
                threshold=float(model['threshold']),
                minimum_area_ratio=float(model['minimum_area_ratio']),
                output_channel=int(model.get('output_channel', 0)),
                performance=config.get('performance'),
            )
            self.model_name = self.dirt_detector.model_name
        self.optimizer = None
        self.scene_change = None
        optimizer_path = config.get('runtime', {}).get('optimizer_config')
        if optimizer_path and not self.panel_only:
            from laptop_ai.joint_optimizer import load_optimizer_config
            from laptop_ai.optimizer_runtime import RuntimeJointOptimizer
            from laptop_ai.scene_change import SceneChangeDetector

            optimizer_config = load_optimizer_config(str(optimizer_path))
            if optimizer_config.mode == 'apply':
                raise RuntimeError(
                    'optimizer apply mode requires a measured encoder/model '
                    'adapter; use observe mode for the production worker'
                )
            self.optimizer = RuntimeJointOptimizer(optimizer_config)
            if optimizer_config.enabled:
                self.scene_change = SceneChangeDetector(
                    optimizer_config.scene_change_threshold
                )
        self.session_id = str(uuid.uuid4())
        self.sequence = 0
        self.frame_id = 0
        runtime = config['runtime']
        self.idle_heartbeat_s = float(runtime['idle_heartbeat_s'])
        self._last_idle_send_s = -float('inf')
        debug_directory = str(runtime.get('survey_debug_directory', '')).strip()
        self.survey_debug_directory = (
            Path(debug_directory).expanduser().resolve()
            if debug_directory else None
        )
        self.survey_debug_interval_s = float(
            runtime.get('survey_debug_interval_s', 1.0)
        )
        self.survey_debug_max_frames = int(
            runtime.get('survey_debug_max_frames', 120)
        )
        self._survey_debug_last_s = -float('inf')
        self._survey_debug_saved = 0

    def run(self) -> None:
        while True:
            self.control.poll()
            control = self.control.state()
            frame = self.video.read()
            self.frame_id += 1
            control_connected = bool(control.session_id) or control.sequence > 0
            if control.mode == 'idle':
                now_s = time.monotonic()
                if now_s - self._last_idle_send_s >= self.idle_heartbeat_s:
                    self._send_idle(frame, control.active_panel_id)
                    self._last_idle_send_s = now_s
                if not self._show_frame(
                    frame,
                    mode='idle',
                    active_panel_id=control.active_panel_id,
                    control_connected=control_connected,
                    panels=[],
                    selected=None,
                    valid=False,
                    panel_visible=False,
                    dirt_found=False,
                    dirt_values=dict(ZERO_DIRT),
                    inference_ms=0.0,
                    invalid_reason='mission-idle-or-control-stale',
                ):
                    return
                continue
            if not self._process_frame(
                frame,
                control.mode,
                control.active_panel_id,
                control_connected=control_connected,
            ):
                return

    def _process_frame(
        self,
        frame,
        mode: str,
        panel_id: int,
        *,
        control_connected: bool = True,
    ) -> bool:
        capture_ns = time.time_ns()
        started_s = time.perf_counter()
        panels = self.panel_detector.detect(frame)
        self._maybe_save_survey_debug(frame, mode, panels)
        height, width = frame.shape[:2]
        normalized_panels = [item.normalized(width, height) for item in panels]
        selected = select_panel_nearest_target(
            panels,
            image_width=width,
            image_height=height,
            target_x_norm=self.target_x_norm,
            target_y_norm=self.target_y_norm,
            maximum_distance_norm=self.maximum_target_distance_norm,
        )
        if self.scene_change is not None:
            centroid = None
            if selected is not None:
                centroid = (
                    (selected.x + selected.width / 2.0) / width,
                    (selected.y + selected.height / 2.0) / height,
                )
            sample = self.scene_change.update(frame, centroid)
            decision = self.optimizer.maybe_decide(
                scene_changed=sample.significant
            )
            if decision is not None:
                self.optimizer.log_decision(
                    logging.getLogger('laptop_ai.optimizer'),
                    decision,
                )
        dirt_found = False
        dirt_values = dict(ZERO_DIRT)
        panel_visible = bool(panels) if mode == 'survey' else selected is not None
        valid = True
        invalid_reason = ''
        if mode == 'clean':
            if selected is None:
                valid = False
                invalid_reason = (
                    'panel-not-centered' if panels else 'panel-not-found'
                )
            elif self.dirt_detector is not None:
                roi = frame[
                    selected.y:selected.y + selected.height,
                    selected.x:selected.x + selected.width,
                ]
                dirt = self.dirt_detector.detect(roi)
                if dirt is not None:
                    dirt_found = True
                    dirt_values = translate_dirt_to_frame(
                        dirt, selected, width, height
                    )
        inference_ns = time.time_ns()
        inference_ms = (time.perf_counter() - started_s) * 1000.0
        self._send(
            mode=mode,
            width=width,
            height=height,
            capture_ns=capture_ns,
            inference_ns=inference_ns,
            valid=valid,
            panel_visible=panel_visible,
            panels=normalized_panels,
            active_panel_id=panel_id,
            dirt_found=dirt_found,
            dirt_values=dirt_values,
            inference_ms=inference_ms,
            invalid_reason=invalid_reason,
        )
        return self._show_frame(
            frame,
            mode=mode,
            active_panel_id=panel_id,
            control_connected=control_connected,
            panels=panels,
            selected=selected,
            valid=valid,
            panel_visible=panel_visible,
            dirt_found=dirt_found,
            dirt_values=dirt_values,
            inference_ms=inference_ms,
            invalid_reason=invalid_reason,
        )

    def _maybe_save_survey_debug(self, frame, mode, panels) -> None:
        """Persist sparse live survey frames for field detection diagnosis."""
        directory = getattr(self, 'survey_debug_directory', None)
        if mode != 'survey' or directory is None:
            return
        saved = int(getattr(self, '_survey_debug_saved', 0))
        maximum = int(getattr(self, 'survey_debug_max_frames', 0))
        if maximum <= 0 or saved >= maximum:
            return
        now_s = time.monotonic()
        interval_s = max(
            0.1,
            float(getattr(self, 'survey_debug_interval_s', 1.0)),
        )
        last_s = float(getattr(self, '_survey_debug_last_s', -float('inf')))
        if now_s - last_s < interval_s:
            return
        session_directory = Path(directory) / self.session_id
        filename = (
            f'frame_{self.frame_id:06d}_panels_{len(panels):02d}.jpg'
        )
        try:
            session_directory.mkdir(parents=True, exist_ok=True)
            written = cv2.imwrite(str(session_directory / filename), frame)
        except (OSError, cv2.error) as exc:
            print(f'WARNING: survey debug frame write failed: {exc}', flush=True)
            self._survey_debug_last_s = now_s
            return
        self._survey_debug_last_s = now_s
        if not written:
            print('WARNING: survey debug frame write returned false', flush=True)
            return
        self._survey_debug_saved = saved + 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print(
            'Pi CPU survey frame saved: '
            f'{session_directory / filename}; panels={len(panels)}; '
            f'boxes={[(p.x, p.y, p.width, p.height) for p in panels]}; '
            f'mean={float(gray.mean()):.2f}; max={int(gray.max())}',
            flush=True,
        )

    def _show_frame(
        self,
        frame,
        *,
        mode,
        active_panel_id,
        control_connected,
        panels,
        selected,
        valid,
        panel_visible,
        dirt_found,
        dirt_values,
        inference_ms,
        invalid_reason,
    ) -> bool:
        """Give the existing result to the optional UI without new inference."""
        if self.viewer is None:
            return True
        from laptop_ai.visualization import VisualizationState

        state = VisualizationState(
            mode=mode,
            active_panel_id=active_panel_id,
            frame_id=self.frame_id,
            control_connected=control_connected,
            valid=valid,
            panel_visible=panel_visible,
            dirt_found=dirt_found,
            inference_ms=inference_ms,
            invalid_reason=invalid_reason,
            model_name=self.model_name,
            pi_ip=self.pi_ip,
            target_x_norm=self.target_x_norm,
            target_y_norm=self.target_y_norm,
            dirt_values=dirt_values,
        )
        return bool(
            self.viewer.show(
                frame,
                panels=panels,
                selected=selected,
                state=state,
            )
        )

    def _send_idle(self, frame, panel_id: int) -> None:
        now_ns = time.time_ns()
        height, width = frame.shape[:2]
        self._send(
            mode='idle',
            width=width,
            height=height,
            capture_ns=now_ns,
            inference_ns=now_ns,
            valid=False,
            panel_visible=False,
            panels=[],
            active_panel_id=panel_id,
            dirt_found=False,
            dirt_values=dict(ZERO_DIRT),
            inference_ms=0.0,
            invalid_reason='mission-idle-or-control-stale',
        )

    def _send(
        self,
        *,
        mode,
        width,
        height,
        capture_ns,
        inference_ns,
        valid,
        panel_visible,
        panels,
        active_panel_id,
        dirt_found,
        dirt_values,
        inference_ms,
        invalid_reason,
    ) -> None:
        self.sequence += 1
        send_ns = max(time.time_ns(), inference_ns)
        payload = encode_result(
            source_id=self.source_id,
            session_id=self.session_id,
            frame_id=self.frame_id,
            sequence=self.sequence,
            capture_timestamp_ns=capture_ns,
            inference_timestamp_ns=inference_ns,
            send_timestamp_ns=send_ns,
            mode=mode,
            image_width=width,
            image_height=height,
            valid=valid,
            panel_visible=panel_visible,
            panels=panels,
            active_panel_id=active_panel_id,
            dirt_found=dirt_found,
            inference_time_ms=inference_ms,
            invalid_reason=invalid_reason,
            model_name=self.model_name,
            **dirt_values,
        )
        self.result_socket.sendto(payload, self.pi_address)

    def close(self) -> None:
        try:
            if self.viewer is not None:
                self.viewer.close()
        finally:
            self.video.close()
            self.control_socket.close()
            self.result_socket.close()
