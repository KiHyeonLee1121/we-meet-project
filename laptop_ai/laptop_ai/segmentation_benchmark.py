"""Benchmark the production binary-segmentation CUDA path."""

import argparse
import json
from pathlib import Path
import statistics
import time

import numpy as np
import yaml

from laptop_ai.onnx_dirt_detector import OnnxDirtSegmenter


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        raise ValueError('benchmark values cannot be empty')
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * quantile)))
    return ordered[index]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--runs', type=int, default=100)
    parser.add_argument('--output')
    args = parser.parse_args()
    if args.runs <= 0:
        raise ValueError('--runs must be positive')
    with Path(args.config).open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    model = config['dirt_model']
    detector = OnnxDirtSegmenter(
        str(model['path']),
        input_width=int(model['input_width']),
        input_height=int(model['input_height']),
        threshold=float(model['threshold']),
        minimum_area_ratio=float(model['minimum_area_ratio']),
        output_channel=int(model.get('output_channel', 0)),
        performance=config.get('performance'),
    )
    frame = np.zeros(
        (int(model['input_height']), int(model['input_width']), 3),
        dtype=np.uint8,
    )
    timings = []
    for _ in range(args.runs):
        started = time.perf_counter()
        detector.detect(frame)
        timings.append((time.perf_counter() - started) * 1000.0)
    report = {
        'runs': args.runs,
        'model': detector.model_name,
        'mean_ms': statistics.fmean(timings),
        'p50_ms': percentile(timings, 0.50),
        'p95_ms': percentile(timings, 0.95),
        'p99_ms': percentile(timings, 0.99),
        'fps_from_mean': 1000.0 / statistics.fmean(timings),
    }
    rendered = json.dumps(report, indent=2)
    print(rendered)
    if args.output:
        Path(args.output).write_text(rendered + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
