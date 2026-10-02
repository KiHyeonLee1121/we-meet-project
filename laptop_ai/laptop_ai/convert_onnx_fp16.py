"""Convert an FP32 ONNX model to FP16 while keeping float32 model I/O."""

from __future__ import annotations

import argparse
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create an FP16 ONNX model for laptop GPU inference"
    )
    parser.add_argument("--input", required=True, help="Source FP32 ONNX model")
    parser.add_argument("--output", required=True, help="Destination FP16 ONNX model")
    parser.add_argument(
        "--allow-fp16-io",
        action="store_true",
        help="Also convert model inputs/outputs to FP16",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    source = Path(args.input).expanduser()
    destination = Path(args.output).expanduser()
    if not source.is_file():
        raise FileNotFoundError(f"ONNX model file does not exist: {source}")
    if source.resolve() == destination.resolve():
        raise ValueError("input and output ONNX paths must be different")

    try:
        import onnx
        from onnxconverter_common import float16
    except ImportError as exc:
        raise RuntimeError(
            "FP16 conversion tools are missing; install requirements-tools.txt"
        ) from exc

    model = onnx.load(str(source))
    converted = float16.convert_float_to_float16(
        model,
        keep_io_types=not args.allow_fp16_io,
    )
    onnx.checker.check_model(converted)
    destination.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(converted, str(destination))
    print(destination.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
