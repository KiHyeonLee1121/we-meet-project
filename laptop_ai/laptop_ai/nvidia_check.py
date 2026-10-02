"""Linux NVIDIA/ONNX Runtime preflight for the DA-DAKA laptop AI host."""

from __future__ import annotations

import platform
import subprocess
import sys


def _nvidia_smi() -> list[str]:
    command = [
        "nvidia-smi",
        "--query-gpu=index,name,driver_version,memory.total",
        "--format=csv,noheader,nounits",
    ]
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if completed.returncode != 0:
        message = completed.stderr.strip() or completed.stdout.strip() or "unknown error"
        raise RuntimeError(f"nvidia-smi failed: {message}")
    return [line.strip() for line in completed.stdout.splitlines() if line.strip()]


def main() -> int:
    print(f"platform={platform.system()} {platform.release()}")
    if platform.system() != "Linux":
        print("ERROR: production NVIDIA profile expects Linux", file=sys.stderr)
        return 2

    try:
        gpu_lines = _nvidia_smi()
    except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if not gpu_lines:
        print("ERROR: nvidia-smi reported no GPU", file=sys.stderr)
        return 2
    for line in gpu_lines:
        print(f"gpu={line}")

    try:
        import onnxruntime as ort
    except ImportError:
        print("ERROR: onnxruntime-gpu is not installed", file=sys.stderr)
        return 2

    providers = ort.get_available_providers()
    print(f"onnxruntime={ort.__version__}")
    print(f"providers={providers}")
    if "CUDAExecutionProvider" not in providers:
        print(
            "ERROR: CUDAExecutionProvider is unavailable; do not run the "
            "production GPU profile with CPU fallback",
            file=sys.stderr,
        )
        return 2

    if not any("5060" in line for line in gpu_lines):
        print(
            "WARNING: GPU name does not contain '5060'; re-benchmark the hardware profile",
            file=sys.stderr,
        )
    print("OK: Linux NVIDIA CUDA inference path is available")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
