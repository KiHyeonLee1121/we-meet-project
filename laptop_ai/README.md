# DA-DAKA laptop AI monitor

This process borrows the laptop's NVIDIA CUDA resources while the Raspberry
Pi 5 remains the sole mission and flight-control owner.

1. Install a recent NVIDIA driver and CUDA-compatible `onnxruntime-gpu`.
2. Put the trained **binary dirt-segmentation** model at the path configured
   in `config/laptop_ai.yaml`. Expected output is `[1,1,H,W]`, `[1,H,W]` or
   `[H,W]` logits/probabilities.
3. Set `network.pi_ip` to the Pi address and open UDP 5005, 5006 and 5600 on
   the private field network.
4. Install and run the integrated monitor:

   ```bash
   chmod +x tools/start_laptop_ai_viewer.sh
   ./tools/start_laptop_ai_viewer.sh --pi-ip <PI_IP>
   ```

The launcher creates `.venv`, installs the package, checks the NVIDIA CUDA
provider and opens the monitor. Put the trained model at
`models/dirt_segmentation.onnx`, or pass `--model <ONNX_PATH>`. Use
`--skip-install` after the first successful setup.

The monitor is the only user-facing laptop AI runtime. Internally it uses one
UDP decoder and sends the exact same panel/dirt result drawn in the OpenCV
window to the Pi; video reception and ONNX inference are not duplicated.

Overlay legend and keys:

- blue boxes: every panel candidate
- green box: the panel selected for cleaning
- red box/cross: dirt segmentation bbox and centroid
- header: Pi control link, mission mode, panel/frame ID and inference latency
- `Q`/`Esc`: quit, `S`: save screenshot, `F`: toggle fullscreen

Start the monitor first, then start the existing Pi stream from another laptop
terminal:

```bash
PI_IP=<PI_IP> LAPTOP_IP=<LAPTOP_IP> PI_PROJECT=<PI_REPOSITORY> \
  ./tools/gpu_laptop_start_pi_camera.sh
```

The application rejects CPU-only ONNX Runtime. It decodes the Pi's low-latency
MPEG-TS/H.264 stream through PyAV/FFmpeg on UDP 5600, receives mission mode on
UDP 5006 only from the configured Pi IP/source ID, and sends validated
protocol-v2 results to the Pi on UDP 5005. The Pi mission launch applies the
same IP/source-ID allowlist to incoming laptop results. Use an isolated field
network and firewall as this allowlist is not cryptographic authentication.

Useful deployment checks and profiling commands:

```bash
da-daka-nvidia-check
da-daka-segmentation-benchmark \
  --config laptop_ai/config/laptop_ai.yaml --runs 200
da-daka-joint-optimizer \
  --config laptop_ai/config/pendulum_optimization.yaml \
  --bandwidth-mbps 20 --compute-ms 33.3
```

Install FP16 conversion extras only on the model-preparation machine:

```bash
python -m pip install -e './laptop_ai[tools]'
```

The Pendulum-inspired optimizer is wired into the worker in **observe-only**
mode. It can log a profiled bitrate/model recommendation after a scene change,
but cannot modify the Pi encoder or flight control. Replace the example profile
accuracy values with measurements before enabling it; apply mode fails closed
until a measured encoder/model adapter exists.

No model weights are committed because trained project weights are required;
an arbitrary placeholder model would make autonomous spray decisions unsafe.
