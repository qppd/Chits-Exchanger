"""
Lightweight inference benchmarking script for Raspberry Pi.

Usage examples (run on the Pi):
  python3 benchmark_inference.py --model yolo11n.pt --backend ultralytics --iters 100
  python3 benchmark_inference.py --model yolo11n.torchscript --backend torchscript --iters 200 --size 640

This script measures model-only latency (not camera I/O or postprocessing) so it's useful
to compare backends and model formats on-device.

Notes / assumptions:
 - ultralytics backend: calls `ultralytics.YOLO(model_path)` and runs `model(img)` where img
   is a synthetic OpenCV-style HxWxC uint8 image.
 - torchscript backend: loads the file with `torch.jit.load` and runs a synthetic torch.float32
   NCHW tensor. This assumes the TorchScript expects a tensor input; adjust input preprocessing
   if your TorchScript model expects a different format.
 - Run on the Pi and experiment with --size (e.g., 320, 416, 640) and --threads OS vars.
"""

import time
import argparse
import numpy as np
import os

def bench_ultralytics(model_path, iters=100, warmup=10, size=640):
    try:
        from ultralytics import YOLO
    except Exception as e:
        print(f"ultralytics not available: {e}")
        return

    print(f"Loading Ultraytics model: {model_path}")
    model = YOLO(model_path)

    # synthetic image HxWxC uint8
    img = (np.random.rand(size, size, 3) * 255).astype('uint8')

    # warmup
    for i in range(warmup):
        _ = model(img, verbose=False)

    # timed runs
    t0 = time.perf_counter()
    for i in range(iters):
        _ = model(img, verbose=False)
    t1 = time.perf_counter()

    total = t1 - t0
    print(f"Ultralytics: {iters} runs, total={total:.3f}s, avg={total/iters*1000:.2f} ms, FPS={iters/total:.2f}")


def bench_torchscript(model_path, iters=100, warmup=10, size=640, device='cpu'):
    try:
        import torch
    except Exception as e:
        print(f"torch not available: {e}")
        return

    print(f"Loading TorchScript model: {model_path}")
    model = torch.jit.load(model_path, map_location=device)
    model.eval()

    inp = torch.randn(1, 3, size, size, dtype=torch.float32, device=device)

    # warmup
    with torch.no_grad():
        for i in range(warmup):
            _ = model(inp)

    # timed runs
    with torch.no_grad():
        t0 = time.perf_counter()
        for i in range(iters):
            _ = model(inp)
        t1 = time.perf_counter()

    total = t1 - t0
    print(f"TorchScript: {iters} runs, total={total:.3f}s, avg={total/iters*1000:.2f} ms, FPS={iters/total:.2f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True, help='Path to model file (.pt, .torchscript, etc)')
    parser.add_argument('--backend', choices=['ultralytics','torchscript'], default='ultralytics')
    parser.add_argument('--iters', type=int, default=100, help='Number of timed iterations')
    parser.add_argument('--warmup', type=int, default=10, help='Number of warmup iterations')
    parser.add_argument('--size', type=int, default=640, help='Square input size (pixels)')
    parser.add_argument('--device', default='cpu', help='Device for torchscript (cpu)')
    args = parser.parse_args()

    # Print some diagnostics
    print('Python PID:', os.getpid())
    print('Model:', args.model)
    print('Backend:', args.backend)
    print('Iters:', args.iters, 'Warmup:', args.warmup, 'Size:', args.size)

    if args.backend == 'ultralytics':
        bench_ultralytics(args.model, iters=args.iters, warmup=args.warmup, size=args.size)
    elif args.backend == 'torchscript':
        bench_torchscript(args.model, iters=args.iters, warmup=args.warmup, size=args.size, device=args.device)


if __name__ == '__main__':
    main()
