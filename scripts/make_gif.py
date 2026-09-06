#!/usr/bin/env python3
"""Assemble a numbered PNG sequence into a GIF.

Used for the README recording; the frames themselves come from
`arm_viz --frames`, which renders offscreen and needs no window.
"""
import argparse
import glob
import os
import sys

try:
    from PIL import Image
except ImportError:
    sys.exit("this script needs Pillow: pip3 install --user pillow")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames_dir")
    ap.add_argument("output")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--delay-ms", type=int, default=50)
    ap.add_argument("--colors", type=int, default=64)
    ap.add_argument("--every", type=int, default=1, help="keep every Nth frame")
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.frames_dir, "frame_*.png")))[:: args.every]
    if not paths:
        sys.exit(f"no frames in {args.frames_dir}")

    frames = []
    for path in paths:
        img = Image.open(path).convert("RGB")
        if args.scale != 1.0:
            size = (round(img.width * args.scale), round(img.height * args.scale))
            img = img.resize(size, Image.LANCZOS)
        # One adaptive palette per frame keeps the dark UI from banding.
        frames.append(img.quantize(colors=args.colors, method=Image.MEDIANCUT))

    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=args.delay_ms,
        loop=0,
        optimize=True,
    )
    size_mb = os.path.getsize(args.output) / 1e6
    print(f"wrote {args.output}: {len(frames)} frames, {frames[0].size[0]}x{frames[0].size[1]}, "
          f"{size_mb:.1f} MB")


if __name__ == "__main__":
    main()
