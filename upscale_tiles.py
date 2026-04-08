#!/usr/bin/env python3
"""
Batch upscale SUMO background tile images using Real-ESRGAN (4x super-resolution).
Processes all JPEG tiles in-place, keeping the same filenames so settings.xml
doesn't need updating.

Usage:
    conda run -n realesrgan python upscale_tiles.py
"""

import os
import sys
import glob
import time
import shutil
import types
from pathlib import Path

# --- Monkey-patch to fix basicsr / torchvision compatibility ---
# basicsr imports from torchvision.transforms.functional_tensor which was
# removed in torchvision >= 0.18. We create a shim module before importing.
import torchvision.transforms.functional as _tvf
_shim = types.ModuleType("torchvision.transforms.functional_tensor")
_shim.rgb_to_grayscale = _tvf.rgb_to_grayscale
sys.modules["torchvision.transforms.functional_tensor"] = _shim
# --- End monkey-patch ---

import cv2
import numpy as np
import torch
from basicsr.archs.rrdbnet_arch import RRDBNet
from realesrgan import RealESRGANer

TILE_DIR = "san_jose_full_new/background_images"
BACKUP_DIR = "san_jose_full_new/background_images_backup_256"
SCALE = 4  # 4x upscale: 256x256 -> 1024x1024
MODEL_URL = "https://github.com/xinntao/Real-ESRGAN/releases/download/v0.1.0/RealESRGAN_x4plus.pth"


def setup_model():
    """Initialize Real-ESRGAN 4x model."""
    model = RRDBNet(
        num_in_ch=3, num_out_ch=3, num_feat=64,
        num_block=23, num_grow_ch=32, scale=4
    )
    # Download model weights to a local cache dir
    model_dir = os.path.join(os.path.dirname(__file__), "weights")
    os.makedirs(model_dir, exist_ok=True)
    model_path = os.path.join(model_dir, "RealESRGAN_x4plus.pth")

    if not os.path.exists(model_path):
        print(f"Downloading model weights to {model_path} ...")
        from urllib.request import urlretrieve
        urlretrieve(MODEL_URL, model_path)
        print("Download complete.")

    upsampler = RealESRGANer(
        scale=SCALE,
        model_path=model_path,
        model=model,
        tile=256,         # process in tiles to save GPU memory
        tile_pad=10,
        pre_pad=0,
        half=True,        # fp16 for speed on RTX 4080
        gpu_id=0
    )
    return upsampler


def main():
    tile_dir = Path(TILE_DIR)
    backup_dir = Path(BACKUP_DIR)

    # Find all JPEG tiles (exclude settings.xml and other files)
    jpeg_files = sorted(glob.glob(str(tile_dir / "tile*.jpeg")))
    total = len(jpeg_files)
    print(f"Found {total} tile images to upscale")

    if total == 0:
        print("No tile images found!")
        return

    # Create backup of originals
    if not backup_dir.exists():
        print(f"Backing up originals to {backup_dir}/ ...")
        backup_dir.mkdir(parents=True)
        for f in jpeg_files:
            shutil.copy2(f, backup_dir / Path(f).name)
        # Also backup settings.xml
        settings_src = tile_dir / "settings.xml"
        if settings_src.exists():
            shutil.copy2(settings_src, backup_dir / "settings.xml")
        print("Backup complete.")
    else:
        print(f"Backup directory already exists, skipping backup.")

    # Setup model
    print("Loading Real-ESRGAN model...")
    upsampler = setup_model()
    print("Model loaded. Starting upscaling...")

    start_time = time.time()
    processed = 0
    errors = 0

    for i, filepath in enumerate(jpeg_files):
        fname = Path(filepath).name
        try:
            img = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
            if img is None:
                print(f"  [{i+1}/{total}] SKIP (cannot read): {fname}")
                errors += 1
                continue

            output, _ = upsampler.enhance(img, outscale=SCALE)

            # Save back as JPEG with high quality
            cv2.imwrite(filepath, output, [cv2.IMWRITE_JPEG_QUALITY, 95])
            processed += 1

            if (i + 1) % 200 == 0 or (i + 1) == total:
                elapsed = time.time() - start_time
                rate = (i + 1) / elapsed
                eta = (total - i - 1) / rate if rate > 0 else 0
                print(f"  [{i+1}/{total}] {fname} done "
                      f"({rate:.1f} img/s, ETA: {eta:.0f}s)")

        except Exception as e:
            print(f"  [{i+1}/{total}] ERROR on {fname}: {e}")
            errors += 1

    elapsed = time.time() - start_time
    print(f"\nDone! Processed: {processed}, Errors: {errors}, "
          f"Time: {elapsed:.1f}s ({elapsed/60:.1f}min)")

    # Verify a sample
    sample = jpeg_files[0]
    img = cv2.imread(sample)
    if img is not None:
        print(f"Sample verification: {Path(sample).name} -> {img.shape[1]}x{img.shape[0]}")


if __name__ == "__main__":
    main()
