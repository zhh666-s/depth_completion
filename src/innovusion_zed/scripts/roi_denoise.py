#!/usr/bin/env python3
import argparse
import glob
import os
from pathlib import Path

import cv2
import numpy as np


def compute_density_map(valid_mask: np.ndarray, kernel_size=(61, 61)) -> np.ndarray:
    return cv2.boxFilter(valid_mask.astype(np.float32), ddepth=-1, ksize=kernel_size, normalize=True)


def get_central_region(shape, x_frac=(0.1, 0.9), y_frac=(0.08, 0.92)):
    h, w = shape
    mask = np.zeros(shape, dtype=np.uint8)
    xmin = int(w * x_frac[0])
    xmax = int(w * x_frac[1])
    ymin = int(h * y_frac[0])
    ymax = int(h * y_frac[1])
    mask[ymin:ymax, xmin:xmax] = 1
    return mask


def remove_dense_roi(depth: np.ndarray, kernel_size=(61, 61), density_percentile=92, min_density=0.08) -> np.ndarray:
    if depth.ndim != 2:
        raise ValueError('depth image must be single-channel')

    valid_mask = (depth > 0).astype(np.uint8)
    if valid_mask.sum() == 0:
        return depth.copy()

    density = compute_density_map(valid_mask, kernel_size=kernel_size)
    threshold = max(min_density, np.percentile(density, density_percentile))
    roi = (density >= threshold).astype(np.uint8)

    # 限制 ROI 只在图像中心区域内，避免边缘误处理
    central_region = get_central_region(depth.shape, x_frac=(0.08, 0.92), y_frac=(0.05, 0.95))
    roi = cv2.bitwise_and(roi, central_region)

    if roi.sum() < 50:
        return depth.copy()

    roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    hole_mask = cv2.erode(cv2.bitwise_and(roi, valid_mask), np.ones((5, 5), np.uint8))

    if hole_mask.sum() < 10:
        hole_mask = cv2.bitwise_and(roi, valid_mask)
    if hole_mask.sum() == 0:
        return depth.copy()

    depth_float = depth.astype(np.float32)
    depth_norm = depth_float / 10000.0
    inpainted = cv2.inpaint(depth_norm, hole_mask, 7, cv2.INPAINT_TELEA) * 10000.0

    output = depth_float.copy()
    output[hole_mask > 0] = inpainted[hole_mask > 0]

    # 对 ROI 区域做轻度平滑，使其与周围一致
    smoothed = cv2.medianBlur(output.astype(np.uint16), 5).astype(np.float32)
    output[hole_mask > 0] = smoothed[hole_mask > 0]

    return np.clip(output, 0, 65535).astype(np.uint16)


def process_file(input_path: str, output_path: str) -> bool:
    depth = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)
    if depth is None:
        print(f'WARN: cannot read {input_path}')
        return False
    denoised = remove_dense_roi(depth)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    success = cv2.imwrite(output_path, denoised)
    if not success:
        print(f'ERROR: cannot write {output_path}')
    return success


def batch_process(input_dir: str, output_dir: str, pattern='*.png') -> None:
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    paths = sorted(input_dir.glob(pattern))
    if not paths:
        raise FileNotFoundError(f'No matching files under {input_dir}')

    print(f'Processing {len(paths)} files from {input_dir} -> {output_dir}')
    for path in paths:
        rel = path.relative_to(input_dir)
        out_path = output_dir / rel
        process_file(str(path), str(out_path))

    print('Done.')


def parse_args():
    parser = argparse.ArgumentParser(description='Denoise ROI artifacts in sparse depth maps.')
    parser.add_argument('--input', required=True, help='Input depth folder containing PNG files.')
    parser.add_argument('--output', required=True, help='Output folder for denoised depth PNGs.')
    parser.add_argument('--pattern', default='*.png', help='Filename glob pattern for depth images.')
    parser.add_argument('--kernel', type=int, default=61, help='Kernel size for local density estimation.')
    parser.add_argument('--percentile', type=int, default=92, help='Density percentile threshold for ROI mask.')
    parser.add_argument('--min-density', type=float, default=0.08, help='Minimum normalized density threshold for ROI detection.')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    if args.kernel % 2 == 0:
        raise ValueError('kernel must be odd')
    batch_process(args.input, args.output, pattern=args.pattern)
