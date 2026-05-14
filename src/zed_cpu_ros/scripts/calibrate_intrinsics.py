#!/usr/bin/env python3
"""Calibrate a single camera with OpenCV chessboard images.

Example:
  python3 calibrate_intrinsics.py \
    --image-glob "/data/calib/left/*.png" \
    --board-cols 9 --board-rows 6 --square-size 0.025 \
    --camera-name camera/left \
    --output ../config/left_calibrated.yaml

  python3 calibrate_intrinsics.py \
    --raw-glob "/data/calib/raw/*.png" \
    --split-output-dir "/data/calib/split" \
    --camera-side left \
    --board-cols 9 --board-rows 6 --square-size 0.025 \
    --camera-name camera/left \
    --output ../config/left_calibrated.yaml
"""

import argparse
import glob
import os
from typing import List, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_IMAGE_GLOB = "./calib_images/*.png"
DEFAULT_SPLIT_OUTPUT_DIR = "./split_images"
DEFAULT_BOARD_COLS = 9  # Number of inner corners per chessboard row.
DEFAULT_BOARD_ROWS = 6  # Number of inner corners per chessboard column.
DEFAULT_SQUARE_SIZE = 0.025  # Chessboard square size in meters.
DEFAULT_CAMERA_NAME = "camera"
DEFAULT_OUTPUT = "camera_intrinsics.yaml"


def make_object_points(board_size: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = board_size
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return points * square_size


def split_side_by_side_images(
    raw_paths: Sequence[str],
    output_dir: str,
    expected_width: int,
    expected_height: int,
) -> Tuple[List[str], List[str]]:
    left_dir = os.path.join(output_dir, "left")
    right_dir = os.path.join(output_dir, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    left_paths = []
    right_paths = []
    for index, raw_path in enumerate(raw_paths):
        image = cv2.imread(raw_path)
        if image is None:
            print(f"[WARN] Cannot read raw image: {raw_path}")
            continue

        height, width = image.shape[:2]
        if expected_width > 0 and width != expected_width:
            print(f"[WARN] Raw image width is {width}, expected {expected_width}: {raw_path}")
        if expected_height > 0 and height != expected_height:
            print(f"[WARN] Raw image height is {height}, expected {expected_height}: {raw_path}")
        if width % 2 != 0:
            print(f"[WARN] Skip raw image with odd width: {raw_path}")
            continue

        middle = width // 2
        left_image = image[:, :middle]
        right_image = image[:, middle:]
        stem = os.path.splitext(os.path.basename(raw_path))[0]
        left_path = os.path.join(left_dir, f"{index:06d}_{stem}.png")
        right_path = os.path.join(right_dir, f"{index:06d}_{stem}.png")
        if not cv2.imwrite(left_path, left_image):
            raise RuntimeError(f"Failed to write split image: {left_path}")
        if not cv2.imwrite(right_path, right_image):
            raise RuntimeError(f"Failed to write split image: {right_path}")
        left_paths.append(left_path)
        right_paths.append(right_path)

    if not left_paths:
        raise RuntimeError("No raw side-by-side images were split successfully.")

    print(f"Split raw images into: {left_dir} and {right_dir}")
    return left_paths, right_paths


def find_corners(
    image_paths: Sequence[str],
    board_size: Tuple[int, int],
    square_size: float,
    show: bool,
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    obj_template = make_object_points(board_size, square_size)
    object_points = []
    image_points = []
    image_size = None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for path in image_paths:
        image = cv2.imread(path)
        if image is None:
            print(f"[WARN] Cannot read image: {path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif image_size != (gray.shape[1], gray.shape[0]):
            print(f"[WARN] Skip image with different size: {path}")
            continue

        found, corners = cv2.findChessboardCorners(
            gray,
            board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FAST_CHECK,
        )

        if not found:
            print(f"[WARN] Chessboard not found: {path}")
            continue

        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(obj_template.copy())
        image_points.append(refined)
        print(f"[OK] {path}")

        if show:
            preview = image.copy()
            cv2.drawChessboardCorners(preview, board_size, refined, found)
            cv2.imshow("corners", preview)
            cv2.waitKey(100)

    if show:
        cv2.destroyAllWindows()

    if image_size is None:
        raise RuntimeError("No readable images were found.")

    return object_points, image_points, image_size


def format_matrix(data: np.ndarray) -> str:
    flat = data.reshape(-1)
    return "[" + ", ".join(f"{value:.9f}" for value in flat) + "]"


def write_ros_camera_yaml(
    path: str,
    camera_name: str,
    image_size: Tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    reprojection_error: float,
) -> None:
    width, height = image_size
    dist = dist_coeffs.reshape(-1)
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:3, :3] = camera_matrix

    content = f"""# Generated by calibrate_intrinsics.py
# reprojection_error: {reprojection_error:.9f}
image_width: {width}
image_height: {height}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: {format_matrix(camera_matrix)}
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: {len(dist)}
  data: {format_matrix(dist)}
rectification_matrix:
  rows: 3
  cols: 3
  data: {format_matrix(np.eye(3, dtype=np.float64))}
projection_matrix:
  rows: 3
  cols: 4
  data: {format_matrix(projection)}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        file.write(content)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate single-camera intrinsics from chessboard images."
    )
    parser.add_argument("--image-glob", default=DEFAULT_IMAGE_GLOB)
    parser.add_argument(
        "--raw-glob",
        help="3840x1080 side-by-side raw image glob. If set, images are split first.",
    )
    parser.add_argument(
        "--split-output-dir",
        default=DEFAULT_SPLIT_OUTPUT_DIR,
        help="Directory used to save split left/right images when --raw-glob is set.",
    )
    parser.add_argument(
        "--camera-side",
        choices=("left", "right"),
        default="left",
        help="Which half of --raw-glob images to calibrate.",
    )
    parser.add_argument(
        "--raw-width",
        type=int,
        default=3840,
        help="Expected side-by-side raw image width. Use 0 to skip this check.",
    )
    parser.add_argument(
        "--raw-height",
        type=int,
        default=1080,
        help="Expected side-by-side raw image height. Use 0 to skip this check.",
    )
    parser.add_argument("--board-cols", type=int, default=DEFAULT_BOARD_COLS)
    parser.add_argument("--board-rows", type=int, default=DEFAULT_BOARD_ROWS)
    parser.add_argument("--square-size", type=float, default=DEFAULT_SQUARE_SIZE)
    parser.add_argument("--camera-name", default=DEFAULT_CAMERA_NAME)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--show", action="store_true")
    parser.add_argument(
        "--fix-k3",
        action="store_true",
        help="Fix the third radial distortion coefficient to zero.",
    )
    parser.add_argument(
        "--min-valid-images",
        type=int,
        default=8,
        help="Minimum valid chessboard images required before calibration.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.raw_glob:
        raw_paths = sorted(glob.glob(args.raw_glob))
        if not raw_paths:
            raise RuntimeError(f"No images matched --raw-glob: {args.raw_glob}")
        left_paths, right_paths = split_side_by_side_images(
            raw_paths,
            args.split_output_dir,
            args.raw_width,
            args.raw_height,
        )
        image_paths = left_paths if args.camera_side == "left" else right_paths
    else:
        image_paths = sorted(glob.glob(args.image_glob))
        if not image_paths:
            raise RuntimeError(f"No images matched --image-glob: {args.image_glob}")

    board_size = (args.board_cols, args.board_rows)
    object_points, image_points, image_size = find_corners(
        image_paths, board_size, args.square_size, args.show
    )
    if len(object_points) < args.min_valid_images:
        raise RuntimeError(
            f"Only {len(object_points)} valid images found. "
            f"Use at least {args.min_valid_images}, preferably 15-30."
        )

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3

    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=flags,
    )

    write_ros_camera_yaml(
        args.output,
        args.camera_name,
        image_size,
        camera_matrix,
        dist_coeffs,
        rms,
    )

    print("")
    print(f"Valid images: {len(object_points)} / {len(image_paths)}")
    print(f"Image size: {image_size[0]}x{image_size[1]}")
    print(f"RMS reprojection error: {rms:.6f}")
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
