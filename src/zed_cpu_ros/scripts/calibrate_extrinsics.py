#!/usr/bin/env python3
"""Calibrate stereo extrinsics with OpenCV chessboard image pairs.

Example:
  python3 calibrate_extrinsics.py \
    --left-glob "/data/calib/left/*.png" \
    --right-glob "/data/calib/right/*.png" \
    --board-cols 9 --board-rows 6 --square-size 0.025 \
    --left-intrinsics ../config/left.yaml \
    --right-intrinsics ../config/right.yaml \
    --output stereo_extrinsics.yaml \
    --left-output left_stereo.yaml \
    --right-output right_stereo.yaml

  python3 calibrate_extrinsics.py \
    --raw-glob "/data/calib/stereo_raw/*.png" \
    --split-output-dir "/data/calib/stereo_split" \
    --board-cols 9 --board-rows 6 --square-size 0.025 \
    --left-intrinsics ../config/left_calibrated.yaml \
    --right-intrinsics ../config/right_calibrated.yaml \
    --output stereo_extrinsics.yaml
"""

import argparse
import glob
import os
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml


DEFAULT_LEFT_GLOB = "./left/*.png"
DEFAULT_RIGHT_GLOB = "./right/*.png"
DEFAULT_SPLIT_OUTPUT_DIR = "./stereo_split_images"
DEFAULT_BOARD_COLS = 9
DEFAULT_BOARD_ROWS = 6
DEFAULT_SQUARE_SIZE = 0.025
DEFAULT_OUTPUT = "stereo_extrinsics.yaml"
DEFAULT_LEFT_OUTPUT = "left_stereo.yaml"
DEFAULT_RIGHT_OUTPUT = "right_stereo.yaml"


def make_object_points(board_size: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = board_size
    points = np.zeros((rows * cols, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return points * square_size


def read_ros_camera_yaml(path: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int], str]:
    with open(path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float64).reshape(1, -1)
    image_size = (int(data["image_width"]), int(data["image_height"]))
    camera_name = data.get("camera_name", os.path.splitext(os.path.basename(path))[0])
    return camera_matrix, dist_coeffs, image_size, camera_name


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


def find_pair_corners(
    left_paths: Sequence[str],
    right_paths: Sequence[str],
    board_size: Tuple[int, int],
    square_size: float,
    show: bool,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    if len(left_paths) != len(right_paths):
        raise RuntimeError(
            f"Left/right image counts differ: {len(left_paths)} vs {len(right_paths)}"
        )

    obj_template = make_object_points(board_size, square_size)
    object_points = []
    left_points = []
    right_points = []
    image_size = None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for left_path, right_path in zip(left_paths, right_paths):
        left = cv2.imread(left_path)
        right = cv2.imread(right_path)
        if left is None or right is None:
            print(f"[WARN] Cannot read pair: {left_path} | {right_path}")
            continue

        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        pair_size = (left_gray.shape[1], left_gray.shape[0])
        if pair_size != (right_gray.shape[1], right_gray.shape[0]):
            print(f"[WARN] Skip different left/right sizes: {left_path} | {right_path}")
            continue

        if image_size is None:
            image_size = pair_size
        elif image_size != pair_size:
            print(f"[WARN] Skip pair with different size: {left_path} | {right_path}")
            continue

        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_NORMALIZE_IMAGE
            + cv2.CALIB_CB_FAST_CHECK
        )
        left_found, left_corners = cv2.findChessboardCorners(left_gray, board_size, flags)
        right_found, right_corners = cv2.findChessboardCorners(right_gray, board_size, flags)
        if not left_found or not right_found:
            print(f"[WARN] Chessboard not found in pair: {left_path} | {right_path}")
            continue

        left_refined = cv2.cornerSubPix(left_gray, left_corners, (11, 11), (-1, -1), criteria)
        right_refined = cv2.cornerSubPix(right_gray, right_corners, (11, 11), (-1, -1), criteria)
        object_points.append(obj_template.copy())
        left_points.append(left_refined)
        right_points.append(right_refined)
        print(f"[OK] {left_path} | {right_path}")

        if show:
            left_preview = left.copy()
            right_preview = right.copy()
            cv2.drawChessboardCorners(left_preview, board_size, left_refined, left_found)
            cv2.drawChessboardCorners(right_preview, board_size, right_refined, right_found)
            cv2.imshow("left corners", left_preview)
            cv2.imshow("right corners", right_preview)
            cv2.waitKey(100)

    if show:
        cv2.destroyAllWindows()

    if image_size is None:
        raise RuntimeError("No readable stereo image pairs were found.")

    return object_points, left_points, right_points, image_size


def format_matrix(data: np.ndarray) -> str:
    flat = data.reshape(-1)
    return "[" + ", ".join(f"{value:.9f}" for value in flat) + "]"


def write_ros_camera_yaml(
    path: str,
    camera_name: str,
    image_size: Tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rectification: np.ndarray,
    projection: np.ndarray,
) -> None:
    width, height = image_size
    dist = dist_coeffs.reshape(-1)
    content = f"""# Generated by calibrate_extrinsics.py
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
  data: {format_matrix(rectification)}
projection_matrix:
  rows: 3
  cols: 4
  data: {format_matrix(projection)}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        file.write(content)


def write_extrinsics_yaml(
    path: str,
    image_size: Tuple[int, int],
    rms: float,
    rotation: np.ndarray,
    translation: np.ndarray,
    essential: np.ndarray,
    fundamental: np.ndarray,
) -> None:
    width, height = image_size
    content = f"""# Generated by calibrate_extrinsics.py
image_width: {width}
image_height: {height}
reprojection_error: {rms:.9f}
rotation_matrix:
  rows: 3
  cols: 3
  data: {format_matrix(rotation)}
translation_vector:
  rows: 3
  cols: 1
  data: {format_matrix(translation)}
essential_matrix:
  rows: 3
  cols: 3
  data: {format_matrix(essential)}
fundamental_matrix:
  rows: 3
  cols: 3
  data: {format_matrix(fundamental)}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        file.write(content)


def calibrate_intrinsics_if_needed(
    object_points: List[np.ndarray],
    image_points: List[np.ndarray],
    image_size: Tuple[int, int],
    intrinsics_path: Optional[str],
) -> Tuple[np.ndarray, np.ndarray, str]:
    if intrinsics_path:
        camera_matrix, dist_coeffs, yaml_size, camera_name = read_ros_camera_yaml(intrinsics_path)
        if yaml_size != image_size:
            raise RuntimeError(
                f"Image size in {intrinsics_path} is {yaml_size}, but calibration images are {image_size}"
            )
        return camera_matrix, dist_coeffs, camera_name

    _rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    return camera_matrix, dist_coeffs, "camera"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate stereo camera extrinsics from paired chessboard images."
    )
    parser.add_argument("--left-glob", default=DEFAULT_LEFT_GLOB)
    parser.add_argument("--right-glob", default=DEFAULT_RIGHT_GLOB)
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
    parser.add_argument("--left-intrinsics")
    parser.add_argument("--right-intrinsics")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--left-output", default=DEFAULT_LEFT_OUTPUT)
    parser.add_argument("--right-output", default=DEFAULT_RIGHT_OUTPUT)
    parser.add_argument("--show", action="store_true")
    parser.add_argument(
        "--estimate-intrinsics",
        action="store_true",
        help="Allow stereo calibration to optimize intrinsics too. Default fixes intrinsics.",
    )
    parser.add_argument(
        "--min-valid-pairs",
        type=int,
        default=8,
        help="Minimum valid chessboard pairs required before calibration.",
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
    else:
        left_paths = sorted(glob.glob(args.left_glob))
        right_paths = sorted(glob.glob(args.right_glob))
        if not left_paths:
            raise RuntimeError(f"No images matched --left-glob: {args.left_glob}")
        if not right_paths:
            raise RuntimeError(f"No images matched --right-glob: {args.right_glob}")

    board_size = (args.board_cols, args.board_rows)
    object_points, left_points, right_points, image_size = find_pair_corners(
        left_paths,
        right_paths,
        board_size,
        args.square_size,
        args.show,
    )
    if len(object_points) < args.min_valid_pairs:
        raise RuntimeError(
            f"Only {len(object_points)} valid pairs found. "
            f"Use at least {args.min_valid_pairs}, preferably 15-30."
        )

    left_matrix, left_dist, left_name = calibrate_intrinsics_if_needed(
        object_points, left_points, image_size, args.left_intrinsics
    )
    right_matrix, right_dist, right_name = calibrate_intrinsics_if_needed(
        object_points, right_points, image_size, args.right_intrinsics
    )

    flags = 0 if args.estimate_intrinsics else cv2.CALIB_FIX_INTRINSIC
    rms, left_matrix, left_dist, right_matrix, right_dist, rotation, translation, essential, fundamental = (
        cv2.stereoCalibrate(
            object_points,
            left_points,
            right_points,
            left_matrix,
            left_dist,
            right_matrix,
            right_dist,
            image_size,
            flags=flags,
            criteria=(
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                100,
                1e-5,
            ),
        )
    )

    r1, r2, p1, p2, _q, _roi1, _roi2 = cv2.stereoRectify(
        left_matrix,
        left_dist,
        right_matrix,
        right_dist,
        image_size,
        rotation,
        translation,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0,
    )

    write_extrinsics_yaml(args.output, image_size, rms, rotation, translation, essential, fundamental)
    write_ros_camera_yaml(args.left_output, left_name, image_size, left_matrix, left_dist, r1, p1)
    write_ros_camera_yaml(args.right_output, right_name, image_size, right_matrix, right_dist, r2, p2)

    print("")
    print(f"Valid pairs: {len(object_points)} / {len(left_paths)}")
    print(f"Image size: {image_size[0]}x{image_size[1]}")
    print(f"RMS reprojection error: {rms:.6f}")
    print(f"Translation vector: {translation.reshape(-1)}")
    print(f"Saved extrinsics: {args.output}")
    print(f"Saved left camera YAML: {args.left_output}")
    print(f"Saved right camera YAML: {args.right_output}")


if __name__ == "__main__":
    main()
