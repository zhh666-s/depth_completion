#!/usr/bin/env python3

"""Calibrate each EM4 capture folder independently and render intensity overlays.

The original checkerboard evaluator in this directory estimates one extrinsic
from many frames.  The EM4 capture dataset stores one stereo image pair and one
PCD per capture folder, and the LiDAR-camera relation may change between
captures.  This script therefore solves a conservative per-capture transform:

* detect the checkerboard in the left image;
* fit the corresponding board plane in the LiDAR cloud;
* align the LiDAR board plane normal and center to the camera board pose,
  keeping the in-plane rotation closest to the standard x-forward LiDAR prior;
* write a calibration JSON and an intensity-colored projected overlay into the
  capture folder.

One image/PCD pair with only a planar checkerboard cannot fully constrain all
6 DoF unless LiDAR checkerboard corners are also detected.  The in-plane degree
of freedom is resolved with the rough LiDAR frame convention prior.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


AX = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=np.float64)

# Common LiDAR convention for this EM4 dataset: x forward, y left, z up.
R_LIDAR_TO_CAMERA_PRIOR = np.array(
    [[0, -1, 0], [0, 0, -1], [1, 0, 0]],
    dtype=np.float64,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset-dir",
        default="src/innovusion_zed/data/EM4_calibration20260521",
        help="Directory containing capture_* subdirectories.",
    )
    parser.add_argument(
        "--config",
        default="src/innovusion_zed/config/camera_left.json",
        help="Camera intrinsic/distortion JSON used as the base output config.",
    )
    parser.add_argument("--pattern-cols", type=int, default=12)
    parser.add_argument("--pattern-rows", type=int, default=18)
    parser.add_argument("--square-size", type=float, default=0.045)
    parser.add_argument("--roi-margin", type=float, default=120.0)
    parser.add_argument("--plane-threshold", type=float, default=0.025)
    parser.add_argument("--max-depth", type=float, default=80.0)
    parser.add_argument(
        "--max-rough-rotation-deg",
        type=float,
        default=35.0,
        help="Reject per-capture solutions that rotate this far away from the x-forward LiDAR prior.",
    )
    parser.add_argument(
        "--max-translation-norm",
        type=float,
        default=1.5,
        help="Reject implausible LiDAR-to-camera translations in meters.",
    )
    parser.add_argument("--point-size", type=int, default=1)
    parser.add_argument("--alpha", type=float, default=0.65)
    parser.add_argument(
        "--write-depth-png",
        action="store_true",
        default=True,
        help="Write 16-bit projected sparse depth image in millimeters.",
    )
    return parser.parse_args()


def read_pcd_points(path):
    with open(path, "rb") as f:
        meta = {}
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            parts = line.split()
            if parts:
                meta[parts[0]] = parts[1:]
            if line.startswith("DATA"):
                data_type = parts[1]
                break

        fields = meta["FIELDS"]
        sizes = list(map(int, meta["SIZE"]))
        types = meta["TYPE"]
        n_points = int(meta.get("POINTS", meta.get("WIDTH"))[0])

        if data_type == "ascii":
            arr = np.loadtxt(f, dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            cols = {name: i for i, name in enumerate(fields)}
            xyz = arr[:, [cols["x"], cols["y"], cols["z"]]].astype(np.float64)
            intensity = (
                arr[:, cols["intensity"]].astype(np.float32)
                if "intensity" in cols
                else np.zeros(len(xyz), dtype=np.float32)
            )
            return xyz, intensity, fields

        dtype_fields = []
        for name, size, type_name in zip(fields, sizes, types):
            if type_name == "F" and size == 4:
                dtype = "<f4"
            elif type_name == "F" and size == 8:
                dtype = "<f8"
            elif type_name == "U" and size == 4:
                dtype = "<u4"
            elif type_name == "U" and size == 2:
                dtype = "<u2"
            elif type_name == "U" and size == 1:
                dtype = "u1"
            elif type_name == "I" and size == 4:
                dtype = "<i4"
            else:
                dtype = "<f4"
            dtype_fields.append((name, dtype))

        arr = np.frombuffer(
            f.read(n_points * np.dtype(dtype_fields).itemsize),
            dtype=np.dtype(dtype_fields),
            count=n_points,
        )

    xyz = np.vstack([arr["x"], arr["y"], arr["z"]]).T.astype(np.float64)
    intensity = (
        arr["intensity"].astype(np.float32)
        if "intensity" in arr.dtype.names
        else np.zeros(len(xyz), dtype=np.float32)
    )
    return xyz, intensity, fields


def detect_checkerboard(image, pattern):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags_sb = cv2.CALIB_CB_NORMALIZE_IMAGE
    for optional in ("CALIB_CB_EXHAUSTIVE", "CALIB_CB_ACCURACY"):
        if hasattr(cv2, optional):
            flags_sb |= getattr(cv2, optional)

    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags_sb)
        if ok:
            return True, corners.astype(np.float32)

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FILTER_QUADS
    )
    variants = [gray, cv2.equalizeHist(gray)]
    for variant in variants:
        ok, corners = cv2.findChessboardCorners(variant, pattern, flags=flags)
        if ok:
            cv2.cornerSubPix(
                variant,
                corners,
                (11, 11),
                (-1, -1),
                (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                ),
            )
            return True, corners.astype(np.float32)

    return False, None


def checkerboard_object_points(pattern, square_size):
    obj = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0 : pattern[0], 0 : pattern[1]].T.reshape(-1, 2)
    obj *= square_size
    return obj


def solve_camera_board_pose(corners, k, dist, pattern, square_size):
    obj = checkerboard_object_points(pattern, square_size)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    r_board = cv2.Rodrigues(rvec)[0]
    t_board = tvec.ravel()
    normal = r_board[:, 2].copy()
    normal /= np.linalg.norm(normal)
    center_obj = np.array(
        [
            (pattern[0] - 1) * square_size / 2.0,
            (pattern[1] - 1) * square_size / 2.0,
            0.0,
        ],
        dtype=np.float64,
    )
    center = r_board @ center_obj + t_board
    reproj = cv2.projectPoints(obj, rvec, tvec, k, dist)[0].reshape(-1, 2)
    err = reproj - corners.reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    return {
        "r_board": r_board,
        "t_board": t_board,
        "normal": normal,
        "center": center,
        "reprojection_rmse_px": rmse,
    }


def project_points(xyz, r_lidar_to_camera, t_lidar_to_camera, k, dist, min_depth, max_depth):
    finite = np.isfinite(xyz).all(axis=1)
    cam = (r_lidar_to_camera @ xyz[finite].T + t_lidar_to_camera[:, None]).T
    source_indices = np.flatnonzero(finite)
    valid_z = (cam[:, 2] >= min_depth) & (cam[:, 2] <= max_depth)
    cam = cam[valid_z]
    source_indices = source_indices[valid_z]
    uv = np.empty((0, 2), dtype=np.float64)
    if len(cam):
        uv = cv2.projectPoints(
            cam.reshape(-1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            k,
            dist,
        )[0].reshape(-1, 2)
    return uv, cam, source_indices


def fit_plane(points, target_normal, threshold, max_normal_angle_deg=55.0):
    if len(points) < 200:
        return None

    rng = np.random.default_rng(7)
    best_mask = None
    best_score = -1.0
    iterations = min(700, max(180, len(points) // 80))
    target_normal = target_normal / np.linalg.norm(target_normal)

    for _ in range(iterations):
        ids = rng.choice(len(points), 3, replace=False)
        sample = points[ids]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-8:
            continue
        normal = normal / normal_norm
        if normal @ target_normal < 0:
            normal = -normal
        angle = math.degrees(
            math.acos(float(np.clip(normal @ target_normal, -1.0, 1.0)))
        )
        if angle > max_normal_angle_deg:
            continue

        d = -normal @ sample[0]
        mask = np.abs(points @ normal + d) < threshold
        count = int(mask.sum())
        score = count * (1.0 - 0.35 * angle / max_normal_angle_deg)
        if score > best_score:
            best_score = score
            best_mask = mask

    if best_mask is None or int(best_mask.sum()) < 120:
        return None

    inliers = points[best_mask]
    center = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    if normal @ target_normal < 0:
        normal = -normal
    d = -normal @ center
    angle = math.degrees(math.acos(float(np.clip(normal @ target_normal, -1.0, 1.0))))
    return {
        "normal": normal,
        "d": float(d),
        "center": np.median(inliers, axis=0),
        "mean_center": center,
        "inliers": int(best_mask.sum()),
        "total": int(len(points)),
        "normal_angle_to_prior_deg": float(angle),
    }


def align_vectors(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(a @ b)
    s = float(np.linalg.norm(v))
    if s < 1e-10:
        if c > 0:
            return np.eye(3, dtype=np.float64)
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a @ axis) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v /= np.linalg.norm(v)
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + 2.0 * vx @ vx

    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def direct_to_config_rotation_translation(r_lidar_to_camera, t_lidar_to_camera):
    r_sensor = AX @ r_lidar_to_camera.T
    t_sensor = -(r_sensor @ t_lidar_to_camera)
    return r_sensor, t_sensor


def colorize_intensity(values):
    if len(values) == 0:
        return np.empty((0, 3), dtype=np.uint8), 0.0, 1.0

    lo = float(np.percentile(values, 2.0))
    hi = float(np.percentile(values, 98.0))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo = float(np.min(values))
        hi = float(np.max(values))
    if hi <= lo:
        hi = lo + 1.0
    norm = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
    colors = cv2.applyColorMap((norm * 255.0).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)
    return colors.reshape(-1, 3), lo, hi


def render_intensity_projection(
    image,
    xyz,
    intensity,
    r_lidar_to_camera,
    t_lidar_to_camera,
    k,
    dist,
    max_depth,
    point_size,
    alpha,
):
    height, width = image.shape[:2]
    uv, cam, source_indices = project_points(
        xyz, r_lidar_to_camera, t_lidar_to_camera, k, dist, 0.2, max_depth
    )
    if len(cam) == 0:
        return image.copy(), np.zeros((height, width), dtype=np.float32), {
            "projected_pixels": 0,
            "intensity_min": None,
            "intensity_max": None,
        }

    px = np.rint(uv[:, 0]).astype(np.int32)
    py = np.rint(uv[:, 1]).astype(np.int32)
    inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    px = px[inside]
    py = py[inside]
    depths = cam[inside, 2].astype(np.float32)
    source_indices = source_indices[inside]
    projected_intensity = intensity[source_indices].astype(np.float32)
    if len(depths) == 0:
        return image.copy(), np.zeros((height, width), dtype=np.float32), {
            "projected_pixels": 0,
            "intensity_min": None,
            "intensity_max": None,
        }

    flat = py * width + px
    order = np.lexsort((depths, flat))
    flat = flat[order]
    depths = depths[order]
    projected_intensity = projected_intensity[order]
    keep = np.r_[True, flat[1:] != flat[:-1]]
    flat = flat[keep]
    depths = depths[keep]
    projected_intensity = projected_intensity[keep]

    pixels = np.vstack((flat % width, flat // width)).T.astype(np.int32)
    colors, intensity_lo, intensity_hi = colorize_intensity(projected_intensity)
    point_layer = image.copy()
    if point_size <= 0:
        point_layer[pixels[:, 1], pixels[:, 0]] = colors
    else:
        for (x, y), color in zip(pixels, colors):
            cv2.circle(
                point_layer,
                (int(x), int(y)),
                point_size,
                tuple(int(v) for v in color),
                -1,
                cv2.LINE_AA,
            )
    overlay = cv2.addWeighted(image, 1.0 - alpha, point_layer, alpha, 0)

    depth_map = np.zeros(height * width, dtype=np.float32)
    depth_map[flat] = depths
    depth_map = depth_map.reshape(height, width)
    return overlay, depth_map, {
        "projected_pixels": int(len(pixels)),
        "projected_depth_min": float(depths.min()),
        "projected_depth_max": float(depths.max()),
        "projected_intensity_min": float(projected_intensity.min()),
        "projected_intensity_max": float(projected_intensity.max()),
        "color_intensity_min": float(intensity_lo),
        "color_intensity_max": float(intensity_hi),
    }


def draw_checkerboard(image, corners, pattern):
    out = image.copy()
    cv2.drawChessboardCorners(out, pattern, corners, True)
    return out


def angle_between_rotations(a, b):
    value = float((np.trace(a @ b.T) - 1.0) / 2.0)
    return math.degrees(math.acos(max(-1.0, min(1.0, value))))


def find_single_file(group, pattern):
    matches = sorted(group.glob(pattern))
    return matches[0] if len(matches) == 1 else None


def clear_previous_outputs(group):
    for name in (
        "lidar_camera_calibration_left.json",
        "lidar_intensity_overlay.png",
        "lidar_projected_depth_mm.png",
        "checkerboard_detection_overlay.png",
        "checkerboard_detection_failed.png",
    ):
        path = group / name
        if path.exists():
            path.unlink()


def calibrate_group(group, base_config, k, dist, args):
    clear_previous_outputs(group)
    pattern = (args.pattern_cols, args.pattern_rows)
    image_path = find_single_file(group, "*_left.png")
    pcd_path = find_single_file(group, "*.pcd")
    if image_path is None or pcd_path is None:
        return {"status": "failed", "reason": "missing_unique_left_image_or_pcd"}

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return {"status": "failed", "reason": "failed_to_read_left_image"}

    ok, corners = detect_checkerboard(image, pattern)
    if not ok:
        cv2.imwrite(str(group / "checkerboard_detection_failed.png"), image)
        return {"status": "failed", "reason": "checkerboard_not_detected"}

    camera_board = solve_camera_board_pose(corners, k, dist, pattern, args.square_size)
    if camera_board is None:
        return {"status": "failed", "reason": "camera_solvepnp_failed"}

    xyz, intensity, fields = read_pcd_points(pcd_path)
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    intensity = intensity[finite]
    pcd_intensity_stats = {
        "fields": list(fields),
        "has_intensity": "intensity" in fields,
        "points": int(len(xyz)),
        "intensity_min": float(np.min(intensity)) if len(intensity) else None,
        "intensity_max": float(np.max(intensity)) if len(intensity) else None,
        "intensity_mean": float(np.mean(intensity)) if len(intensity) else None,
        "intensity_std": float(np.std(intensity)) if len(intensity) else None,
        "intensity_nonzero_ratio": float(np.mean(intensity != 0.0)) if len(intensity) else None,
    }

    uv_prior, cam_prior, source_indices = project_points(
        xyz, R_LIDAR_TO_CAMERA_PRIOR, np.zeros(3), k, dist, 0.2, min(args.max_depth, 30.0)
    )
    if len(uv_prior) == 0:
        return {"status": "failed", "reason": "rough_projection_empty"}

    x0, y0 = corners.reshape(-1, 2).min(axis=0)
    x1, y1 = corners.reshape(-1, 2).max(axis=0)
    keep = (
        (uv_prior[:, 0] >= x0 - args.roi_margin)
        & (uv_prior[:, 0] <= x1 + args.roi_margin)
        & (uv_prior[:, 1] >= y0 - args.roi_margin)
        & (uv_prior[:, 1] <= y1 + args.roi_margin)
    )
    roi_indices = source_indices[keep]
    roi_points = xyz[roi_indices]
    target_normal_lidar = R_LIDAR_TO_CAMERA_PRIOR.T @ camera_board["normal"]
    plane = fit_plane(roi_points, target_normal_lidar, args.plane_threshold)
    if plane is None:
        return {
            "status": "failed",
            "reason": "lidar_board_plane_not_found",
            "checkerboard_reprojection_rmse_px": camera_board["reprojection_rmse_px"],
            "roi_points": int(len(roi_points)),
            "pcd_intensity": pcd_intensity_stats,
        }

    normal_lidar = plane["normal"]
    if (R_LIDAR_TO_CAMERA_PRIOR @ normal_lidar) @ camera_board["normal"] < 0:
        normal_lidar = -normal_lidar

    delta_r = align_vectors(R_LIDAR_TO_CAMERA_PRIOR @ normal_lidar, camera_board["normal"])
    r_lidar_to_camera = delta_r @ R_LIDAR_TO_CAMERA_PRIOR
    t_lidar_to_camera = camera_board["center"] - r_lidar_to_camera @ plane["center"]
    rough_rotation_deg = angle_between_rotations(
        r_lidar_to_camera, R_LIDAR_TO_CAMERA_PRIOR
    )
    translation_norm = float(np.linalg.norm(t_lidar_to_camera))
    metrics = {
        "checkerboard_reprojection_rmse_px": camera_board["reprojection_rmse_px"],
        "rough_to_estimated_rotation_deg": rough_rotation_deg,
        "rough_to_estimated_translation_m": translation_norm,
        "lidar_plane_inliers": int(plane["inliers"]),
        "lidar_plane_roi_points": int(plane["total"]),
        "lidar_plane_inlier_ratio": float(plane["inliers"] / max(1, plane["total"])),
        "lidar_plane_normal_angle_to_prior_deg": plane["normal_angle_to_prior_deg"],
    }
    if (
        rough_rotation_deg > args.max_rough_rotation_deg
        or translation_norm > args.max_translation_norm
    ):
        return {
            "status": "failed",
            "reason": "quality_gate_failed",
            "r_lidar_to_camera": [float(x) for x in r_lidar_to_camera.reshape(-1)],
            "t_lidar_to_camera": [float(x) for x in t_lidar_to_camera],
            "metrics": metrics,
            "pcd_intensity": pcd_intensity_stats,
        }

    r_sensor, t_sensor = direct_to_config_rotation_translation(
        r_lidar_to_camera, t_lidar_to_camera
    )

    out_config = json.loads(json.dumps(base_config))
    out_config["rotation"] = [float(x) for x in r_sensor.reshape(-1)]
    out_config["translation"] = [float(x) for x in t_sensor.reshape(-1)]
    out_config["_calibration_method"] = "per_capture_checkerboard_plane_center"
    out_config["_calibration_warning"] = (
        "A single planar checkerboard constrains the board plane and center. "
        "The in-plane rotation is kept closest to the x-forward/y-left/z-up "
        "LiDAR prior because LiDAR checkerboard corners were not directly detected."
    )
    out_config["_source_capture"] = group.name
    out_config["_source_image"] = image_path.name
    out_config["_source_pcd"] = pcd_path.name
    out_config["_pattern_cols"] = int(args.pattern_cols)
    out_config["_pattern_rows"] = int(args.pattern_rows)
    out_config["_square_size_m"] = float(args.square_size)
    out_config["_r_lidar_to_camera"] = [float(x) for x in r_lidar_to_camera.reshape(-1)]
    out_config["_t_lidar_to_camera"] = [float(x) for x in t_lidar_to_camera]
    out_config["_metrics"] = metrics
    out_config["_pcd_intensity"] = pcd_intensity_stats

    calibration_path = group / "lidar_camera_calibration_left.json"
    with open(calibration_path, "w") as f:
        json.dump(out_config, f, indent=4)

    overlay, depth_map, projection_stats = render_intensity_projection(
        image,
        xyz,
        intensity,
        r_lidar_to_camera,
        t_lidar_to_camera,
        k,
        dist,
        args.max_depth,
        args.point_size,
        args.alpha,
    )
    overlay_path = group / "lidar_intensity_overlay.png"
    cv2.imwrite(str(overlay_path), overlay)
    if args.write_depth_png:
        depth_path = group / "lidar_projected_depth_mm.png"
        cv2.imwrite(
            str(depth_path),
            np.clip(depth_map * 1000.0, 0, 65535).astype(np.uint16),
        )
    else:
        depth_path = None

    cv2.imwrite(
        str(group / "checkerboard_detection_overlay.png"),
        draw_checkerboard(image, corners, pattern),
    )

    status = {
        "status": "ok",
        "capture": group.name,
        "image": image_path.name,
        "pcd": pcd_path.name,
        "calibration": calibration_path.name,
        "overlay": overlay_path.name,
        "depth_mm": depth_path.name if depth_path else None,
        "r_lidar_to_camera": [float(x) for x in r_lidar_to_camera.reshape(-1)],
        "t_lidar_to_camera": [float(x) for x in t_lidar_to_camera],
        "metrics": out_config["_metrics"],
        "projection": projection_stats,
        "pcd_intensity": pcd_intensity_stats,
    }
    return status


def greedy_pose_groups(successes, rotation_threshold_deg=5.0, translation_threshold_m=0.35):
    groups = []
    for item in successes:
        r = np.array(item["r_lidar_to_camera"], dtype=np.float64).reshape(3, 3)
        t = np.array(item["t_lidar_to_camera"], dtype=np.float64)
        placed = False
        for group in groups:
            gr = np.array(group["representative_r"], dtype=np.float64).reshape(3, 3)
            gt = np.array(group["representative_t"], dtype=np.float64)
            if (
                angle_between_rotations(r, gr) <= rotation_threshold_deg
                and np.linalg.norm(t - gt) <= translation_threshold_m
            ):
                group["captures"].append(item["capture"])
                placed = True
                break
        if not placed:
            groups.append(
                {
                    "representative_capture": item["capture"],
                    "representative_r": item["r_lidar_to_camera"],
                    "representative_t": item["t_lidar_to_camera"],
                    "captures": [item["capture"]],
                }
            )
    return groups


def main():
    args = parse_args()
    dataset_dir = Path(args.dataset_dir)
    with open(args.config) as f:
        base_config = json.load(f)
    k = np.array(base_config["intrinsic"], dtype=np.float64).reshape(3, 3)
    dist = np.array(base_config["distortion"], dtype=np.float64).reshape(-1, 1)

    capture_dirs = sorted(p for p in dataset_dir.glob("capture_*") if p.is_dir())
    summary = []
    for index, group in enumerate(capture_dirs, 1):
        try:
            status = calibrate_group(group, base_config, k, dist, args)
        except Exception as exc:
            status = {"status": "failed", "reason": f"exception: {exc}"}
        status["capture"] = group.name
        summary.append(status)
        with open(group / "lidar_camera_calibration_status.json", "w") as f:
            json.dump(status, f, indent=4)
        reason = status.get("reason", "")
        print(f"[{index:03d}/{len(capture_dirs):03d}] {group.name}: {status['status']} {reason}")

    successes = [item for item in summary if item.get("status") == "ok"]
    summary_path = dataset_dir / "per_capture_calibration_summary.json"
    with open(summary_path, "w") as f:
        json.dump(
            {
                "dataset_dir": str(dataset_dir),
                "config": str(args.config),
                "pattern_cols": int(args.pattern_cols),
                "pattern_rows": int(args.pattern_rows),
                "square_size_m": float(args.square_size),
                "success_count": len(successes),
                "failure_count": len(summary) - len(successes),
                "similar_pose_groups": greedy_pose_groups(successes),
                "captures": summary,
            },
            f,
            indent=4,
        )
    print(f"wrote {summary_path}")


if __name__ == "__main__":
    main()
