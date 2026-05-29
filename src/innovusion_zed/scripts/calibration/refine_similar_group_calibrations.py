#!/usr/bin/env python3

"""Refine calibration by solving one shared extrinsic per similar-pose group."""

import argparse
import json
import math
import shutil
from pathlib import Path

import cv2
import numpy as np

from batch_calibrate_em4_captures import (
    R_LIDAR_TO_CAMERA_PRIOR,
    angle_between_rotations,
    detect_checkerboard,
    direct_to_config_rotation_translation,
    draw_checkerboard,
    fit_plane,
    project_points,
    read_pcd_points,
    render_intensity_projection,
    solve_camera_board_pose,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--groups-root",
        default=(
            "src/innovusion_zed/data/EM4_calibration20260521/grouping/"
            "recognition_success/by_similar_calibration"
        ),
    )
    parser.add_argument(
        "--config",
        default="src/innovusion_zed/config/camera_left.json",
        help="Base camera config supplying intrinsics/distortion.",
    )
    parser.add_argument("--pattern-cols", type=int, default=12)
    parser.add_argument("--pattern-rows", type=int, default=18)
    parser.add_argument("--square-size", type=float, default=0.045)
    parser.add_argument("--roi-margin", type=float, default=120.0)
    parser.add_argument("--plane-threshold", type=float, default=0.025)
    parser.add_argument("--max-depth", type=float, default=80.0)
    parser.add_argument("--point-size", type=int, default=1)
    parser.add_argument("--alpha", type=float, default=0.65)
    parser.add_argument("--min-refit-samples", type=int, default=2)
    parser.add_argument(
        "--reject-outliers",
        action="store_true",
        help="Optionally reject captures with large residuals before final group refit.",
    )
    parser.add_argument("--outlier-center-m", type=float, default=0.35)
    parser.add_argument("--outlier-plane-m", type=float, default=0.16)
    parser.add_argument("--outlier-normal-deg", type=float, default=8.0)
    return parser.parse_args()


def r_to_vec(r):
    return cv2.Rodrigues(r)[0].ravel()


def vec_to_r(vec):
    return cv2.Rodrigues(np.asarray(vec, dtype=np.float64).reshape(3, 1))[0]


def rotation_vector_delta(r, r_prior):
    return cv2.Rodrigues(r @ r_prior.T)[0].ravel()


def load_existing_initial(capture_dir):
    status_path = capture_dir / "lidar_camera_calibration_status.json"
    if status_path.exists():
        status = json.load(open(status_path))
        if status.get("status") == "ok" and "r_lidar_to_camera" in status:
            return (
                np.array(status["r_lidar_to_camera"], dtype=np.float64).reshape(3, 3),
                np.array(status["t_lidar_to_camera"], dtype=np.float64),
                status,
            )

    calib_path = capture_dir / "lidar_camera_calibration_left.json"
    if calib_path.exists():
        cfg = json.load(open(calib_path))
        if "_r_lidar_to_camera" in cfg and "_t_lidar_to_camera" in cfg:
            return (
                np.array(cfg["_r_lidar_to_camera"], dtype=np.float64).reshape(3, 3),
                np.array(cfg["_t_lidar_to_camera"], dtype=np.float64),
                {},
            )
    return R_LIDAR_TO_CAMERA_PRIOR.copy(), np.zeros(3, dtype=np.float64), {}


def find_single_file(capture_dir, pattern):
    matches = sorted(capture_dir.glob(pattern))
    return matches[0] if len(matches) == 1 else None


def clean_capture_outputs(capture_dir):
    for name in (
        "lidar_camera_calibration_left.json",
        "lidar_intensity_overlay.png",
        "lidar_projected_depth_mm.png",
        "checkerboard_detection_overlay.png",
        "checkerboard_detection_failed.png",
    ):
        path = capture_dir / name
        if path.exists():
            path.unlink()


def extract_pair(capture_dir, base_config, k, dist, args):
    image_path = find_single_file(capture_dir, "*_left.png")
    pcd_path = find_single_file(capture_dir, "*.pcd")
    if image_path is None or pcd_path is None:
        return None, {"status": "failed", "reason": "missing_unique_left_image_or_pcd"}

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        return None, {"status": "failed", "reason": "failed_to_read_left_image"}

    pattern = (args.pattern_cols, args.pattern_rows)
    ok, corners = detect_checkerboard(image, pattern)
    if not ok:
        return None, {"status": "failed", "reason": "checkerboard_not_detected"}

    camera_board = solve_camera_board_pose(corners, k, dist, pattern, args.square_size)
    if camera_board is None:
        return None, {"status": "failed", "reason": "camera_solvepnp_failed"}

    xyz, intensity, fields = read_pcd_points(pcd_path)
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    intensity = intensity[finite]

    r_init, t_init, old_status = load_existing_initial(capture_dir)
    uv, _, source_indices = project_points(
        xyz, r_init, t_init, k, dist, 0.2, min(args.max_depth, 30.0)
    )
    if len(uv) == 0:
        return None, {"status": "failed", "reason": "initial_projection_empty"}

    x0, y0 = corners.reshape(-1, 2).min(axis=0)
    x1, y1 = corners.reshape(-1, 2).max(axis=0)
    keep = (
        (uv[:, 0] >= x0 - args.roi_margin)
        & (uv[:, 0] <= x1 + args.roi_margin)
        & (uv[:, 1] >= y0 - args.roi_margin)
        & (uv[:, 1] <= y1 + args.roi_margin)
    )
    roi_indices = source_indices[keep]
    roi_points = xyz[roi_indices]
    target_normal_lidar = r_init.T @ camera_board["normal"]
    plane = fit_plane(roi_points, target_normal_lidar, args.plane_threshold)
    if plane is None:
        return None, {
            "status": "failed",
            "reason": "lidar_board_plane_not_found",
            "roi_points": int(len(roi_points)),
        }

    normal_lidar = plane["normal"]
    if (r_init @ normal_lidar) @ camera_board["normal"] < 0:
        normal_lidar = -normal_lidar
        plane["d"] = -plane["d"]

    camera_d = float(-camera_board["normal"] @ camera_board["t_board"])
    pair = {
        "capture": capture_dir.name,
        "capture_dir": str(capture_dir),
        "image_path": str(image_path),
        "pcd_path": str(pcd_path),
        "image": image,
        "corners": corners,
        "xyz": xyz,
        "intensity": intensity,
        "pcd_fields": list(fields),
        "nc": camera_board["normal"],
        "dc": camera_d,
        "center_cam": camera_board["center"],
        "nl": normal_lidar,
        "dl": float(plane["d"]),
        "center_lidar": plane["center"],
        "camera_reprojection_rmse_px": camera_board["reprojection_rmse_px"],
        "lidar_plane_inliers": int(plane["inliers"]),
        "lidar_plane_roi_points": int(plane["total"]),
        "lidar_plane_inlier_ratio": float(plane["inliers"] / max(1, plane["total"])),
        "r_init": r_init,
        "t_init": t_init,
        "old_status": old_status,
    }
    return pair, {"status": "ok"}


def average_initial_pose(pairs):
    if not pairs:
        return R_LIDAR_TO_CAMERA_PRIOR.copy(), np.zeros(3)
    # Use the first successful per-capture rotation as a stable Rodrigues chart.
    r0 = pairs[0]["r_init"]
    rel_vecs = [rotation_vector_delta(pair["r_init"], r0) for pair in pairs]
    r_prior = vec_to_r(np.mean(rel_vecs, axis=0)) @ r0
    t_prior = np.mean([pair["t_init"] for pair in pairs], axis=0)
    return r_prior, t_prior


def residual_vector(params, pairs, r_prior, t_prior):
    r = vec_to_r(params[:3])
    t = params[3:6]
    residuals = []
    for pair in pairs:
        pred_n = r @ pair["nl"]
        pred_d = pair["dl"] - pred_n @ t
        if pred_n @ pair["nc"] < 0:
            pred_n = -pred_n
            pred_d = -pred_d
        center_pred = r @ pair["center_lidar"] + t
        residuals.extend((3.0 * (pred_n - pair["nc"])).tolist())
        residuals.extend(((center_pred - pair["center_cam"]) / 0.16).tolist())
        residuals.append((pred_d - pair["dc"]) / 0.06)

    residuals.extend((rotation_vector_delta(r, r_prior) / math.radians(8.0)).tolist())
    residuals.extend(((t - t_prior) / 0.45).tolist())
    return np.asarray(residuals, dtype=np.float64)


def optimize_pose(pairs, r_initial, t_initial):
    params = np.r_[r_to_vec(r_initial), t_initial].astype(np.float64)
    damping = 1e-3
    for _ in range(80):
        residual = residual_vector(params, pairs, r_initial, t_initial)
        jacobian = np.zeros((len(residual), 6), dtype=np.float64)
        eps = 1e-6
        for i in range(6):
            plus = params.copy()
            minus = params.copy()
            plus[i] += eps
            minus[i] -= eps
            jacobian[:, i] = (
                residual_vector(plus, pairs, r_initial, t_initial)
                - residual_vector(minus, pairs, r_initial, t_initial)
            ) / (2.0 * eps)
        lhs = jacobian.T @ jacobian + damping * np.eye(6)
        rhs = -jacobian.T @ residual
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            break
        if np.linalg.norm(step) < 1e-10:
            break
        candidate = params + step
        if np.mean(residual_vector(candidate, pairs, r_initial, t_initial) ** 2) < np.mean(
            residual**2
        ):
            params = candidate
            damping = max(damping / 3.0, 1e-9)
        else:
            damping *= 10.0
    return vec_to_r(params[:3]), params[3:6]


def pair_metrics(pair, r, t):
    pred_n = r @ pair["nl"]
    pred_d = pair["dl"] - pred_n @ t
    if pred_n @ pair["nc"] < 0:
        pred_n = -pred_n
        pred_d = -pred_d
    normal_deg = math.degrees(math.acos(float(np.clip(pred_n @ pair["nc"], -1.0, 1.0))))
    center_error = float(np.linalg.norm(r @ pair["center_lidar"] + t - pair["center_cam"]))
    plane_error = float(pred_d - pair["dc"])
    return {
        "capture": pair["capture"],
        "normal_error_deg": normal_deg,
        "center_error_m": center_error,
        "plane_error_m": plane_error,
        "abs_plane_error_m": abs(plane_error),
        "checkerboard_reprojection_rmse_px": pair["camera_reprojection_rmse_px"],
        "lidar_plane_inlier_ratio": pair["lidar_plane_inlier_ratio"],
    }


def reject_outliers(pairs, r, t, args):
    kept = []
    rejected = []
    for pair in pairs:
        metrics = pair_metrics(pair, r, t)
        is_outlier = (
            metrics["center_error_m"] > args.outlier_center_m
            or metrics["abs_plane_error_m"] > args.outlier_plane_m
            or metrics["normal_error_deg"] > args.outlier_normal_deg
        )
        if is_outlier and len(pairs) - len(rejected) > args.min_refit_samples:
            rejected.append({**metrics, "reason": "joint_fit_outlier"})
        else:
            kept.append(pair)
    return kept, rejected


def summarize_metrics(metrics):
    if not metrics:
        return {}
    return {
        "normal_error_mean_deg": float(np.mean([m["normal_error_deg"] for m in metrics])),
        "normal_error_max_deg": float(np.max([m["normal_error_deg"] for m in metrics])),
        "center_error_mean_m": float(np.mean([m["center_error_m"] for m in metrics])),
        "center_error_max_m": float(np.max([m["center_error_m"] for m in metrics])),
        "abs_plane_error_mean_m": float(np.mean([m["abs_plane_error_m"] for m in metrics])),
        "abs_plane_error_max_m": float(np.max([m["abs_plane_error_m"] for m in metrics])),
        "checkerboard_reprojection_rmse_mean_px": float(
            np.mean([m["checkerboard_reprojection_rmse_px"] for m in metrics])
        ),
    }


def make_output_config(base_config, group_name, pairs, r, t, metrics, rejected):
    r_sensor, t_sensor = direct_to_config_rotation_translation(r, t)
    out = json.loads(json.dumps(base_config))
    out["rotation"] = [float(x) for x in r_sensor.reshape(-1)]
    out["translation"] = [float(x) for x in t_sensor.reshape(-1)]
    out["_calibration_method"] = "similar_group_joint_checkerboard_plane"
    out["_calibration_group"] = group_name
    out["_group_used_captures"] = [pair["capture"] for pair in pairs]
    out["_group_rejected_captures"] = rejected
    out["_r_lidar_to_camera"] = [float(x) for x in r.reshape(-1)]
    out["_t_lidar_to_camera"] = [float(x) for x in t]
    out["_metrics"] = summarize_metrics(metrics)
    out["_per_capture_metrics"] = metrics
    out["_calibration_warning"] = (
        "This shared extrinsic was refined from multiple captures in the same "
        "similar-pose group. Constraints come from camera checkerboard poses and "
        "LiDAR board planes."
    )
    return out


def write_capture_outputs(capture_dir, base_config, out_config, pair, r, t, k, dist, args):
    clean_capture_outputs(capture_dir)
    calib_path = capture_dir / "lidar_camera_calibration_left.json"
    with open(calib_path, "w") as f:
        json.dump(out_config, f, indent=4)

    overlay, depth_map, projection_stats = render_intensity_projection(
        pair["image"],
        pair["xyz"],
        pair["intensity"],
        r,
        t,
        k,
        dist,
        args.max_depth,
        args.point_size,
        args.alpha,
    )
    cv2.imwrite(str(capture_dir / "lidar_intensity_overlay.png"), overlay)
    cv2.imwrite(
        str(capture_dir / "lidar_projected_depth_mm.png"),
        np.clip(depth_map * 1000.0, 0, 65535).astype(np.uint16),
    )
    cv2.imwrite(
        str(capture_dir / "checkerboard_detection_overlay.png"),
        draw_checkerboard(pair["image"], pair["corners"], (args.pattern_cols, args.pattern_rows)),
    )
    status = {
        "status": "ok",
        "calibration_scope": "similar_group_joint",
        "group": out_config["_calibration_group"],
        "group_used_captures": out_config["_group_used_captures"],
        "group_rejected_captures": out_config["_group_rejected_captures"],
        "calibration": calib_path.name,
        "overlay": "lidar_intensity_overlay.png",
        "depth_mm": "lidar_projected_depth_mm.png",
        "r_lidar_to_camera": out_config["_r_lidar_to_camera"],
        "t_lidar_to_camera": out_config["_t_lidar_to_camera"],
        "metrics": out_config["_metrics"],
        "this_capture_metrics": pair_metrics(pair, r, t),
        "projection": projection_stats,
    }
    with open(capture_dir / "lidar_camera_calibration_status.json", "w") as f:
        json.dump(status, f, indent=4)


def process_group(group_dir, base_config, k, dist, args):
    capture_dirs = sorted(p for p in group_dir.iterdir() if p.is_dir())
    extracted = []
    extraction_failures = []
    for capture_dir in capture_dirs:
        pair, status = extract_pair(capture_dir, base_config, k, dist, args)
        if pair is None:
            extraction_failures.append({"capture": capture_dir.name, **status})
        else:
            extracted.append(pair)

    if not extracted:
        return {
            "group": group_dir.name,
            "status": "failed",
            "reason": "no_extractable_pairs",
            "extraction_failures": extraction_failures,
        }

    r_prior, t_prior = average_initial_pose(extracted)
    r_joint, t_joint = optimize_pose(extracted, r_prior, t_prior)
    if args.reject_outliers:
        kept, rejected = reject_outliers(extracted, r_joint, t_joint, args)
        if len(kept) >= args.min_refit_samples and len(kept) < len(extracted):
            r_prior, t_prior = average_initial_pose(kept)
            r_joint, t_joint = optimize_pose(kept, r_prior, t_prior)
        else:
            kept = extracted
            rejected = []
    else:
        kept = extracted
        rejected = []

    metrics = [pair_metrics(pair, r_joint, t_joint) for pair in kept]
    out_config = make_output_config(
        base_config, group_dir.name, kept, r_joint, t_joint, metrics, rejected
    )
    group_config_path = group_dir / "group_lidar_camera_calibration_left.json"
    with open(group_config_path, "w") as f:
        json.dump(out_config, f, indent=4)

    for pair in kept:
        write_capture_outputs(Path(pair["capture_dir"]), base_config, out_config, pair, r_joint, t_joint, k, dist, args)

    for item in rejected:
        capture_dir = group_dir / item["capture"]
        clean_capture_outputs(capture_dir)
        with open(capture_dir / "lidar_camera_calibration_status.json", "w") as f:
            json.dump(
                {
                    "status": "failed",
                    "calibration_scope": "similar_group_joint",
                    "group": group_dir.name,
                    "reason": item["reason"],
                    "metrics": item,
                },
                f,
                indent=4,
            )

    summary = {
        "group": group_dir.name,
        "status": "ok",
        "capture_count": len(capture_dirs),
        "extracted_count": len(extracted),
        "used_count": len(kept),
        "rejected_count": len(rejected),
        "extraction_failures": extraction_failures,
        "rejected": rejected,
        "used_captures": [pair["capture"] for pair in kept],
        "calibration": str(group_config_path),
        "r_lidar_to_camera": [float(x) for x in r_joint.reshape(-1)],
        "t_lidar_to_camera": [float(x) for x in t_joint],
        "metrics": summarize_metrics(metrics),
        "per_capture_metrics": metrics,
    }
    with open(group_dir / "group_refined_calibration_summary.json", "w") as f:
        json.dump(summary, f, indent=4)
    return summary


def main():
    args = parse_args()
    groups_root = Path(args.groups_root)
    with open(args.config) as f:
        base_config = json.load(f)
    k = np.array(base_config["intrinsic"], dtype=np.float64).reshape(3, 3)
    dist = np.array(base_config["distortion"], dtype=np.float64).reshape(-1, 1)

    group_dirs = sorted(p for p in groups_root.glob("group_*") if p.is_dir())
    summaries = []
    for index, group_dir in enumerate(group_dirs, 1):
        summary = process_group(group_dir, base_config, k, dist, args)
        summaries.append(summary)
        print(
            f"[{index:02d}/{len(group_dirs):02d}] {group_dir.name}: "
            f"{summary['status']} used={summary.get('used_count', 0)} "
            f"rejected={summary.get('rejected_count', 0)}"
        )

    out_path = groups_root / "joint_group_calibration_summary.json"
    with open(out_path, "w") as f:
        json.dump(
            {
                "groups_root": str(groups_root),
                "config": args.config,
                "pattern_cols": args.pattern_cols,
                "pattern_rows": args.pattern_rows,
                "square_size_m": args.square_size,
                "groups": summaries,
            },
            f,
            indent=4,
        )
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
