#!/usr/bin/env python3

"""Project LiDAR PCD frames into camera images for calibration visual checks."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


AX = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=np.float64)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--data-dir", required=True, nargs="+",
                   help="One or more true_data directories containing left/ and pcd/.")
    p.add_argument("--config", required=True,
                   help="Camera calibration JSON, for example camera_left_auto_calibrated_true_data_2_3.json.")
    default_out = Path(__file__).resolve().parents[2] / "data" / "remap"
    p.add_argument("--out-dir", default=str(default_out),
                   help=f"Output directory. Defaults to {default_out}.")
    p.add_argument("--max-samples", type=int, default=0,
                   help="Limit rendered frames after sorting. 0 means render all frames.")
    p.add_argument("--alpha", type=float, default=0.55,
                   help="Overlay opacity for projected depth colors.")
    p.add_argument("--point-size", type=int, default=2,
                   help="Projected point radius in pixels. 0 writes only exact projected pixels.")
    p.add_argument("--min-depth", type=float, default=0.2)
    p.add_argument("--max-depth", type=float, default=80.0)
    p.add_argument("--color-mode", choices=["auto", "fixed"], default="auto",
                   help="auto uses per-frame depth percentiles for high-contrast colors; fixed uses min/max-depth.")
    p.add_argument("--depth-percentile-low", type=float, default=2.0)
    p.add_argument("--depth-percentile-high", type=float, default=98.0)
    p.add_argument("--no-colorbar", action="store_true",
                   help="Do not draw the depth colorbar on overlay images.")
    p.add_argument("--use-undistort", action="store_true",
                   help="Undistort the image first and project with undistort_intrinsic.")
    p.add_argument("--write-depth-png", action="store_true",
                   help="Also save a 16-bit depth image in millimeters.")
    p.add_argument("--board-view", action="store_true",
                   help="Also render checkerboard-focused overlays for drift inspection.")
    p.add_argument("--board-color", choices=["auto", "intensity", "plane_error", "depth"], default="auto",
                   help="Color board points by intensity when available, otherwise signed plane error.")
    p.add_argument("--pattern-cols", type=int, default=9)
    p.add_argument("--pattern-rows", type=int, default=6)
    p.add_argument("--square-size", type=float, default=0.13)
    p.add_argument("--board-margin", type=float, default=80.0,
                   help="Pixel margin around detected checkerboard inner corners.")
    p.add_argument("--board-plane-band", type=float, default=0.12,
                   help="Keep projected LiDAR points within this signed distance from the camera checkerboard plane.")
    return p.parse_args()


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
            cols = {name: i for i, name in enumerate(fields)}
            xyz = arr[:, [cols["x"], cols["y"], cols["z"]]].astype(np.float64)
            intensity = arr[:, cols["intensity"]].astype(np.float32) if "intensity" in cols else np.zeros(len(xyz), dtype=np.float32)
            return xyz, intensity

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
    intensity = arr["intensity"].astype(np.float32) if "intensity" in arr.dtype.names else np.zeros(len(xyz), dtype=np.float32)
    return xyz, intensity


def load_calibration(path, use_undistort):
    with open(path) as f:
        cfg = json.load(f)

    k = np.array(cfg["intrinsic"], dtype=np.float64).reshape(3, 3)
    dist = np.array(cfg["distortion"], dtype=np.float64).reshape(-1, 1)
    if use_undistort:
        k = np.array(cfg.get("undistort_intrinsic", cfg["intrinsic"]), dtype=np.float64).reshape(3, 3)
        dist = np.array(cfg.get("undistort_distortion", [0, 0, 0, 0, 0]), dtype=np.float64).reshape(-1, 1)

    r_sensor = np.array(cfg["rotation"], dtype=np.float64).reshape(3, 3)
    t_sensor = np.array(cfg["translation"], dtype=np.float64)
    r_lidar_to_cam = np.linalg.inv(r_sensor) @ AX
    t_lidar_to_cam = -np.linalg.inv(r_sensor) @ t_sensor
    image_size = tuple(int(x) for x in cfg.get("image_size", [1920, 1080]))
    return cfg, k, dist, r_lidar_to_cam, t_lidar_to_cam, image_size


def collect_samples(data_dirs, max_samples):
    samples = []
    for data_dir in data_dirs:
        left_dir = data_dir / "left"
        pcd_dir = data_dir / "pcd"
        for img_path in sorted(left_dir.glob("*.png")):
            pcd_path = pcd_dir / f"{img_path.stem}.pcd"
            if pcd_path.exists():
                samples.append((data_dir.name, img_path, pcd_path))
    samples.sort(key=lambda item: (item[0], item[1].name))
    if max_samples > 0:
        samples = samples[:max_samples]
    return samples


def project_depth(xyz, r_lidar_to_cam, t_lidar_to_cam, k, dist, width, height, min_depth, max_depth):
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    cam = (r_lidar_to_cam @ xyz.T + t_lidar_to_cam[:, None]).T
    valid_z = (cam[:, 2] >= min_depth) & (cam[:, 2] <= max_depth)
    cam = cam[valid_z]
    if len(cam) == 0:
        return np.zeros((height, width), dtype=np.float32), np.empty((0, 2), dtype=np.int32), np.empty(0, dtype=np.float32)

    uv = cv2.projectPoints(
        cam.reshape(-1, 1, 3),
        np.zeros(3),
        np.zeros(3),
        k,
        dist,
    )[0].reshape(-1, 2)
    px = np.rint(uv[:, 0]).astype(np.int32)
    py = np.rint(uv[:, 1]).astype(np.int32)
    inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    px = px[inside]
    py = py[inside]
    depth = cam[inside, 2].astype(np.float32)
    if len(depth) == 0:
        return np.zeros((height, width), dtype=np.float32), np.empty((0, 2), dtype=np.int32), np.empty(0, dtype=np.float32)

    flat = py * width + px
    order = np.lexsort((depth, flat))
    flat = flat[order]
    depth = depth[order]
    keep = np.r_[True, flat[1:] != flat[:-1]]
    flat = flat[keep]
    depth = depth[keep]

    depth_map = np.zeros(height * width, dtype=np.float32)
    depth_map[flat] = depth
    depth_map = depth_map.reshape(height, width)
    pixels = np.vstack((flat % width, flat // width)).T.astype(np.int32)
    return depth_map, pixels, depth


def depth_color_range(depth_map, min_depth, max_depth, color_mode, percentile_low, percentile_high):
    mask = depth_map > 0
    if not mask.any():
        return min_depth, max_depth
    if color_mode == "fixed":
        return min_depth, max_depth

    values = depth_map[mask]
    lo = float(np.percentile(values, percentile_low))
    hi = float(np.percentile(values, percentile_high))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo = float(values.min())
        hi = float(values.max())
    if hi <= lo:
        hi = lo + 1.0
    return lo, hi


def colorize_depth(depth_map, color_min, color_max):
    mask = depth_map > 0
    normalized = np.zeros_like(depth_map, dtype=np.uint8)
    clipped = np.clip(depth_map, color_min, color_max)
    normalized[mask] = (255.0 * (1.0 - (clipped[mask] - color_min) / (color_max - color_min))).astype(np.uint8)
    color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    color[~mask] = 0
    return color, mask


def overlay_depth(image, depth_color, pixels, alpha, point_size):
    overlay = image.copy()
    if point_size <= 0:
        mask = np.any(depth_color > 0, axis=2)
        overlay[mask] = cv2.addWeighted(image[mask], 1.0 - alpha, depth_color[mask], alpha, 0)
        return overlay

    for x, y in pixels:
        color = tuple(int(v) for v in depth_color[y, x])
        cv2.circle(overlay, (int(x), int(y)), point_size, color, thickness=-1, lineType=cv2.LINE_AA)
    return cv2.addWeighted(image, 1.0 - alpha, overlay, alpha, 0)


def draw_colorbar(image, color_min, color_max):
    out = image.copy()
    h, w = out.shape[:2]
    bar_h = max(220, h // 5)
    bar_w = 26
    x0 = w - bar_w - 32
    y0 = 36
    gradient = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(bar_h, 1)
    bar = cv2.applyColorMap(np.repeat(gradient, bar_w, axis=1), cv2.COLORMAP_TURBO)
    out[y0:y0 + bar_h, x0:x0 + bar_w] = bar
    cv2.rectangle(out, (x0 - 1, y0 - 1), (x0 + bar_w, y0 + bar_h), (255, 255, 255), 1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(out, f"{color_min:.2f}m", (x0 - 8, y0 + bar_h + 24), font, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(out, f"{color_max:.2f}m", (x0 - 8, y0 - 10), font, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def detect_checkerboard(image, pattern):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    else:
        ok, corners = False, None
    if not ok:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
        if ok:
            cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
    return ok, corners


def checkerboard_object_points(pattern, square_size):
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


def draw_checkerboard_grid(image, corners, pattern):
    out = image.copy()
    pts = corners.reshape(pattern[1], pattern[0], 2).astype(np.int32)
    for row in range(pattern[1]):
        cv2.polylines(out, [np.ascontiguousarray(pts[row])], False, (0, 255, 0), 2, cv2.LINE_AA)
    for col in range(pattern[0]):
        cv2.polylines(out, [np.ascontiguousarray(pts[:, col])], False, (0, 255, 0), 2, cv2.LINE_AA)
    for p in pts.reshape(-1, 2):
        cv2.circle(out, tuple(p), 4, (0, 255, 255), -1, cv2.LINE_AA)
    return out


def colorize_values(values, mode, plane_band):
    if len(values) == 0:
        return np.empty((0, 3), dtype=np.uint8), 0.0, 1.0
    if mode == "plane_error":
        lo, hi = -plane_band, plane_band
        norm = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
    else:
        lo = float(np.percentile(values, 2.0))
        hi = float(np.percentile(values, 98.0))
        if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
            lo = float(values.min())
            hi = float(values.max())
        if hi <= lo:
            hi = lo + 1.0
        norm = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
    colors = cv2.applyColorMap((norm * 255.0).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
    return colors, lo, hi


def render_board_overlay(image, xyz, intensity, r_lidar_to_cam, t_lidar_to_cam, k, dist, args):
    pattern = (args.pattern_cols, args.pattern_rows)
    ok, corners = detect_checkerboard(image, pattern)
    if not ok:
        return None, None, {"board_detected": False}

    objp = checkerboard_object_points(pattern, args.square_size)
    ok, rvec, tvec = cv2.solvePnP(objp, corners, k, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None, {"board_detected": False}

    r_board = cv2.Rodrigues(rvec)[0]
    board_normal = r_board[:, 2]
    board_normal /= np.linalg.norm(board_normal)
    board_d = -board_normal @ tvec.ravel()

    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    intensity = intensity[finite]
    cam = (r_lidar_to_cam @ xyz.T + t_lidar_to_cam[:, None]).T
    valid_z = (cam[:, 2] >= args.min_depth) & (cam[:, 2] <= args.max_depth)
    cam = cam[valid_z]
    intensity = intensity[valid_z]
    if len(cam) == 0:
        return draw_checkerboard_grid(image, corners, pattern), None, {"board_detected": True, "board_points": 0}

    uv = cv2.projectPoints(cam.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, dist)[0].reshape(-1, 2)
    x0, y0 = corners.reshape(-1, 2).min(axis=0) - args.board_margin
    x1, y1 = corners.reshape(-1, 2).max(axis=0) + args.board_margin
    plane_error = cam @ board_normal + board_d
    keep = (
        (uv[:, 0] >= x0) & (uv[:, 0] <= x1) &
        (uv[:, 1] >= y0) & (uv[:, 1] <= y1) &
        (np.abs(plane_error) <= args.board_plane_band)
    )
    uv = uv[keep]
    cam = cam[keep]
    intensity = intensity[keep]
    plane_error = plane_error[keep]

    color_mode = args.board_color
    if color_mode == "auto":
        color_mode = "intensity" if len(intensity) and float(np.nanstd(intensity)) > 1e-6 else "plane_error"
    if color_mode == "intensity":
        values = intensity
    elif color_mode == "depth":
        values = cam[:, 2]
    else:
        values = plane_error

    colors, value_min, value_max = colorize_values(values.astype(np.float32), color_mode, args.board_plane_band)
    overlay = draw_checkerboard_grid(image, corners, pattern)
    for (u, v), color in zip(uv, colors):
        cv2.circle(overlay, (int(round(u)), int(round(v))), args.point_size, tuple(int(c) for c in color), -1, cv2.LINE_AA)

    blended = cv2.addWeighted(image, 1.0 - args.alpha, overlay, args.alpha, 0)
    blended = draw_checkerboard_grid(blended, corners, pattern)

    x0c = max(0, int(np.floor(x0)))
    y0c = max(0, int(np.floor(y0)))
    x1c = min(image.shape[1], int(np.ceil(x1)))
    y1c = min(image.shape[0], int(np.ceil(y1)))
    crop = blended[y0c:y1c, x0c:x1c].copy()
    if crop.size:
        crop = cv2.resize(crop, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)

    label = f"{color_mode}: {value_min:.3f}..{value_max:.3f}"
    cv2.putText(blended, label, (24, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
    stats = {
        "board_detected": True,
        "board_points": int(len(uv)),
        "board_color": color_mode,
        "value_min": float(value_min),
        "value_max": float(value_max),
        "mean_abs_plane_error": float(np.mean(np.abs(plane_error))) if len(plane_error) else None,
        "median_plane_error": float(np.median(plane_error)) if len(plane_error) else None,
    }
    return blended, crop if crop.size else None, stats


def main():
    args = parse_args()
    data_dirs = [Path(x) for x in args.data_dir]
    out_dir = Path(args.out_dir)
    overlay_dir = out_dir / "overlay"
    depth_color_dir = out_dir / "depth_color"
    depth_raw_dir = out_dir / "depth_mm"
    board_dir = out_dir / "board_overlay"
    board_crop_dir = out_dir / "board_crop"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    depth_color_dir.mkdir(parents=True, exist_ok=True)
    if args.board_view:
        board_dir.mkdir(parents=True, exist_ok=True)
        board_crop_dir.mkdir(parents=True, exist_ok=True)
    if args.write_depth_png:
        depth_raw_dir.mkdir(parents=True, exist_ok=True)

    cfg, k, dist, r_lidar_to_cam, t_lidar_to_cam, image_size = load_calibration(args.config, args.use_undistort)
    width, height = image_size
    samples = collect_samples(data_dirs, args.max_samples)
    if not samples:
        raise RuntimeError("No matching left image and PCD pairs were found.")

    raw_k = np.array(cfg["intrinsic"], dtype=np.float64).reshape(3, 3)
    raw_dist = np.array(cfg["distortion"], dtype=np.float64).reshape(-1, 1)

    summary = []
    for data_name, img_path, pcd_path in samples:
        image = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        if args.use_undistort:
            image = cv2.undistort(image, raw_k, raw_dist, None, k)

        xyz, intensity = read_pcd_points(pcd_path)
        depth_map, pixels, depths = project_depth(
            xyz, r_lidar_to_cam, t_lidar_to_cam, k, dist,
            width, height, args.min_depth, args.max_depth,
        )
        color_min, color_max = depth_color_range(
            depth_map,
            args.min_depth,
            args.max_depth,
            args.color_mode,
            args.depth_percentile_low,
            args.depth_percentile_high,
        )
        depth_color, _ = colorize_depth(depth_map, color_min, color_max)
        overlay = overlay_depth(image, depth_color, pixels, args.alpha, args.point_size)
        if not args.no_colorbar:
            overlay = draw_colorbar(overlay, color_min, color_max)

        prefix = f"{data_name}_{img_path.stem}"
        cv2.imwrite(str(overlay_dir / f"{prefix}_overlay.png"), overlay)
        cv2.imwrite(str(depth_color_dir / f"{prefix}_depth_color.png"), depth_color)
        if args.write_depth_png:
            cv2.imwrite(str(depth_raw_dir / f"{prefix}_depth_mm.png"), np.clip(depth_map * 1000.0, 0, 65535).astype(np.uint16))
        board_stats = None
        if args.board_view:
            board_overlay, board_crop, board_stats = render_board_overlay(
                image, xyz, intensity, r_lidar_to_cam, t_lidar_to_cam, k, dist, args
            )
            if board_overlay is not None:
                cv2.imwrite(str(board_dir / f"{prefix}_board_overlay.png"), board_overlay)
            if board_crop is not None:
                cv2.imwrite(str(board_crop_dir / f"{prefix}_board_crop.png"), board_crop)
        summary.append({
            "sample": prefix,
            "projected_pixels": int(len(pixels)),
            "min_depth": float(depths.min()) if len(depths) else None,
            "max_depth": float(depths.max()) if len(depths) else None,
            "color_min_depth": color_min,
            "color_max_depth": color_max,
            "board": board_stats,
        })
        board_msg = ""
        if board_stats:
            board_msg = f" board_points={board_stats.get('board_points', 0)} board_color={board_stats.get('board_color')}"
        print(f"wrote {prefix}: projected_pixels={len(pixels)} color_depth=[{color_min:.2f}, {color_max:.2f}]m{board_msg}")

    with open(out_dir / "projection_summary.json", "w") as f:
        json.dump({
            "config": str(args.config),
            "data_dir": [str(x) for x in data_dirs],
            "use_undistort": bool(args.use_undistort),
            "color_mode": args.color_mode,
            "samples": summary,
        }, f, indent=4)


if __name__ == "__main__":
    main()
