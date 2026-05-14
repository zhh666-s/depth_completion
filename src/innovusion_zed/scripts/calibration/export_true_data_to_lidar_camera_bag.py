#!/usr/bin/env python3

import argparse
import json
import re
import struct
from pathlib import Path

import cv2
import numpy as np
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export innovusion_zed true_data folders to a rosbag compatible with heethesh/lidar_camera_calibration."
    )
    parser.add_argument("--data-dir", default="src/innovusion_zed/true_data_2")
    parser.add_argument("--config", default="src/innovusion_zed/config/camera_left.json")
    parser.add_argument("--output", default="src/lidar_camera_calibration/bagfiles/true_data_2_calib.bag")
    parser.add_argument("--camera-frame", default="camera")
    parser.add_argument("--lidar-frame", default="velodyne")
    parser.add_argument("--image-topic", default="/sensors/camera/image_color")
    parser.add_argument("--camera-info-topic", default="/sensors/camera/camera_info")
    parser.add_argument("--points-topic", default="/sensors/velodyne_points")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--use-original-stamps",
        action="store_true",
        help="Use timestamps parsed from sample filenames. By default samples are packed at --rate for interactive calibration.",
    )
    return parser.parse_args()


def read_pcd_xyzi(path):
    with open(path, "rb") as handle:
        header_lines = []
        while True:
            line = handle.readline()
            if not line:
                raise RuntimeError("PCD file has no DATA line: %s" % path)
            text = line.decode("ascii", errors="ignore").strip()
            header_lines.append(text)
            if text.startswith("DATA"):
                data_type = text.split()[1]
                break

        meta = {}
        for line in header_lines:
            parts = line.split()
            if parts:
                meta[parts[0]] = parts[1:]

        fields = meta["FIELDS"]
        sizes = list(map(int, meta["SIZE"]))
        types = meta["TYPE"]
        counts = list(map(int, meta.get("COUNT", ["1"] * len(fields))))
        points_count = int(meta.get("POINTS", meta.get("WIDTH"))[0])

        if data_type == "ascii":
            arr = np.loadtxt(handle, dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            columns = {name: i for i, name in enumerate(fields)}
            intensity = arr[:, columns["intensity"]] if "intensity" in columns else np.zeros(arr.shape[0])
            return np.column_stack(
                [arr[:, columns["x"]], arr[:, columns["y"]], arr[:, columns["z"]], intensity]
            ).astype(np.float32)

        if data_type != "binary":
            raise RuntimeError("Unsupported PCD DATA type %s in %s" % (data_type, path))

        dtype_fields = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            if typ == "F" and size == 4:
                dtype = "<f4"
            elif typ == "F" and size == 8:
                dtype = "<f8"
            elif typ == "U" and size == 1:
                dtype = "u1"
            elif typ == "U" and size == 2:
                dtype = "<u2"
            elif typ == "U" and size == 4:
                dtype = "<u4"
            elif typ == "I" and size == 1:
                dtype = "i1"
            elif typ == "I" and size == 2:
                dtype = "<i2"
            elif typ == "I" and size == 4:
                dtype = "<i4"
            else:
                raise RuntimeError("Unsupported PCD field %s %s %s" % (name, size, typ))
            dtype_fields.append((name, dtype) if count == 1 else (name, dtype, (count,)))

        dtype = np.dtype(dtype_fields)
        raw = handle.read(points_count * dtype.itemsize)
        arr = np.frombuffer(raw, dtype=dtype, count=points_count)
        intensity = arr["intensity"] if "intensity" in arr.dtype.names else np.zeros(points_count)
        return np.column_stack([arr["x"], arr["y"], arr["z"], intensity]).astype(np.float32)


def camera_info_from_config(config, stamp, frame_id):
    width, height = config["image_size"]
    k = np.array(config["intrinsic"], dtype=float).reshape(3, 3)
    d = list(map(float, config["distortion"]))

    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.D = d
    msg.K = [float(x) for x in k.reshape(-1)]
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = [
        float(k[0, 0]), 0.0, float(k[0, 2]), 0.0,
        0.0, float(k[1, 1]), float(k[1, 2]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return msg


def make_cloud(points, stamp, frame_id):
    header = Header(stamp=stamp, frame_id=frame_id)
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]
    clean = points[np.isfinite(points).all(axis=1)]
    return point_cloud2.create_cloud(header, fields, clean.tolist())


def sample_stamp(stem, index, rate, use_original_stamps):
    match = re.match(r"sample_\d+_([0-9.]+)$", stem)
    if use_original_stamps and match:
        return rospy.Time.from_sec(float(match.group(1)))
    return rospy.Time.from_sec(index / max(rate, 1e-6))


def main():
    args = parse_args()
    data_dir = Path(args.data_dir)
    left_dir = data_dir / "left"
    pcd_dir = data_dir / "pcd"
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    config = json.load(open(args.config))
    bridge = CvBridge()
    stems = sorted(set(p.stem for p in left_dir.glob("*.png")) & set(p.stem for p in pcd_dir.glob("*.pcd")))
    if not stems:
        raise RuntimeError("No matching left/*.png and pcd/*.pcd samples in %s" % data_dir)

    with rosbag.Bag(str(output), "w") as bag:
        for index, stem in enumerate(stems):
            stamp = sample_stamp(stem, index, args.rate, args.use_original_stamps)

            image = cv2.imread(str(left_dir / (stem + ".png")), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("Failed to read image %s" % stem)
            image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = args.camera_frame

            info_msg = camera_info_from_config(config, stamp, args.camera_frame)
            points = read_pcd_xyzi(pcd_dir / (stem + ".pcd"))
            cloud_msg = make_cloud(points, stamp, args.lidar_frame)

            bag.write(args.image_topic, image_msg, stamp)
            bag.write(args.camera_info_topic, info_msg, stamp)
            bag.write(args.points_topic, cloud_msg, stamp)
            print("wrote", stem, "points", len(points))

    print("Wrote bag:", output)
    print("Topics:")
    print(" ", args.image_topic)
    print(" ", args.camera_info_topic)
    print(" ", args.points_topic)


if __name__ == "__main__":
    main()
