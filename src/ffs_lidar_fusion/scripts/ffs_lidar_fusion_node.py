#!/usr/bin/env python3

import copy
import json
import math
from typing import Optional, Tuple

import cv2
import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class FFSLidarFusionNode:
    def __init__(self):
        self.bridge = CvBridge()

        self.left_topic = rospy.get_param("~left_topic", "/camera/left/image_raw")
        self.right_topic = rospy.get_param("~right_topic", "/camera/right/image_raw")
        self.left_info_topic = rospy.get_param("~left_info_topic", "/camera/left/camera_info")
        self.right_info_topic = rospy.get_param("~right_info_topic", "/camera/right/camera_info")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/iv_points")
        self.ffs_disp_topic = rospy.get_param("~ffs_disp_topic", "/ffs/disp")

        self.visual_depth_topic = rospy.get_param(
            "~visual_depth_topic", "/ffs_lidar_fusion/visual_depth"
        )
        self.lidar_depth_topic = rospy.get_param(
            "~lidar_depth_topic", "/ffs_lidar_fusion/lidar_depth"
        )
        self.fused_depth_topic = rospy.get_param(
            "~fused_depth_topic", "/ffs_lidar_fusion/fused_depth"
        )
        self.fused_depth_viz_topic = rospy.get_param(
            "~fused_depth_viz_topic", "/ffs_lidar_fusion/fused_depth_viz"
        )
        self.debug_overlay_topic = rospy.get_param(
            "~debug_overlay_topic", "/ffs_lidar_fusion/debug_overlay"
        )

        self.sync_slop = float(rospy.get_param("~sync_slop", 0.05))
        self.max_lidar_age = float(rospy.get_param("~max_lidar_age", 0.25))
        self.max_input_age = float(rospy.get_param("~max_input_age", 0.45))
        self.lidar_stride = max(1, int(rospy.get_param("~lidar_stride", 2)))
        self.min_disp = float(rospy.get_param("~min_disp", 0.1))
        self.max_depth = float(rospy.get_param("~max_depth", 80.0))
        self.min_depth = float(rospy.get_param("~min_depth", 0.3))
        self.use_lidar_dilation = bool(rospy.get_param("~use_lidar_dilation", True))
        self.lidar_dilation_kernel = int(rospy.get_param("~lidar_dilation_kernel", 3))
        self.publish_debug_overlay = bool(rospy.get_param("~publish_debug_overlay", True))
        self.calib_json = rospy.get_param(
            "~calib_json", "$(find innovusion_zed)/config/camera_left.json"
        )

        self.left_info: Optional[CameraInfo] = None
        self.right_info: Optional[CameraInfo] = None
        self.rotation_l2c = np.eye(3, dtype=np.float32)
        self.translation_l2c = np.zeros((3, 1), dtype=np.float32)
        self.calib_fx = None
        self.calib_fy = None
        self.calib_cx = None
        self.calib_cy = None
        self.latest_lidar_msg: Optional[PointCloud2] = None
        self.latest_lidar_arrival: Optional[rospy.Time] = None

        self._load_calibration()

        self.visual_depth_pub = rospy.Publisher(self.visual_depth_topic, Image, queue_size=1)
        self.lidar_depth_pub = rospy.Publisher(self.lidar_depth_topic, Image, queue_size=1)
        self.fused_depth_pub = rospy.Publisher(self.fused_depth_topic, Image, queue_size=1)
        self.fused_depth_viz_pub = rospy.Publisher(self.fused_depth_viz_topic, Image, queue_size=1)
        self.debug_overlay_pub = rospy.Publisher(self.debug_overlay_topic, Image, queue_size=1)

        rospy.Subscriber(self.left_info_topic, CameraInfo, self.left_info_callback, queue_size=1)
        rospy.Subscriber(self.right_info_topic, CameraInfo, self.right_info_callback, queue_size=1)

        left_sub = message_filters.Subscriber(
            self.left_topic,
            Image,
            queue_size=1,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )
        disp_sub = message_filters.Subscriber(
            self.ffs_disp_topic,
            Image,
            queue_size=1,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, disp_sub], queue_size=5, slop=self.sync_slop
        )
        self.sync.registerCallback(self.callback)
        rospy.Subscriber(
            self.lidar_topic,
            PointCloud2,
            self.lidar_callback,
            queue_size=1,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )

        rospy.loginfo("ffs_lidar_fusion node started")
        rospy.loginfo("left_topic      = %s", self.left_topic)
        rospy.loginfo("ffs_disp_topic  = %s", self.ffs_disp_topic)
        rospy.loginfo("lidar_topic     = %s", self.lidar_topic)
        rospy.loginfo("max_lidar_age   = %.3f s", self.max_lidar_age)
        rospy.loginfo("max_input_age   = %.3f s", self.max_input_age)
        rospy.loginfo("lidar_stride    = %d", self.lidar_stride)

    def _load_calibration(self):
        calib_path = self.calib_json
        if calib_path.startswith("$(find "):
            rospy.logwarn(
                "Parameter ~calib_json still contains roslaunch substitution syntax. "
                "Use a launch file to resolve it. Current value: %s",
                calib_path,
            )
            return

        try:
            with open(calib_path, "r", encoding="utf-8") as f:
                calib = json.load(f)
        except Exception as exc:
            rospy.logwarn("Failed to load calib_json %s: %s", calib_path, exc)
            return

        rotation = np.asarray(calib["rotation"], dtype=np.float32).reshape(3, 3)
        translation = np.asarray(calib["translation"], dtype=np.float32).reshape(3, 1)

        self.rotation_l2c = np.linalg.inv(rotation)
        self.translation_l2c = -self.rotation_l2c @ translation

        intrinsic = calib.get("undistort_intrinsic", calib.get("intrinsic", None))
        if intrinsic is not None:
            intrinsic = np.asarray(intrinsic, dtype=np.float32).reshape(3, 3)
            self.calib_fx = float(intrinsic[0, 0])
            self.calib_fy = float(intrinsic[1, 1])
            self.calib_cx = float(intrinsic[0, 2])
            self.calib_cy = float(intrinsic[1, 2])

        rospy.loginfo("Loaded lidar-camera calibration from %s", calib_path)

    def left_info_callback(self, msg: CameraInfo):
        self.left_info = msg

    def right_info_callback(self, msg: CameraInfo):
        self.right_info = msg

    def lidar_callback(self, msg: PointCloud2):
        # Keep the latest lidar message by wall-clock arrival time. This avoids
        # device-clock header stamp mismatch with camera/FFS ROS time.
        self.latest_lidar_msg = msg
        self.latest_lidar_arrival = rospy.Time.now()

    def callback(self, left_msg: Image, disp_msg: Image):
        if self.left_info is None or self.right_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for left/right camera info")
            return
        if disp_msg.header.stamp and self.max_input_age > 0:
            input_age = (rospy.Time.now() - disp_msg.header.stamp).to_sec()
            if input_age > self.max_input_age:
                rospy.logwarn_throttle(
                    2.0,
                    "Drop stale fusion pair: age=%.3f s (> %.3f s)",
                    input_age,
                    self.max_input_age,
                )
                return
        if self.latest_lidar_msg is None or self.latest_lidar_arrival is None:
            rospy.logwarn_throttle(2.0, "Waiting for lidar input")
            return
        lidar_age = (rospy.Time.now() - self.latest_lidar_arrival).to_sec()
        if lidar_age > self.max_lidar_age:
            rospy.logwarn_throttle(
                2.0, "Lidar data too old: %.3f s (max %.3f s)", lidar_age, self.max_lidar_age
            )
            return

        try:
            left_bgr = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            disparity = self.bridge.imgmsg_to_cv2(disp_msg, desired_encoding="32FC1")
        except CvBridgeError as exc:
            rospy.logerr_throttle(1.0, "cv_bridge conversion failed: %s", exc)
            return

        visual_depth = self.disparity_to_depth(disparity, self.left_info, self.right_info)
        lidar_depth = self.project_lidar_to_depth(self.latest_lidar_msg, visual_depth.shape)
        fused_depth = self.fuse_depths(visual_depth, lidar_depth)

        self.visual_depth_pub.publish(self.float_depth_to_msg(visual_depth, left_msg.header))
        self.lidar_depth_pub.publish(self.float_depth_to_msg(lidar_depth, left_msg.header))
        self.fused_depth_pub.publish(self.float_depth_to_msg(fused_depth, left_msg.header))
        self.fused_depth_viz_pub.publish(
            self.color_depth_to_msg(fused_depth, left_msg.header, "Fused")
        )

        if self.publish_debug_overlay:
            overlay = self.build_overlay(left_bgr, lidar_depth, fused_depth)
            self.debug_overlay_pub.publish(self.bgr_to_msg(overlay, left_msg.header))

    def disparity_to_depth(
        self, disparity: np.ndarray, left_info: CameraInfo, right_info: CameraInfo
    ) -> np.ndarray:
        fx = float(left_info.P[0]) if left_info.P[0] > 0 else float(left_info.K[0])
        tx = float(right_info.P[3])
        baseline = abs(tx / fx) if fx > 0 and abs(tx) > 1e-9 else 0.0

        if baseline <= 0.0:
            rospy.logwarn_throttle(2.0, "Invalid stereo baseline from camera info")
            return np.zeros_like(disparity, dtype=np.float32)

        depth = np.zeros_like(disparity, dtype=np.float32)
        valid = np.isfinite(disparity) & (disparity > self.min_disp)
        depth[valid] = (fx * baseline) / disparity[valid]
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0.0
        return depth

    def project_lidar_to_depth(
        self, lidar_msg: PointCloud2, image_shape: Tuple[int, int]
    ) -> np.ndarray:
        height, width = image_shape[:2]
        depth = np.zeros((height, width), dtype=np.float32)

        fx, fy, cx, cy = self.choose_projection_intrinsics()
        if fx is None:
            rospy.logwarn_throttle(2.0, "No valid projection intrinsics available yet")
            return depth

        for idx, pt in enumerate(pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)):
            if (idx % self.lidar_stride) != 0:
                continue
            xyz_lidar = np.asarray(pt, dtype=np.float32).reshape(3, 1)
            xyz_cam = self.rotation_l2c @ xyz_lidar + self.translation_l2c
            z = float(xyz_cam[2, 0])
            if z <= self.min_depth or z > self.max_depth:
                continue

            u = int(round((float(xyz_cam[0, 0]) / z) * fx + cx))
            v = int(round((float(xyz_cam[1, 0]) / z) * fy + cy))
            if u < 0 or u >= width or v < 0 or v >= height:
                continue

            old = depth[v, u]
            if old == 0.0 or z < old:
                depth[v, u] = z

        if self.use_lidar_dilation and self.lidar_dilation_kernel > 1:
            kernel = np.ones(
                (self.lidar_dilation_kernel, self.lidar_dilation_kernel), dtype=np.uint8
            )
            valid_mask = (depth > 0.0).astype(np.uint8)
            dilated_mask = cv2.dilate(valid_mask, kernel, iterations=1)
            dilated_depth = cv2.dilate(depth, kernel, iterations=1)
            depth = np.where(dilated_mask > 0, dilated_depth, 0.0).astype(np.float32)

        return depth

    def choose_projection_intrinsics(self):
        if None not in (self.calib_fx, self.calib_fy, self.calib_cx, self.calib_cy):
            return self.calib_fx, self.calib_fy, self.calib_cx, self.calib_cy

        if self.left_info is None:
            return None, None, None, None

        fx = float(self.left_info.P[0]) if self.left_info.P[0] > 0 else float(self.left_info.K[0])
        fy = float(self.left_info.P[5]) if self.left_info.P[5] > 0 else float(self.left_info.K[4])
        cx = float(self.left_info.P[2]) if self.left_info.P[2] > 0 else float(self.left_info.K[2])
        cy = float(self.left_info.P[6]) if self.left_info.P[6] > 0 else float(self.left_info.K[5])
        return fx, fy, cx, cy

    def fuse_depths(self, visual_depth: np.ndarray, lidar_depth: np.ndarray) -> np.ndarray:
        fused = visual_depth.copy()
        lidar_valid = lidar_depth > 0.0
        fused[lidar_valid] = lidar_depth[lidar_valid]
        fused[(fused < self.min_depth) | (fused > self.max_depth)] = 0.0
        return fused

    def float_depth_to_msg(self, depth: np.ndarray, header) -> Image:
        msg = Image()
        msg.header = copy.deepcopy(header)
        msg.height, msg.width = depth.shape[:2]
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = np.ascontiguousarray(depth.astype(np.float32)).tobytes()
        return msg

    def color_depth_to_msg(self, depth: np.ndarray, header, label: str) -> Image:
        valid = depth > 0.0
        depth_norm = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            clipped = np.clip(depth, self.min_depth, self.max_depth)
            normalized = 255.0 * (1.0 - (clipped - self.min_depth) / (self.max_depth - self.min_depth))
            depth_norm[valid] = normalized[valid].astype(np.uint8)
        color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
        color[~valid] = 0
        cv2.putText(
            color,
            label,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return self.bgr_to_msg(color, header)

    def build_overlay(
        self, left_bgr: np.ndarray, lidar_depth: np.ndarray, fused_depth: np.ndarray
    ) -> np.ndarray:
        overlay = left_bgr.copy()
        h, w = overlay.shape[:2]
        if fused_depth.shape[:2] != (h, w):
            rospy.logwarn_throttle(
                2.0,
                "Resizing fused_depth for overlay: %s -> (%d, %d)",
                str(fused_depth.shape[:2]),
                h,
                w,
            )
            fused_depth = cv2.resize(fused_depth, (w, h), interpolation=cv2.INTER_NEAREST)
        if lidar_depth.shape[:2] != (h, w):
            rospy.logwarn_throttle(
                2.0,
                "Resizing lidar_depth for overlay: %s -> (%d, %d)",
                str(lidar_depth.shape[:2]),
                h,
                w,
            )
            lidar_depth = cv2.resize(lidar_depth, (w, h), interpolation=cv2.INTER_NEAREST)

        valid_lidar = lidar_depth > 0.0
        valid_fused = fused_depth > 0.0

        if np.any(valid_fused):
            fused_vis = self.depth_to_colormap(fused_depth)
            overlay = cv2.addWeighted(overlay, 0.55, fused_vis, 0.45, 0.0)

        overlay[valid_lidar] = (0, 255, 0)
        cv2.putText(
            overlay,
            "green: lidar support",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return overlay

    def depth_to_colormap(self, depth: np.ndarray) -> np.ndarray:
        valid = depth > 0.0
        depth_norm = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            clipped = np.clip(depth, self.min_depth, self.max_depth)
            normalized = 255.0 * (1.0 - (clipped - self.min_depth) / (self.max_depth - self.min_depth))
            depth_norm[valid] = normalized[valid].astype(np.uint8)
        color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
        color[~valid] = 0
        return color

    def bgr_to_msg(self, image: np.ndarray, header) -> Image:
        msg = Image()
        msg.header = copy.deepcopy(header)
        msg.height, msg.width = image.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(image).tobytes()
        return msg


if __name__ == "__main__":
    rospy.init_node("ffs_lidar_fusion_node")
    FFSLidarFusionNode()
    rospy.spin()
