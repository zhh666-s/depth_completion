


#!/opt/miniconda3/envs/ffs_py310/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import copy
import time
import numpy as np
import cv2

ROS_PY_PATH = "/opt/ros/noetic/lib/python3/dist-packages"
if ROS_PY_PATH not in sys.path:
    sys.path.insert(0, ROS_PY_PATH)

import rospy
from sensor_msgs.msg import Image
import message_filters

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
FFS_SCRIPT_DIR = os.path.join(PKG_ROOT, "ffs_runtime", "scripts")
if FFS_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, FFS_SCRIPT_DIR)

from test_video_model_stream import (
    PROJECT_ROOT,
    configure_torch_runtime,
    load_model_and_cfg,
    infer_one_frame,
    draw_info,
)

def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()

    if enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.step)
        arr = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(arr)

    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.step)
        arr = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(arr)

    if enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape(msg.height, msg.step)
        arr = arr[:, : msg.width]
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        return np.ascontiguousarray(arr)

    raise ValueError(f"Unsupported encoding: {msg.encoding}")

def bgr_to_imgmsg(img: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = copy.deepcopy(header)
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg

def float32_to_imgmsg(img: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = copy.deepcopy(header)
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = msg.width * 4
    msg.data = np.ascontiguousarray(img.astype(np.float32)).tobytes()
    return msg

class FFSTopicNode:
    def __init__(self):
        default_model = os.path.join(
            PROJECT_ROOT, "weights", "20-26-39", "model_best_bp2_serialize.pth"
        )

        self.left_topic = rospy.get_param("~left_topic", "/camera/left/image_raw")
        self.right_topic = rospy.get_param("~right_topic", "/camera/right/image_raw")
        self.disp_topic = rospy.get_param("~disp_topic", "/ffs/disp")
        self.vis_topic = rospy.get_param("~vis_topic", "/ffs/vis")
        self.publish_disp = bool(rospy.get_param("~publish_disp", False))
        self.publish_vis = bool(rospy.get_param("~publish_vis", True))
        self.model_dir = rospy.get_param("~model_dir", default_model)
        self.valid_iters = int(rospy.get_param("~valid_iters", 8))
        self.max_disp = int(rospy.get_param("~max_disp", 192))
        self.hiera = int(rospy.get_param("~hiera", 0))
        self.skip = int(rospy.get_param("~skip", 0))
        self.show_local = bool(rospy.get_param("~show_local", False))
        self.display_scale = float(rospy.get_param("~display_scale", 0.8))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.03))
        self.input_queue_size = int(rospy.get_param("~input_queue_size", 1))
        self.sync_queue_size = int(rospy.get_param("~sync_queue_size", 2))
        self.max_input_age = float(rospy.get_param("~max_input_age", 0.25))
        self.infer_width = int(rospy.get_param("~infer_width", 0))
        self.infer_height = int(rospy.get_param("~infer_height", 0))
        self.enable_tf32 = bool(rospy.get_param("~enable_tf32", True))
        self.cudnn_benchmark = bool(rospy.get_param("~cudnn_benchmark", True))
        self.use_torch_compile = bool(rospy.get_param("~use_torch_compile", False))
        self.compile_mode = str(rospy.get_param("~compile_mode", "reduce-overhead"))
        self.use_channels_last = bool(rospy.get_param("~use_channels_last", False))

        rospy.loginfo(f"PROJECT_ROOT = {PROJECT_ROOT}")
        rospy.loginfo(f"Using model = {self.model_dir}")
        rospy.loginfo(f"Subscribe left topic  = {self.left_topic}")
        rospy.loginfo(f"Subscribe right topic = {self.right_topic}")
        rospy.loginfo(
            "Latency opts: input_queue=%d, sync_queue=%d, max_input_age=%.3fs",
            self.input_queue_size,
            self.sync_queue_size,
            self.max_input_age,
        )
        rospy.loginfo("Infer resize: width=%d, height=%d (0 means disabled)", self.infer_width, self.infer_height)
        rospy.loginfo(
            "GPU accel opts: tf32=%s, cudnn_benchmark=%s, torch_compile=%s(%s), channels_last=%s",
            self.enable_tf32,
            self.cudnn_benchmark,
            self.use_torch_compile,
            self.compile_mode,
            self.use_channels_last,
        )

        configure_torch_runtime(
            enable_tf32=self.enable_tf32,
            cudnn_benchmark=self.cudnn_benchmark,
        )

        self.model, _ = load_model_and_cfg(
            model_dir=self.model_dir,
            valid_iters=self.valid_iters,
            max_disp=self.max_disp,
            use_torch_compile=self.use_torch_compile,
            compile_mode=self.compile_mode,
            use_channels_last=self.use_channels_last,
        )

        self.disp_pub = rospy.Publisher(self.disp_topic, Image, queue_size=1) if self.publish_disp else None
        self.vis_pub = rospy.Publisher(self.vis_topic, Image, queue_size=1) if self.publish_vis else None

        self.frame_idx = 0
        self.last_t = time.time()

        left_sub = message_filters.Subscriber(
            self.left_topic,
            Image,
            queue_size=self.input_queue_size,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )
        right_sub = message_filters.Subscriber(
            self.right_topic,
            Image,
            queue_size=self.input_queue_size,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )
        self.sync.registerCallback(self.callback)

        if self.show_local:
            cv2.namedWindow("FFS Topic Node", cv2.WINDOW_NORMAL)

    def callback(self, left_msg: Image, right_msg: Image):
        try:
            # Drop stale synchronized pairs to keep latency low when model FPS < input FPS.
            if left_msg.header.stamp and self.max_input_age > 0:
                age = (rospy.Time.now() - left_msg.header.stamp).to_sec()
                if age > self.max_input_age:
                    rospy.logwarn_throttle(
                        2.0,
                        "Drop stale frame pair: age=%.3fs (> %.3fs)",
                        age,
                        self.max_input_age,
                    )
                    return

            if self.skip > 0 and (self.frame_idx % (self.skip + 1)) != 0:
                self.frame_idx += 1
                return
            self.frame_idx += 1

            left_bgr = imgmsg_to_bgr(left_msg)
            right_bgr = imgmsg_to_bgr(right_msg)

            # Optional pre-inference resize for throughput tuning.
            if self.infer_width > 0 and self.infer_height > 0:
                left_bgr = cv2.resize(
                    left_bgr, (self.infer_width, self.infer_height), interpolation=cv2.INTER_AREA
                )
                right_bgr = cv2.resize(
                    right_bgr, (self.infer_width, self.infer_height), interpolation=cv2.INTER_AREA
                )

            need_vis = self.show_local or (self.publish_vis and self.vis_pub is not None and self.vis_pub.get_num_connections() > 0)
            disp, vis_triplet = infer_one_frame(
                model=self.model,
                left_bgr=left_bgr,
                right_bgr=right_bgr,
                valid_iters=self.valid_iters,
                hiera=self.hiera,
                use_channels_last=self.use_channels_last,
                build_vis=need_vis,
            )

            if self.publish_disp and self.disp_pub is not None:
                self.disp_pub.publish(float32_to_imgmsg(disp, left_msg.header))
            if self.publish_vis and self.vis_pub is not None and vis_triplet is not None:
                self.vis_pub.publish(bgr_to_imgmsg(vis_triplet, left_msg.header))

            now = time.time()
            fps = 1.0 / max(now - self.last_t, 1e-6)
            self.last_t = now
            lag = 0.0
            if left_msg.header.stamp:
                lag = (rospy.Time.now() - left_msg.header.stamp).to_sec()
            rospy.loginfo_throttle(2.0, "FFS inference fps: %.2f, e2e lag: %.3fs", fps, lag)

            if self.show_local:
                if vis_triplet is None:
                    return
                show = vis_triplet.copy()
                draw_info(show, f"Node FPS: {fps:.1f}", 20, 35)
                if self.display_scale != 1.0:
                    show = cv2.resize(
                        show,
                        None,
                        fx=self.display_scale,
                        fy=self.display_scale,
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow("FFS Topic Node", show)
                cv2.waitKey(1)

        except Exception as e:
            rospy.logerr_throttle(1.0, f"FFS node callback failed: {e}")

if __name__ == "__main__":
    rospy.init_node("ffs_topic_node")
    node = FFSTopicNode()
    rospy.spin()
