#!/usr/bin/env python3

import os
import time
from typing import Optional
from urllib.parse import urlsplit, urlunsplit

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def sanitize_url(url: str) -> str:
    parts = urlsplit(url)
    if "@" not in parts.netloc:
        return url

    userinfo, host = parts.netloc.rsplit("@", 1)
    if ":" in userinfo:
        username, _ = userinfo.split(":", 1)
        userinfo = f"{username}:****"
    else:
        userinfo = "****"
    return urlunsplit((parts.scheme, f"{userinfo}@{host}", parts.path, parts.query, parts.fragment))


class HikvisionNode:
    def __init__(self):
        self.bridge = CvBridge()

        self.rtsp_url = rospy.get_param(
            "~rtsp_url",
            "rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101",
        )
        self.image_topic = rospy.get_param("~image_topic", "/hikvison/image_raw")
        self.frame_id = rospy.get_param("~frame_id", "hikvision_camera")
        self.encoding = rospy.get_param("~encoding", "bgr8")
        self.capture_backend = rospy.get_param("~capture_backend", "ffmpeg").lower()
        self.ffmpeg_capture_options = rospy.get_param(
            "~ffmpeg_capture_options", "rtsp_transport;tcp"
        )
        self.reconnect_delay = max(0.1, float(rospy.get_param("~reconnect_delay", 2.0)))
        self.read_fail_limit = max(1, int(rospy.get_param("~read_fail_limit", 25)))
        self.target_fps = float(rospy.get_param("~target_fps", 0.0))
        self.show_local = bool(rospy.get_param("~show_local", False))
        self.report_interval = max(1.0, float(rospy.get_param("~report_interval", 5.0)))

        self.publisher = rospy.Publisher(self.image_topic, Image, queue_size=1)
        self.capture: Optional[cv2.VideoCapture] = None

        rospy.loginfo("hikvision node started")
        rospy.loginfo("rtsp_url        = %s", sanitize_url(self.rtsp_url))
        rospy.loginfo("image_topic     = %s", self.image_topic)
        rospy.loginfo("frame_id        = %s", self.frame_id)
        rospy.loginfo("encoding        = %s", self.encoding)
        rospy.loginfo("capture_backend = %s", self.capture_backend)
        rospy.loginfo("target_fps      = %.2f", self.target_fps)

    def configure_ffmpeg_options(self):
        if self.capture_backend != "ffmpeg" or not self.ffmpeg_capture_options:
            return
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = self.ffmpeg_capture_options
        rospy.loginfo("OPENCV_FFMPEG_CAPTURE_OPTIONS=%s", self.ffmpeg_capture_options)

    def open_capture(self) -> bool:
        self.close_capture()
        self.configure_ffmpeg_options()

        if self.capture_backend == "ffmpeg" and hasattr(cv2, "CAP_FFMPEG"):
            capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        elif self.capture_backend == "gstreamer" and hasattr(cv2, "CAP_GSTREAMER"):
            capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_GSTREAMER)
        else:
            capture = cv2.VideoCapture(self.rtsp_url)

        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not capture.isOpened():
            rospy.logwarn("Failed to open RTSP stream: %s", sanitize_url(self.rtsp_url))
            capture.release()
            return False

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = capture.get(cv2.CAP_PROP_FPS)
        rospy.loginfo("Opened RTSP stream: %dx%d %.2f fps", width, height, fps)
        self.capture = capture
        return True

    def close_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def publish_frame(self, frame):
        header_stamp = rospy.Time.now()
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
        except CvBridgeError as exc:
            rospy.logerr_throttle(1.0, "cv_bridge conversion failed: %s", exc)
            return

        msg.header.stamp = header_stamp
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def spin(self):
        frame_count = 0
        fail_count = 0
        last_report = time.monotonic()
        last_publish = 0.0
        min_period = 1.0 / self.target_fps if self.target_fps > 0 else 0.0

        while not rospy.is_shutdown():
            if self.capture is None and not self.open_capture():
                rospy.sleep(self.reconnect_delay)
                continue

            ok, frame = self.capture.read()
            if not ok or frame is None or frame.size == 0:
                fail_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    "Failed to read RTSP frame (%d/%d)",
                    fail_count,
                    self.read_fail_limit,
                )
                if fail_count >= self.read_fail_limit:
                    rospy.logwarn("Reconnecting RTSP stream after repeated read failures")
                    self.close_capture()
                    fail_count = 0
                    rospy.sleep(self.reconnect_delay)
                continue

            fail_count = 0
            now = time.monotonic()
            if min_period > 0.0 and now - last_publish < min_period:
                continue

            self.publish_frame(frame)
            last_publish = now
            frame_count += 1

            if self.show_local:
                cv2.imshow("hikvision", frame)
                cv2.waitKey(1)

            if now - last_report >= self.report_interval:
                rospy.loginfo("Published %.2f fps to %s", frame_count / (now - last_report), self.image_topic)
                frame_count = 0
                last_report = now

        self.close_capture()
        if self.show_local:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("hikvision_node")
    node = HikvisionNode()
    node.spin()


if __name__ == "__main__":
    main()
