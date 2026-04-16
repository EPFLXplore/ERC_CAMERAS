import os
import time

import cv2
from sensor_msgs.msg import CompressedImage


class MonocularCamera:
    def __init__(self, node):
        self.node = node

    def publish_feeds(self):
        """Open the V4L device from ROS param `devrule` only (path or udev symlink)."""
        raw = (self.node.devrule or "").strip()
        if not raw:
            self.node.get_logger().error(
                "Parameter 'devrule' is empty; set e.g. /dev/video0"
            )
            return

        device_path = os.path.realpath(raw)
        if not os.path.exists(device_path):
            self.node.get_logger().error(
                f"devrule not found: '{raw}' (resolved: '{device_path}')"
            )
            return

        self.node.get_logger().info(
            f"Opening camera devrule='{raw}' -> {device_path}"
        )

        cam = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not cam.isOpened():
            self.node.get_logger().error(f"Failed to open V4L2 device: {device_path}")
            return

        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc("M", "J", "P", "G"))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.node.x))
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.node.y))
        cam.set(cv2.CAP_PROP_FPS, float(self.node.fps))
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # don't buffer stale frames

        quality = max(1, min(100, self.node.jpeg_quality))
        encode_params = [
            cv2.IMWRITE_JPEG_QUALITY,
            quality,
            cv2.IMWRITE_JPEG_OPTIMIZE,
            0,
        ]  # optimize=0 is faster

        msg = CompressedImage()
        msg.format = "jpeg"
        previous_time = 0.0

        while not self.node.stopped:
            ret, frame = cam.read()
            if not ret or frame is None:
                continue

            ok, buf = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue

            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.data = buf.tobytes()

            current_time = time.time()
            bw = self.node.calculate_bandwidth(
                current_time, previous_time, len(msg.data)
            )
            previous_time = current_time

            self.node.cam_pubs.publish(msg)
            self.node.cam_bw.publish(bw)

        cam.release()
