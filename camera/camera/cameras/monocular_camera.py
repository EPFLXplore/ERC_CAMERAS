from ..interfaces.monocular_camera_interface import MonocularCameraInterface
import cv2
from time import sleep
import time
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class MonocularCamera(MonocularCameraInterface):
    def __init__(self, node):

        self.node = node

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()

    def publish_feeds(self, camera_id):
        self.node.get_logger().info("REQUEST")
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        camera.set(cv2.CAP_PROP_FPS, 15)

        while True:
            self.node.get_logger().info("RDGRE")
            for i in range(10):
                ret, frame = camera.read()

            image_idx = 0
            while True:
                self.node.get_logger().info("Capturing " + str(image_idx) + " | time: " + str(time.time()))
                ret, frame = camera.read()
                if (not ret) or self.node.stopped:
                    break
                
                compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
                self.node.cam_pubs.publish(compressed_image)
                image_idx += 1
                sleep(1/15)

            if self.node.stopped:
                break

            camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
            camera.set(cv2.CAP_PROP_FPS, 15)
            sleep(1)

    def get_rgb(self):
        pass
