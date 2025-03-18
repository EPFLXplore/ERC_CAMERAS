from ..interfaces.monocular_camera_interface import MonocularCameraInterface
import cv2
from time import sleep
import time
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys
from std_msgs.msg import Float32

class MonocularCamera(MonocularCameraInterface):
    def __init__(self, node):

        self.node = node

        self.bridge = CvBridge()

    def publish_feeds(self, camera_id):
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        camera.set(cv2.CAP_PROP_FPS, 15)

        while True:
            for i in range(10):
                ret, frame = camera.read()

            image_idx = 0
            previous_time = 0
            while True:
                #self.node.get_logger().info("Capturing " + str(image_idx) + " | time: " + str(time.time()))
                ret, frame = camera.read()
                if (not ret) or self.node.stopped:
                    break
                
                compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
                frame_size = sys.getsizeof(frame)
                # Calculate the BW 
                current_time = time.time()
                elapsed_time = current_time - previous_time
                bw = Float32()
                bw.data = float((len(compressed_image.data) * 8) / (elapsed_time * 1_000_000))  # Bandwidth in Mbps
                previous_time = current_time 

                self.node.cam_pubs.publish(compressed_image)
                self.node.cam_bw.publish(bw) 
                image_idx += 1
                sleep(1/15)

            if self.node.stopped:
                break

            camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
            camera.set(cv2.CAP_PROP_FPS, 15)
            sleep(1)

    def get_rgb(self):
        pass