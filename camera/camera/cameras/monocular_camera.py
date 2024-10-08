from ..interfaces.monocular_camera_interface import MonocularCameraInterface
import cv2
import time
from time import sleep
import threading, yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

'''
FOR THE 4 CS CAMERAS
'''

class MonocularCamera(MonocularCameraInterface):
    def __init__(self, node):

        with open('/home/xplore/dev_ws/src/custom_msg/config/rover_interface_names.yaml', 'r') as file:
            self.rover_names = yaml.safe_load(file)["/**"]["ros__parameters"]

        self.camera_ids = ["/dev/video0", "/dev/video2", "/dev/video8", "/dev/video26"]
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # publishers for the cameras
        self.cam_pubs = [self.create_publisher(CompressedImage, self.rover_names['rover_cameras_cs_prefix'] + str(i), qos_profile) for i in range(len(self.camera_ids))]
        self.bridge = CvBridge()

        global stop_threads
        stop_threads = False
        self.threads = []
        
        self.threads = [threading.Thread(target=publish_feeds, args=(self.camera_ids[i], self.cam_pubs[i], self.bridge,)) for i in range(len(self.camera_ids))]

    def start_cameras_callback(self, request, response):
        global stop_threads
        if request.data:
            stop_threads = False

            self.threads = [threading.Thread(target=publish_feeds, args=(self.camera_ids[i], self.cam_pubs[i], self.bridge,)) for i in range(len(self.camera_ids))]

            for thread in self.threads:
                thread.start()
            response.success = True
            response.message = "Cameras started"
        else:
            stop_threads = True
            for thread in self.threads:
                thread.join()

            self.threads = []
            
            response.success = True
            response.message = "Cameras stopped"
        return response


def publish_feeds(camera_id, publisher, bridge):
    camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
    camera.set(cv2.CAP_PROP_FPS, 15)
    global stop_threads
    
    while True:

        for i in range(10):
            ret, frame = camera.read()

        image_idx = 0
        while True:
            ret, frame = camera.read()
            if (not ret) or stop_threads:
                break
            
            compressed_image = bridge.cv2_to_compressed_imgmsg(frame)
            publisher.publish(compressed_image)
            image_idx += 1
            sleep(1/15)

        if stop_threads:
            break
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
        camera.set(cv2.CAP_PROP_FPS, 15)
        sleep(1)
