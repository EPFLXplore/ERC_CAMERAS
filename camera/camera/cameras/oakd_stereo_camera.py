#Author : Arno Laurie
#Date : 21/11/2024

from ..interfaces.stereo_camera_interface import StereoCameraInterface
import depthai as dai
import cv2
from time import sleep
import time
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sys
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage


class OakDStereoCamera(StereoCameraInterface):
    def __init__(self, node):
        # raise NotImplementedError(f"__init__ not implemented for {self._name}")
        self.node = node
        self.bridge = CvBridge()
        self.fps = 30

        self.pipeline = dai.Pipeline()

        self.color_cam = self.pipeline.createColorCamera()
        self.color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.color_cam.setFps(self.fps)

        self.stereo = self.pipeline.createStereoDepth()
        self.stereo.setOutputDepth(True)
        self.stereo.setOutputRectified(True)
        self.stereo.setConfidenceThreshold(200)

        self.rgb_out = self.pipeline.createXLinkOut()
        self.rgb_out.setStreamName("rgb")
        self.color_cam.video.link(self.rgb_out.input)

        self.depth_out = self.pipeline.createXLinkOut()
        self.depth_out.setStreamName("depth")
        self.stereo.depth.link(self.depth_out.input)

        try:
            self.device = dai.Device(self.pipeline)
            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            self.node.get_logger().info("OAK-D pipeline started successfully.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to initialize OAK-D: {e}")
            raise

        self.cam_pubs = self.node.create_publisher(CompressedImage, '/camera/rgb/compressed', 10)
        self.cam_bw = self.node.create_publisher(Float32, '/camera/bandwidth', 10)


    def get_image(self):
        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is None:
            self.node.get_logger().warn("No RGB frame received.")
            return None
        return rgb_frame.getCvFrame()
    
    def get_depth(self):
        depth_frame = self.depth_queue.tryGet()
        if depth_frame is None:
            self.node.get_logger().warn("No depth frame received.")
            return None
        return depth_frame.getFrame()

    def get_intrinsics(self):
        calib_data = self.device.readCalibration()
        intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
        return intrinsics

    def get_coeffs(self):
        calib_data = self.device.readCalibration()
        return calib_data.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
    
    def publish_feeds(self, camera_id):
        self.node.get_logger().info("Starting to publish RGB feed from OAK-D camera.")
        image_idx = 0
        previous_time = time.time()

        while not self.node.stopped:
            frame = self.get_image()
            if frame is None:
                sleep(1 / self.fps)
                continue

            compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
            current_time = time.time()
            elapsed_time = max(current_time - previous_time, 1e-8)  # Avoid div by zero
            previous_time = current_time

            bw = Float32()
            bw.data = float((len(compressed_image.data) * 8) / (elapsed_time * 1_000_000))  # Bandwidth in Mbps

            self.cam_pubs.publish(compressed_image)
            self.cam_bw.publish(bw)

            self.node.get_logger().info(f"Captured Frame {image_idx} | Bandwidth: {bw.data:.2f} Mbps")
            image_idx += 1

            sleep(1 / self.fps)  # Limit to 30 FPS

        # Cleanup when stopped
        self.node.get_logger().info("Stopping OAK-D feed.")
        self.device.close()