#Author : Arno Laurie
#Date : 09/12/2024

from ..interfaces.stereo_camera_interface import StereoCameraInterface
import depthai as dai
import cv2
import time
import rclpy
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
        self.fps = 24

        self.pipeline = dai.Pipeline()

        #mono = black and white image from the left and right cams

        self.mono_left = self.pipeline.createMonoCamera()
        self.mono_right = self.pipeline.createMonoCamera()

        self.mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        self.mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)


        # ColorCamera = rgb output from the oakd  
        self.color_cam = self.pipeline.createColorCamera()
        self.color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.color_cam.setFps(self.fps)
        self.color_cam.setInterleaved(False)  
        # Non-interleaved frames for better compatibility with ros2 and opencv

        # stereo depth
        self.stereo = self.pipeline.createStereoDepth()
        self.stereo.setOutputDepth(True)
        self.stereo.setOutputRectified(True)
        self.stereo.setConfidenceThreshold(200)

        # Link Mono Cameras to Stereo Depth
        self.mono_left.out.link(self.stereo.left)
        self.mono_right.out.link(self.stereo.right)

         # Link ColorCamera video output to XLinkOut for streaming rgb frames
        self.rgb_out = self.pipeline.createXLinkOut()
        self.rgb_out.setStreamName("rgb")
        self.color_cam.video.link(self.rgb_out.input)

        # Link StereoDepth depth output to XLinkOut for streaming
        self.depth_out = self.pipeline.createXLinkOut()
        self.depth_out.setStreamName("depth")
        self.stereo.depth.link(self.depth_out.input)

        # try to connect to oakd
        try:
            self.device = dai.Device(self.pipeline)
            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            self.node.get_logger().info("OAK-D pipeline started successfully.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to initialize OAK-D: {e}")
            raise

        self.cam_pubs = self.node.cam_pubs
        self.cam_bw = self.node.cam_bw
        self.depth_pubs = self.node.create_publisher(CompressedImage, self.node.publisher_topic + "/depth", 1)



    def get_rgb(self):
        """Retrieve an RGB frame from the RGB queue."""
        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is None:
            #self.node.get_logger().warn("No RGB frame received.")
            return None
        return rgb_frame.getCvFrame()

    def get_rgbd(self):
        """Retrieve both RGB and Depth frames."""
        rgb_frame = self.get_rgb()
        depth_frame = self.get_depth()
        if rgb_frame is None or depth_frame is None:
            #self.node.get_logger().warn("No RGB or Depth frame received.")
            return None, None
        return rgb_frame, depth_frame

    def get_depth(self):
        """Retrieve a Depth frame from the Depth queue."""
        depth_frame = self.depth_queue.tryGet()
        if depth_frame is None:
            #self.node.get_logger().warn("No depth frame received.")
            return None
        return depth_frame.getFrame()

    def get_intrinsics(self):
        calib_data = self.device.readCalibration()
        intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
        return intrinsics

    def get_coeffs(self):
        calib_data = self.device.readCalibration()
        return calib_data.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
    
    def publish_feeds(self, devrule=None):
        """Publish RGB and Depth feeds."""
        self.node.get_logger().info("Starting to publish RGB and Depth feeds from OAK-D camera.")
        image_idx = 0
        previous_time = time.time()

        while not self.node.stopped:
            # Get RGB Frame
            rgb_img = self.get_rgb()
            if rgb_img is not None:
                compressed_rgb = self.bridge.cv2_to_compressed_imgmsg(rgb_img, dst_format="jpeg")
                self.cam_pubs.publish(compressed_rgb)

            # Get Depth Frame
            #depth_img = self.get_depth()
            #if depth_img is not None:
                # Normalize depth image for visualization
                #depth_img_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
                #depth_img_colored = cv2.applyColorMap(depth_img_normalized.astype('uint8'), cv2.COLORMAP_JET)
                #compressed_depth = self.bridge.cv2_to_compressed_imgmsg(depth_img_colored, dst_format="jpeg")
                #self.depth_pubs.publish(compressed_depth)

            # Bandwidth Calculation
            current_time = time.time()
            elapsed_time = max(current_time - previous_time, 1e-8)
            previous_time = current_time

            bw = Float32()
            bw.data = float((len(compressed_rgb.data) * 8) / (elapsed_time * 1_000_000))

            self.cam_bw.publish(bw)

            #self.node.get_logger().info(f"Captured Frame {image_idx} | Bandwidth: {bw.data:.2f} Mbps")
            image_idx += 1

            time.sleep(1 / self.fps)

        self.node.get_logger().info("Stopping OAK-D feed.")
        self.device.close()