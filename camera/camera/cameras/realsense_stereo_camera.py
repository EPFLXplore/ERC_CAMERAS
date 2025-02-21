
import pyrealsense2 as rs
#import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2 as cv
import yaml
import time
from ..interfaces.stereo_camera_interface import StereoCameraInterface
from cv_bridge import CvBridge
from time import sleep
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class RealSenseStereoCamera(StereoCameraInterface):

    def __init__(self, node):

        self.node = node

        self.fps = 15
        self.x = 1280
        self.y = 720

        self.serial_number = self.node.devrule  #we do a bit of cheating

        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.
        self.config = rs.config()  # Disable the depth stream, keep only the RGB stream

        self.context = rs.context()
        self.devices = self.context.query_devices()

        found_desired_cam = False
        for dev in self.devices:
            serial_test = dev.get_info(rs.camera_info.serial_number)
            self.node.get_logger().info(f"Found device: {dev.get_info(rs.camera_info.name)}, serial = {serial_test}")
            if serial_test == self.serial_number:
                found_desired_cam = True
                self.node.get_logger().info("Found desired realsense !")
        
        if found_desired_cam == False:
            self.node.get_logger().error("Did not find desired camera !")
            raise RuntimeError("Did not find desired camera !")

        
        if len(self.devices) == 0:
            self.node.get_logger().error("No RealSense devices found!")
            raise RuntimeError("No RealSense devices found!")

    def get_depth_scale(self):
        depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        return depth_scale

    # A method to get the intrinsic camera matrix.
    def get_intrinsics(self):
        intrinsics = (
            self.profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        intrinsics = {
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "cx": intrinsics.ppx,
            "cy": intrinsics.ppy,
        }
        return intrinsics

    # A method to get the distortion coefficients.
    def get_coeffs(self):
        intrinsics = (
            self.profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        # Extract the distortion coefficients from the intrinsics object.
        return intrinsics.coeffs

    # A method to get the depth data from the RealSense camera.
    def get_depth(self):
        frameset = (
            self.pipe.wait_for_frames()
        )  # Wait for the next set of frames from the pipeline.
        aligned_frame = self.align.process(frameset) # TODO added
        depth_frame = (
            aligned_frame.get_depth_frame()
        ) 
        # depth_frame = (
        #     frameset.get_depth_frame()
        # )  # Get the depth frame from the frameset.
        depth = np.asanyarray(
            depth_frame.get_data()
        )  # Convert the depth frame to a NumPy array.

        return depth
    
    def camera_params_callback(self, request, response):
        intrinsics = self.camera.get_intrinsics()
        depth_scale = self.camera.get_depth_scale()
        distortion_coefficients = self.camera.get_coeffs()
        response.depth_scale = depth_scale
        response.fx = intrinsics["fx"]
        response.fy = intrinsics["fy"]
        response.cx = intrinsics["cx"]
        response.cy = intrinsics["cy"]
        response.distortion_coefficients = distortion_coefficients
        self.get_logger().info(
            f"Provided intrinsics: fx={response.fx}, fy={response.fy}, cx={response.cx}, cy={response.cy}"
        )
        self.get_logger().info(
            f"Provided distortion coefficients: {response.distortion_coefficients}"
        )
        self.get_logger().info(
            f"Provided depth scale: {response.depth_scale}"
        )
        return response

    # A method to get the color image from the RealSense camera.
    def get_image(self):
        frameset = self.pipe.wait_for_frames()

        color_frame = (
            frameset.get_color_frame()
        )  # Get the color frame from the frameset.

        color = np.asanyarray(
            color_frame.get_data()
        )  # Convert the color frame to a NumPy array.

        return color

    def get_rgbd(self):
        frameset = self.pipe.wait_for_frames()
        color_frame = np.asanyarray((frameset.get_color_frame().get_data()))
        depth_frame = np.asanyarray(frameset.get_depth_frame().get_data())
        # depth_frame = self.get_depth()
        return color_frame, depth_frame
    
    def get_rgb(self):
        # frameset = self.pipe.wait_for_frames()
        # color_frame = np.asanyarray((frameset.get_color_frame().get_data()))
        # return color_frame   
        try:
            frameset = self.pipe.wait_for_frames()
            color_frame = frameset.get_color_frame()
            if not color_frame:
                raise RuntimeError("No color frame received")
            color = np.asanyarray(color_frame.get_data())
            return color
        except RuntimeError as e:
            self.node.get_logger().error(f"Error getting RGB frame: {e}")
            return None
   

    def publish_feeds(self, camera_id):        
        self.node.get_logger().info("STARTING TO PUBLISH RGB")

    
        try:
            self.config.enable_stream(rs.stream.color, self.x, self.y, rs.format.bgr8, self.fps)
            self.profile = self.pipe.start(self.config)
            time.sleep(2)
            self.align = rs.align(rs.stream.color)
        except Exception as e:
            self.node.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            return

     
        while not self.node.stopped:
            frame = self.get_rgb()
            if frame is None:
                self.node.get_logger().warn("Skipping frame initialization due to missing RGB frame")
        
            image_idx = 0
            try:
                while True:
                    #self.node.get_logger().info("Capturing " + str(image_idx) + " | time: " + str(time.time()))
                    frame = self.get_rgb()

                    if self.node.stopped:
                        break    
                        
                    compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
                    self.node.cam_pubs.publish(compressed_image)
                    image_idx += 1
            finally:
                self.node.get_logger().info("Stopped RealSense pipeline")
        
        self.pipe.stop()        

            