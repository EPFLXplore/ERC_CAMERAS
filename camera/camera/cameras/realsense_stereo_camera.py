import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import yaml
from ..interfaces.stereo_camera_interface import StereoCameraInterface


class RealSenseStereoCamera(StereoCameraInterface):
    def __init__(self, subsystem):

        self.config_path = f'../../../custom_msg/config/{subsystem}.yaml'
        self.config = self.load_config(self.config_path)

        # the configuration of each camera will be implemented on the CS side
        # need to hardcode now in the code the config
        self.fps = 30
        self.x = 0
        self.y = 0

        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.

        # TODO: Add a configuration object for the pipeline.
        config = rs.config()
        # config.enable_device('123622270224')

        config.enable_stream(rs.stream.depth, self.x, self.y, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.x, self.y, rs.format.bgr8, self.fps)

        # Start streaming from file
        self.profile = self.pipe.start(config)

        self.align = rs.align(rs.stream.color) #TODO added
        

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
