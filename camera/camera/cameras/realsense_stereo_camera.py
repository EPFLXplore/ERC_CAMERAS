import pyrealsense2 as rs
import numpy as np
import cv2 as cv
from ..interfaces.stereo_camera_interface import StereoCameraInterface
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from custom_msg.srv import CameraParams


class RealSenseStereoCamera(StereoCameraInterface):
    def __init__(self, node):
        
        self.node = node

        self.declare_parameter("fps", 0)
        self.declare_parameter("x", 0)
        self.declare_parameter("y", 0)
        self.declare_parameter("get_params", "")

        self.fps = self.get_parameter("fps").get_parameter_value().value
        self.x = self.get_parameter("x").get_parameter_value().value
        self.y = self.get_parameter("y").get_parameter_value().value
        self.topic_get_param = self.get_parameter("get_params").get_parameter_value().string_value

        self.get_params_camera = self.node.create_service(CameraParams, self.topic_get_param, callback_group=self.node.callback_group)

        self.serial_number = self.node.devrule  # we do a bit of cheating

        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.
        self.config = rs.config()  # Disable the depth stream, keep only the RGB stream

        self.context = rs.context()
        self.devices = self.context.query_devices()

        desired_found = False
        for cam in self.devices:
            self.node.get_logger().info(f"Found device : {cam.get_info(rs.camera_info.name)}, with serial nbr: {cam.get_info(rs.camera_info.serial_number)}")
            if cam.get_info(rs.camera_info.serial_number) == self.serial_number:
                desired_found = True

        self.bridge = CvBridge()
        self.config.enable_device(self.serial_number)
        if desired_found:
            self.node.get_logger().info(f"Enabled Realsense Camera with serial : {self.serial_number}")
        else:
            self.node.get_logger().info(f"Could NOT enable Realsense Camera with serial : {self.serial_number}")


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
        ) 
        aligned_frame = self.align.process(frameset)
        depth_frame = (
            aligned_frame.get_depth_frame()
        ) 
        
        depth = np.asanyarray(
            depth_frame.get_data()
        )

        return depth
    
    def realsense_params(self, request, response):

        intrinsics = self.camera.get_intrinsics()
        depth_scale = self.camera.get_depth_scale()
        distortion_coefficients = self.camera.get_coeffs()
        response.depth_scale = depth_scale
        response.fx = intrinsics["fx"]
        response.fy = intrinsics["fy"]
        response.cx = intrinsics["cx"]
        response.cy = intrinsics["cy"]
        response.distortion_coefficients = distortion_coefficients
        
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
    
        return color_frame, depth_frame
    
    def get_rgb(self):
        
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

        self.node.get_logger().info("STARTING TO PUBLISH RGB") #rgb-feed
    
        try:
            self.config.enable_stream(rs.stream.color, self.x, self.y, rs.format.bgr8, self.fps)
            self.profile = self.pipe.start(self.config)
            self.align = rs.align(rs.stream.color)
        except Exception as e:
            self.node.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            return

     
        for i in range(10):
            frame = self.get_rgb()
            if frame is None:
                self.node.get_logger().warn("Skipping frame initialization due to missing RGB frame")
        
            image_idx = 0
            try:
                while True:
                    #self.node.get_logger().info("Capturing " + str(image_idx) + " | time: " + str(time.time()))
                    frame = self.get_rgb()

                    if self.node.stopped:
                        self.pipe.stop()
                        break    
                        
                    compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
                    self.node.cam_pubs.publish(compressed_image)
                    image_idx += 1
            finally:
                self.pipe.stop()
                self.node.get_logger().info("Stopped RealSense pipeline")
                #sleep(1/self.fps)  #realsense pipeline already takes care of fps



 
        # if self.mode == "RGB":

        #     self.config.enable_stream(rs.stream.color, self.x, self.y, rs.format.bgr8, self.fps)

        # elif self.mode == "RGB-D":
        
        #     self.config.enable_stream(rs.stream.depth, self.x, self.y, rs.format.z16, self.fps)
        #     self.config.enable_stream(rs.stream.color, self.x, self.y, rs.format.bgr8, self.fps)

        # # Start streaming from file
        # self.profile = self.pipe.start(self.config)
        # self.node.get_logger().info(f"Camera pipeline started in {self.mode} mode")

        # if self.mode == "RGB-D":
        #     self.align = rs.align(rs.stream.color)


        # for i in range(10):
        #     frame = self.get_rgb() if self.mode == "RGB" else self.get_rgbd()[0]

        # image_idx = 0
        # while not self.node.stopped:
        #     if self.mode == "RGB":
        #         frame = self.get_rgb()
        #         if frame is not None:
        #             compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
        #             self.node.cam_pubs.publish(compressed_image)
        #     elif self.mode == "RGB-D":
        #         color_frame, depth_frame = self.get_rgbd()
        #         if color_frame is not None:
        #             compressed_image = self.bridge.cv2_to_compressed_imgmsg(color_frame)
        #             self.node.cam_pubs.publish(compressed_image)

        #             # Optionally publish depth data on a separate topic
        #             depth_image = self.bridge.cv2_to_compressed_imgmsg(depth_frame)
        #             self.node.depth_pubs.publish(depth_image)

        #     #self.node.get_logger().info(f"Published frame {image_idx} in {self.mode} mode")
        #     image_idx += 1

        #     if self.node.stopped:
        #         self.pipe.stop()  
        #         break  
    
        

            