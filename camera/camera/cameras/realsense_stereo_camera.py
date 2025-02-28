import pyrealsense2 as rs
import numpy as np
import cv2 as cv
from ..interfaces.stereo_camera_interface import StereoCameraInterface
from cv_bridge import CvBridge
from custom_msg.srv import CameraParams
from custom_msg.msg import CompressedRGBD
from std_msgs.msg import Int8
from sensor_msgs.msg import Image

class RealSenseStereoCamera(StereoCameraInterface):
    def __init__(self, node):
        # the configuration of each camera will be implemented on the CS side
        # need to hardcode now in the code the config

        self.node = node

        # will be placed inside the launch file
        self.fps = 15
        self.x = 1280
        self.y = 720

        self.serial_number = self.node.devrule 
        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value
        self.node.declare_parameter("depth_req", self.node.default)
        self.depth_request = self.node.get_parameter("depth_req").get_parameter_value().string_value

        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.
        self.config = rs.config()  # Disable the depth stream, keep only the RGB stream

        self.context = rs.context()
        self.devices = self.context.query_devices()
        
        # Service to retrieve the parameters of the camera
        self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)
        
        # Publisher for RGBD + service to activate the depth mode
        self.color_depth_pub = self.node.create_publisher(CompressedRGBD, self.node.publisher_topic + "_plus_depth", 1)
        self.depth_change = self.node.create_subscription(Int8, self.depth_request, self.depth_callback, 1)
        self.depth_mode = 0
        
        ## -----------------------------------------------------
        ## Check that the camera is found with the serial number
        if len(self.devices) == 0:
            self.node.get_logger().error("No RealSense devices found!")
            raise RuntimeError("No RealSense devices found!")

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
        ## -----------------------------------------------------------

    # Depth mode: 0 => Off, 1 => On
    def depth_callback(self, msg):
        self.depth_mode = msg.data


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
        
        depth = np.asanyarray(
            depth_frame.get_data()
        )  # Convert the depth frame to a NumPy array.

        return depth
    
    def camera_params_callback(self, request, response):
        intrinsics = self.get_intrinsics()
        depth_scale = self.get_depth_scale()
        distortion_coefficients = self.get_coeffs()
        response.depth_scale = depth_scale
        response.fx = float(intrinsics["fx"])
        response.fy = float(intrinsics["fy"])
        response.cx = float(intrinsics["cx"])
        response.cy = float(intrinsics["cy"])
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

        self.node.get_logger().info("STARTING TO PUBLISH RGB")
    
        try:
            self.config.enable_stream(rs.stream.color, self.x, self.y, rs.format.bgr8, self.fps)
            
            if self.depth_mode == 1:
                self.config.enable_stream(rs.stream.depth, self.x, self.y, rs.format.z16, self.fps)
            
            self.profile = self.pipe.start(self.config)
            self.align = rs.align(rs.stream.color)
        except Exception as e:
            self.node.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            return

        
        while True:
            if self.depth_mode == 0:
                frame = self.get_rgb()
                compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
                self.node.cam_pubs.publish(compressed_image)
            else:
                frame_color, frame_depth = self.get_rgbd()
                compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame_color)
                self.node.cam_pubs.publish(compressed_image)
                
                msg = Image()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = "camera_hd_0_depth_frame"
                msg.height = frame_depth.shape[0]
                msg.width = frame_depth.shape[1]
                msg.encoding = "mono16"  # Encoding for uint16 depth images
                msg.is_bigendian = False
                msg.step = msg.width * 2  # 2 bytes per pixel
                msg.data = frame_depth.tobytes()
                
                # Not really optimized because we send on 2 topics, one only RGB and the other one
                # RGBD. But because we have not experienced issues, we keep with this architecture
                self.node.get_logger().info("depthhhh")
                full_msg = CompressedRGBD()
                full_msg.color = compressed_image
                full_msg.depth = msg
                self.color_depth_pub.publish(full_msg)

            if self.node.stopped:
                self.pipe.stop()
                break
            