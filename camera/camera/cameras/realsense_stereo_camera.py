
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
        # the configuration of each camera will be implemented on the CS side
        # need to hardcode now in the code the config

        self.node = node

        self.fps = 15
        self.x = 1280
        self.y = 720

        self.serial_number = self.node.devrule  #we do a bit of cheating

        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.
        self.config = rs.config()  # Disable the depth stream, keep only the RGB stream

        self.context = rs.context()
        self.devices = self.context.query_devices()

        
        if len(self.devices) == 0:
            self.node.get_logger().error("No RealSense devices found!")
            raise RuntimeError("No RealSense devices found!")

        found = False
        for dev in self.devices:
            serial = dev.get_info(rs.camera_info.serial_number)
            self.node.get_logger().info(f"Device serial: {serial}")
            if serial == self.serial_number:
                found = True
                break
        if not found:
            self.node.get_logger().error(f"Device with serial {self.serial_number} not found!")
            raise RuntimeError(f"Device with serial {self.serial_number} not found!")
        

        #for dev in self.devices:
        #    dev.hardware_reset() #fix the timeout problem
        #    self.node.get_logger().info(f"Resetting RealSense device")
        #    time.sleep(7)
        
        #self.devices = self.context.query_devices()

        #if len(self.devices) == 0:
        #    self.node.get_logger().error("No RealSense devices found after reset!")
        #    raise RuntimeError("No RealSense devices found after reset!")


        self.bridge = CvBridge()
        self.config.enable_device(self.serial_number)
        self.node.get_logger().info("enabled realsense d415 device")


    #     self.mode = "RGB"

    # def set_mode(self, mode) :
    #     if mode in ["RGB", "RGB-D"]:
    #         self.mode = mode
    #         self.node.get_logger().info(f"Camera mode set to {self.mode}")
    #     else:
    #         self.node.get_logger().warn("Invalid mode selected")        

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
   

    def publish_feeds(self, camera_id):#camera_id added just so it works with other cameras
        # if camera_id == "/dev/realsense_aruco_0":
        #     target_serial = "102122061110"  # Replace with your actual serial number
        #     self.config.enable_device(target_serial)
        #     self.node.get_logger().info(f"Detected RealSense D415 camera with serial number: {target_serial}")

        # Check if any RealSense devices are connected

        
        self.node.get_logger().info("STARTING TO PUBLISH RGB") #rgb-feed
    
        #self.config.enable_stream(rs.stream.depth, self.x, self.y, rs.format.z16, self.fps)
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
                    self.node.get_logger().info("Capturing " + str(image_idx) + " | time: " + str(time.time()))
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
    
        

            