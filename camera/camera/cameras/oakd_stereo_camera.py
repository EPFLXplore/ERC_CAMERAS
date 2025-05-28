import depthai as dai
import cv2
import time
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image


class OakDStereoCamera():
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        self.serial_number = self.node.devrule 
        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value
        self.node.declare_parameter("depth_req", self.node.default)
        self.depth_request = self.node.get_parameter("depth_req").get_parameter_value().string_value
        self.node.declare_parameter("depth", self.node.default)
        self.depth_topic_string = self.node.get_parameter("depth").get_parameter_value().string_value
        
        self.depth_change = self.node.create_service(SetBool, self.depth_request, self.depth_callback)
        self.depth_mode = False

        self.node.declare_parameter("flip_camera", False)
        self.flip_camera = self.node.get_parameter("flip_camera").get_parameter_value().bool_value
        
        # Service to retrieve the parameters of the camera
        self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)
        
        #publishing timer
        # self.timer_cam = self.node.create_timer(
        #     1/15.0,
        #     self._on_timer
        # )

        self.pipeline = dai.Pipeline()
        
        
        # Create pipeline
        #self.pipeline = dai.Pipeline()

        self.queueNames = []
        
        # ----------------RGB-------------------
        # Define sources and output
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        enc = self.pipeline.create(dai.node.VideoEncoder)
        rgbOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        self.queueNames.append("rgb")

        
        # RGB Properties
        rgbCamSocket = dai.CameraBoardSocket.CAM_A
        camRgb.setBoardSocket(rgbCamSocket)
        # camRgb.setVideoSize(self.node.x, self.node.y)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        camRgb.setFps(self.node.fps)
        enc.setDefaultProfilePreset(15, dai.VideoEncoderProperties.Profile.H264_MAIN)

        # OAKD Hardware encoder for rgb jpeg stream
        camRgb.video.link(enc.input)
        enc.bitstream.link(rgbOut.input)# already-compressed JPEG

        # ------------------ end RGB ------------------
        self.device = dai.Device(self.pipeline)
        self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)

        self.queueEvents = []
        

    # Depth mode: 0 => Off, 1 => On
    def depth_callback(self, request, response):
        self.depth_mode = request.data
        response.success = True
        return response

    def camera_params_callback(self, request, response):
        intrinsics = self.get_intrinsics()
        distortion_coefficients = self.get_coeffs()
        response.depth_scale = 0.1 # to go from mm to cm
        response.fx = float(intrinsics[0][0]) # fx
        response.fy = float(intrinsics[1][1]) # fy
        response.cx = float(intrinsics[0][2]) # cx
        response.cy = float(intrinsics[1][2]) # cy
        response.distortion_coefficients = distortion_coefficients
       
        return response

    def get_rgb(self):
        """Retrieve an RGB frame from the RGB queue."""
        rgb_frame = self.rgbQueue.tryGet()
        if rgb_frame is not None:
        
            if self.flip_camera:
                rotated_frame = cv2.rotate(rgb_frame.getCvFrame(), cv2.ROTATE_180)
                return rotated_frame
            else:
                return rgb_frame.getCvFrame()

    def get_rgbd(self):
        """Retrieve both RGB and Depth frames."""
        rgb_frame = self.get_rgb()
        depth_frame = self.get_depth()
        
        
        if rgb_frame is None and depth_frame is not None:
            self.node.get_logger().warn("No RGB frame received.")
            return None, depth_frame
        
        if depth_frame is None and rgb_frame is not None:
            self.node.get_logger().warn("No depth frame received.")
            return rgb_frame, None
        
        if depth_frame is None and rgb_frame is None:
            self.node.get_logger().warn("No depth and rgb frame received.")
            return None, None
        
        if self.flip_camera:
            rotated_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
            rotated_depth = cv2.rotate(depth_frame, cv2.ROTATE_180)
            return rotated_frame, rotated_depth
        else:
            return rgb_frame, depth_frame

    def get_depth(self):
        """Retrieve a Depth frame from the Depth queue."""
        depth_frame = self.depth_queue.get()
        if depth_frame is not None:
        
            depth_image = depth_frame.getFrame()  # NumPy array (dtype=uint16, shape=H x W)
            if self.flip_camera:
                rotated_depth = cv2.rotate(depth_image, cv2.ROTATE_180)
                return rotated_depth
            else:
                return depth_image


    def get_intrinsics(self):
        calib_data = self.device.readCalibration()
        intrinsics = calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB)
        return intrinsics

    def get_coeffs(self):
        calib_data = self.device.readCalibration()
        return calib_data.getDistortionCoefficients(dai.CameraBoardSocket.RGB)

    # def _on_timer(self):
    #     if not self.node.stopped:
    #         rgb_pkt = self.rgbQueue.tryGet() #blocking
    #         if rgb_pkt:
    #             compressed = rgb_pkt.getData()
    #             ros_msg = CompressedImage()
    #             ros_msg.format = "jpeg"
    #             ros_msg.data = bytearray(compressed)
    #             self.node.cam_pubs.publish(ros_msg)
    #         else:
    #             return

    def publish_feeds(self, devrule=None):
        """Publish RGB and Depth feeds."""

        self.node.get_logger().info("STARTING TO PUBLISH RGB")
        #time.sleep(1)
        frameRgb = None
        encoded_image = None
        previous_time = 0
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]  # Lower quality to save bandwidth
        
        while not self.node.stopped:
            rgb_pkt = self.rgbQueue.tryGet() #blocking
            if rgb_pkt:
                compressed = rgb_pkt.getData()
                ros_msg = CompressedImage()
                ros_msg.format = "jpeg"
                ros_msg.data = bytearray(compressed)
                self.node.cam_pubs.publish(ros_msg)
                time.sleep(1/15)
            else:
                time.sleep(0.001)



                # self.queueEvents = self.device.getQueueEvents(("rgb"))
                                
                # latestPacket = {}
                # latestPacket["rgb"] = None
                
                # for queueName in self.queueEvents:
                #     packets = self.device.getOutputQueue(queueName).tryGetAll()
                #     if len(packets) > 0:
                #         latestPacket[queueName] = packets[-1]
                
                # if latestPacket["rgb"] is not None:
                #     frameRgb = latestPacket["rgb"].getCvFrame()
                    
                #     success, encoded_image = cv2.imencode('.jpg', frameRgb, encode_param)
                #     if not success:
                #         self.node.get_logger().warn("Failed to compress RGB frame.")
                #         continue
                    
                # if encoded_image is not None:
                    
                #     # Convert encoded bytes to ROS-compressed message
                #     compressed_msg = CompressedImage()
                #     compressed_msg.format = "jpeg"
                #     compressed_msg.data = encoded_image.tobytes()
                #     self.node.cam_pubs.publish(compressed_msg)
                    
                #     current_time = time.time()
                #     bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                #     previous_time = current_time 
                #     self.node.cam_bw.publish(bw)
                    
                #     frameRgb = None