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
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        disparityOut = self.pipeline.create(dai.node.XLinkOut)
        disparityOut.setStreamName("disp")

        rgbOut.setStreamName("rgb")
        self.queueNames.append("rgb")
        self.queueNames.append("disp")


        # try:
        #     calibData = self.device.readCalibration2()
        #     lensPosition = calibData.getLensPosition(rgbCamSocket)
        #     if lensPositionp:
        #         camRgb.initialControl.setManualFocus(lensPosition)
        # except:
        #     self.node.get_logger().info("Could NOT set fixed focus to align depth")
        #     raise

        #stereo parameters
        leftSocket = dai.CameraBoardSocket.CAM_B
        rightSocket = dai.CameraBoardSocket.CAM_C
        monoLeft.setBoardSocket(leftSocket)
        monoRight.setBoardSocket(rightSocket)

        monoLeft.out.link(self.stereo.left)
        monoRight.out.link(self.stereo.right)
                
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

        monoLeft.setResolution(monoResolution)
        monoLeft.setCamera("left")
        monoLeft.setFps(5)
        monoRight.setResolution(monoResolution)
        monoRight.setCamera("right")
        monoRight.setFps(5)
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        
        self.stereo.setLeftRightCheck(True) #removed incorrectly calculated dispariy pixels due to occlusions at object borders
        self.stereo.setRectification(True)
        self.stereo.setExtendedDisparity(True) #allow detecting closer distance objects for the given baseline.
        self.stereo.setSubpixel(True) #better precision for longer distances

        self.stereo.disparity.link(disparityOut.input)

        #camRgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION) #?????? does not work if we use ColorCamera !


        # RGB Properties
        rgbCamSocket = dai.CameraBoardSocket.CAM_A
        camRgb.setBoardSocket(rgbCamSocket)
        # camRgb.setVideoSize(self.node.x, self.node.y)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        camRgb.setFps(self.node.fps)
        enc.setDefaultProfilePreset(15, dai.VideoEncoderProperties.Profile.MJPEG)
        enc.setLossless(False)
        enc.setQuality(30)
        enc.setNumFramesPool(2)
        enc.setFrameRate(self.node.fps)

        # OAKD Hardware encoder for rgb jpeg stream
        camRgb.video.link(enc.input)
        enc.bitstream.link(rgbOut.input)# already-compressed JPEG

        # ------------------ end RGB ------------------
        self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)  #10Gbps USB3.2 gen2
        self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue("disp", maxSize=2, blocking=False)

        self.queueEvents = []

        self.depth_pub = self.node.create_publisher(Image, self.depth_topic_string, qos_profile=self.node.qos_profile)
        

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
        depth_frame = self.depthQueue.tryGet()
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

        self.node.get_logger().info("STARTING TO PUBLISH FRAMES")
        #time.sleep(1)
        frameRgb = None
        frameDisp = None
        encoded_image = None
        previous_time = 0
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]  # Lower quality to save bandwidth

########################################################

        while not self.node.stopped:
        
            if self.depth_mode:
                queueEvents = self.device.getQueueEvents(("rgb", "disp"))
                
                latestPacket = {}
                latestPacket["rgb"] = None
                latestPacket["disp"] = None
                
                for queueName in queueEvents:
                    packets = self.device.getOutputQueue(queueName, maxSize=2, blocking=False).tryGetAll()
                    if len(packets) > 0:
                        latestPacket[queueName] = packets[-1]
            
                if latestPacket["rgb"] is not None:
                    frameRgb = latestPacket["rgb"].getCvFrame()

                if latestPacket["disp"] is not None:
                    frameDisp = latestPacket["disp"].getFrame()
                    #maxDisparity = self.stereo.initialConfig.getMaxDisparity()
                    # Optional, extend range 0..95 -> 0..255, for a better visualisation
                    #if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
                    # Optional, apply false colorization
                    #if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_HOT)
                    frameDisp = np.ascontiguousarray(frameDisp)
            
                if frameRgb is not None and frameDisp is not None:
                    
                    # Convert encoded bytes to ROS-compressed message
                    compressed_msg = CompressedImage()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = bytearray(frameRgb)
                    self.node.cam_pubs.publish(compressed_msg)
                    
                    # Need to have both frames in BGR format before blending
                    if len(frameDisp.shape) < 3:
                        frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)
                     
                    # get a portion of the depth for bandwidth improvements
                    depth_img_normalized = cv2.normalize(frameDisp, None, 0, 255, cv2.NORM_MINMAX)
                    msg = Image()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.height = depth_img_normalized.shape[0]
                    msg.width = depth_img_normalized.shape[1]
                    msg.encoding = "mono16"  # Encoding for uint16 depth images
                    msg.is_bigendian = False
                    msg.step = msg.width * 2  # 2 bytes per pixel
                    msg.data = depth_img_normalized.tobytes()
                    self.depth_pub.publish(msg)  
                    
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data) + len(msg.data))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)
                    
                    frameRgb = None
                    frameDisp = None        

                    time.sleep(1/self.node.fps)

            else:
                self.queueEvents = self.device.getQueueEvents(("rgb"))
                                
                latestPacket = {}
                latestPacket["rgb"] = None
                
                for queueName in self.queueEvents:
                    packets = self.device.getOutputQueue(queueName).tryGetAll()
                    if len(packets) > 0:
                        latestPacket[queueName] = packets[-1]
                
                if latestPacket["rgb"] is not None:
                    frameRgb = latestPacket["rgb"].getCvFrame()
                    
                if frameRgb is not None:
                    
                    # Convert encoded bytes to ROS-compressed message
                    compressed_msg = CompressedImage()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = bytearray(frameRgb)
                    self.node.cam_pubs.publish(compressed_msg)
                    
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)
                    
                    frameRgb = None



#########################################################
        #OLD RGB OFFLOADING CODE FROM NAV, WORKS
        
        # while not self.node.stopped:
        #     rgb_pkt = self.rgbQueue.tryGet() #non blocking
        #     if rgb_pkt:
        #         compressed = rgb_pkt.getData()
        #         ros_msg = CompressedImage()
        #         ros_msg.format = "jpeg"
        #         ros_msg.data = bytearray(compressed)
        #         self.node.cam_pubs.publish(ros_msg)
        #         current_time = time.time()
        #         bw = self.node.calculate_bandwidth(current_time, previous_time, len(ros_msg.data))
        #         previous_time = current_time 
        #         self.node.cam_bw.publish(bw)

        #         time.sleep(1/(self.node.fps))
        #     else:
        #         time.sleep(0.001)
########################################################