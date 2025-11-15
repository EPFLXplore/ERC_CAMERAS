import depthai as dai
import cv2
import time
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage, Image
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from std_msgs.msg import Float32, Bool

class OakDStereoCamera():
    def __init__(self, node):
        #------------------- Defining parameters -------------------
        self.node = node
        self.serial_number = self.node.devrule 
        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value
        self.node.declare_parameter("depth_req", self.node.default)
        self.depth_request = self.node.get_parameter("depth_req").get_parameter_value().string_value
        self.node.declare_parameter("depth", self.node.default)
        self.depth_topic_string = self.node.get_parameter("depth").get_parameter_value().string_value
        self.node.declare_parameter("fps_depth", 5)
        self.fps_depth = self.node.get_parameter("fps_depth").get_parameter_value().integer_value

        self.depth_change = self.node.create_service(SetBool, self.depth_request, self.depth_callback)
        self.depth_mode = False
        
        self.node.declare_parameter("state_depth", "")
        self.state_depth_topic = self.node.get_parameter("state_depth").get_parameter_value().string_value

        self.node.declare_parameter("flip_camera", False)
        self.flip_camera = self.node.get_parameter("flip_camera").get_parameter_value().bool_value

        self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)
        
        self.publish_gray = self.node.create_publisher(CompressedImage, "/ROVER/test_gray", qos_profile=self.node.qos_profile, callback_group=self.node.callback_group)


        #------------------- Device and pipeline init -------------------
        self.pipeline = dai.Pipeline()
        self.device = dai.Device()
        self.queueNames = []

        #Sources and Outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        depthOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        depthOut.setStreamName("depth")
        self.queueNames.extend(["rgb", "depth"])

        #Camera parameters
        rgbCamSocket = dai.CameraBoardSocket.CAM_A
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_480_P
        ### ------------ For HDS --------------
        #For Nav it should be the other way around
        subpixel = False
        extended_disparity = True
        ### -----------------------------------

        
        #Properties
        ## RGB Camera
        camRgb.setBoardSocket(rgbCamSocket)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P) #To change depending on needs (max, 12_MP = 4056x3040)
        camRgb.setFps(self.node.fps)

        try:
            calibData = self.device.readCalibration2()
            lensPosition = calibData.getLensPosition(rgbCamSocket)
            if lensPosition:
                camRgb.initialControl.setManualFocus(lensPosition)
        except:
            raise

        ## Mono Cameras
        left.setResolution(monoResolution)
        left.setCamera("left")
        left.setFps(self.fps_depth)
        right.setResolution(monoResolution)
        right.setCamera("right")
        right.setFps(self.fps_depth)

        ##Stereo Properties
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        
        self.stereo.setDepthAlign(rgbCamSocket) #Align depth to RGB camera
        self.stereo.setOutputSize(640, 480) #Prevents automatique rescaling to RGB size 

        self.stereo.setRectification(True)
        self.stereo.setLeftRightCheck(True) #removed incorrectly calculated dispariy pixels due to occlusions at object borders
        self.stereo.setExtendedDisparity(extended_disparity) #allow detecting closer distance objects (halves min distance)
        self.stereo.setSubpixel(subpixel) #better precision for longer distances (but incompatible with extended disparity)

        #linking
        camRgb.isp.link(rgbOut.input)
        left.out.link(self.stereo.left)
        right.out.link(self.stereo.right)
        self.stereo.depth.link(depthOut.input)

        #------------------- Start pipeline -------------------
        self.device.startPipeline(self.pipeline)
        self.queueEvents = []
        try:
            self.device.setIrLaserDotProjectorIntensity(0) #Only useful indoors and if Oak-D pro is used
        except:
            self.node.get_logger().warn("No laser projector found on device.")
        
        self.state_depth = self.node.create_publisher(Bool, self.state_depth_topic, 1)
        self.depth_pubs = self.node.create_publisher(Image, self.depth_topic_string, qos_profile=self.node.qos_profile)
        #------------------- End init -------------------

    def depth_callback(self, request, response):
        self.depth_mode = request.data
        response.success = True
        self.state_depth.publish(Bool(data=self.depth_mode))
        return response

    def camera_params_callback(self, request, response):
        
        calib = self.device.readCalibration()
        intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, (1920, 1080)) #Resolution is for perception nodes
        response.depth_scale = 0.001
        distortion_coefficients = calib.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
        response.fx = float(intrinsics[0][0])
        response.fy = float(intrinsics[1][1])
        response.cx = float(intrinsics[0][2])
        response.cy = float(intrinsics[1][2])
        response.distortion_coefficients = distortion_coefficients

        return response

    def publish_feeds(self, devrule=None):
        self.node.get_logger().info("STARTING TO PUBLISH RGB!!")
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 25]
        previous_time = 0

        while not self.node.stopped:
            rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            rgb_packet = rgb_queue.tryGet()

            # Create and publish RGB compressed image
            if rgb_packet is not None:
                    frameRgb = rgb_packet.getCvFrame()
                    frameRgb = cv2.rotate(frameRgb, cv2.ROTATE_180) if self.flip_camera else frameRgb
                    success, encoded_image = cv2.imencode('.jpg', frameRgb, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)
        
             # Create and publish Depth encoded image
            if self.depth_mode:      
                depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                depth_packet = depth_queue.tryGet()                
                
                if depth_packet is not None:
                    depth_frame = depth_packet.getFrame()
                    depth_frame = np.ascontiguousarray(depth_frame)

                    msg = Image()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.height = depth_frame.shape[0]
                    msg.width = depth_frame.shape[1]
                    msg.encoding = "16UC1"
                    msg.is_bigendian = False
                    msg.step = msg.width * 2
                    msg.data = depth_frame.tobytes()
                    self.depth_pubs.publish(msg)

            if rgb_packet is not None:
                current_time = time.time()
                if self.depth_mode and depth_packet is not None:
                    total_bytes = len(encoded_image.tobytes()) + len(depth_frame.tobytes())
                    # total_bytes = len(encoded_image.tobytes())
                else :
                    total_bytes = len(encoded_image.tobytes())
               
                bw = self.node.calculate_bandwidth(current_time, previous_time, total_bytes)
                previous_time = current_time
                self.node.cam_bw.publish(bw)


                    # gray = cv2.cvtColor(frameRgb, cv2.COLOR_BGR2GRAY)
                    # compressed_msg = CompressedImage()
                    # compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    # compressed_msg.format = "jpeg"
                    # compressed_msg.data = gray.tobytes()
                    # self.publish_gray.publish(compressed_msg)
               