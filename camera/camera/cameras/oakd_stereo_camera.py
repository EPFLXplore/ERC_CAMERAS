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

rgbWeight = 0.4
depthWeight = 0.6
alpha = None

def updateBlendWeights(percent_rgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image
    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percent_rgb)/100.0
    depthWeight = 1.0 - rgbWeigh

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
        
        self.pipeline = dai.Pipeline()
        
        # The disparity is computed at this resolution, then upscaled to RGB resolution
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.device = dai.Device()
        self.queueNames = []
        
        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.Camera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        disparityOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        self.queueNames.append("rgb")
        disparityOut.setStreamName("disp")
        self.queueNames.append("disp")

        # Properties
        rgbCamSocket = dai.CameraBoardSocket.CAM_A

        camRgb.setBoardSocket(rgbCamSocket)
        camRgb.setSize(self.node.x, self.node.y)
        camRgb.setFps(self.node.fps)

        # For now, RGB needs fixed focus to properly align with depth.
        # This value was used during calibration
        try:
            calibData = self.device.readCalibration2()
            lensPosition = calibData.getLensPosition(rgbCamSocket)
            if lensPosition:
                camRgb.initialControl.setManualFocus(lensPosition)
        except:
            raise
        
        left.setResolution(monoResolution)
        left.setCamera("left")
        left.setFps(5)
        right.setResolution(monoResolution)
        right.setCamera("right")
        right.setFps(5)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # LR-check is required for depth alignment
        self.stereo.setLeftRightCheck(True)
        self.stereo.setDepthAlign(rgbCamSocket)

        # Linking
        camRgb.video.link(rgbOut.input)
        left.out.link(self.stereo.left)
        right.out.link(self.stereo.right)
        self.stereo.disparity.link(disparityOut.input)

        camRgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)
        if alpha is not None:
            camRgb.setCalibrationAlpha(alpha)
            self.stereo.setAlphaScaling(alpha)
            
        self.device.startPipeline(self.pipeline)
        self.queueEvents = []
        
        self.depth_pubs = self.node.create_publisher(Image, self.depth_topic_string, qos_profile=self.node.qos_profile)

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
        rgb_frame = self.rgb_queue.tryGet()
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
    
    def publish_feeds(self, devrule=None):
        """Publish RGB and Depth feeds."""

        self.node.get_logger().info("STARTING TO PUBLISH RGB")
        
        frameRgb = None
        frameDisp = None
        encoded_image = None
        previous_time = 0
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]  # Lower quality to save bandwidth
        
        while not self.node.stopped:
        
            if self.depth_mode:
                queueEvents = self.device.getQueueEvents(("rgb", "disp"))
                
                latestPacket = {}
                latestPacket["rgb"] = None
                latestPacket["disp"] = None
                
                for queueName in queueEvents:
                    packets = self.device.getOutputQueue(queueName).tryGetAll()
                    if len(packets) > 0:
                        latestPacket[queueName] = packets[-1]
            
                if latestPacket["rgb"] is not None:
                    frameRgb = latestPacket["rgb"].getCvFrame()
                    
                    success, encoded_image = cv2.imencode('.jpg', frameRgb, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue

                if latestPacket["disp"] is not None:
                    frameDisp = latestPacket["disp"].getFrame()
                    maxDisparity = self.stereo.initialConfig.getMaxDisparity()
                    # Optional, extend range 0..95 -> 0..255, for a better visualisation
                    if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
                    # Optional, apply false colorization
                    if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_HOT)
                    frameDisp = np.ascontiguousarray(frameDisp)
            
                if encoded_image is not None and frameDisp is not None:
                    
                    # Convert encoded bytes to ROS-compressed message
                    compressed_msg = CompressedImage()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)
                    
                    # Need to have both frames in BGR format before blending
                    if len(frameDisp.shape) < 3:
                        frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)
                     
                    # get a portion of the depth for bandwidth improvements
                    depth_img_normalized = cv2.normalize(frameDisp, None, 0, 255, cv2.NORM_MINMAX)[300:980, 100:620]

                    msg = Image()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.height = depth_img_normalized.shape[0]
                    msg.width = depth_img_normalized.shape[1]
                    msg.encoding = "mono16"  # Encoding for uint16 depth images
                    msg.is_bigendian = False
                    msg.step = msg.width * 2  # 2 bytes per pixel
                    msg.data = depth_img_normalized.tobytes()
                    self.depth_pubs.publish(msg)  
                    
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data) + len(msg.data))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)
                                        
                    frameRgb = None
                    frameDisp = None        
                
                
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
                    
                    success, encoded_image = cv2.imencode('.jpg', frameRgb, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue
                    
                if encoded_image is not None:
                    
                    # Convert encoded bytes to ROS-compressed message
                    compressed_msg = CompressedImage()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)
                    
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)
                    
                    frameRgb = None