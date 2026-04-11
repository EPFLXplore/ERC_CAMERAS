import depthai as dai
import cv2
import time, os
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class OakDStereoCamera():
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()
        self.frameRgb = None
        
        self.node.declare_parameter("screenshot", self.node.default)
        self.screenshot_topic = self.node.get_parameter("screenshot").get_parameter_value().string_value
        self.path_images = self.screenshot_topic[5:]
        
        self.take_screenshot = self.node.create_service(SetBool, 
                            self.screenshot_topic, self.take_screenshot, callback_group=MutuallyExclusiveCallbackGroup())

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
        
        self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)

        self.pipeline = dai.Pipeline()
        self.queueNames = []
        
        # ----------------RGB-------------------
        # Define sources and output
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        disparityOut = self.pipeline.create(dai.node.XLinkOut)
        disparityOut.setStreamName("disp")

        rgbOut.setStreamName("rgb")
        self.queueNames.append("rgb")
        self.queueNames.append("disp")

        # stereo parameters
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
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setFps(self.node.fps)

        enc = self.pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(self.node.fps, dai.VideoEncoderProperties.Profile.MJPEG)
        enc.setLossless(False)
        enc.setQuality(80)
        enc.setNumFramesPool(2)
        camRgb.video.link(enc.input)
        enc.bitstream.link(rgbOut.input)

        # ------------------ end RGB ------------------
        self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)  #10Gbps USB3.2 gen2
        self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue("disp", maxSize=2, blocking=False)

        self.queueEvents = []

        self.depth_pub = self.node.create_publisher(Image, self.depth_topic_string, qos_profile=self.node.qos_profile)

        self.last_jpeg_data = None
     
    def take_screenshot(self, request, response):
        jpeg = getattr(self, 'last_jpeg_data', None)
        if jpeg is not None:
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                name = str(time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())) + '.png'
                image_dir = os.path.join(
                    '/home/xplore/dev_ws/photos_competition',
                    self.path_images
                )
                os.makedirs(image_dir, exist_ok=True)
                cv2.imwrite(os.path.join(image_dir, name), frame)
                response.success = True
            else:
                response.success = False
        else:
            response.success = False
        return response

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
        frameDisp = None
        jpeg_data = None
        previous_time = 0

        while not self.node.stopped:
          try:
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
                    raw = latestPacket["rgb"].getData()
                    jpeg_data = np.array(raw, dtype=np.uint8).tobytes()
                    self.last_jpeg_data = jpeg_data

                if latestPacket["disp"] is not None:
                    frameDisp = latestPacket["disp"].getFrame()
                    frameDisp = np.ascontiguousarray(frameDisp)

                if jpeg_data is not None and frameDisp is not None:
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = jpeg_data
                    self.node.cam_pubs.publish(compressed_msg)

                    if len(frameDisp.shape) < 3:
                        frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)

                    depth_img_normalized = cv2.normalize(frameDisp, None, 0, 255, cv2.NORM_MINMAX)
                    msg = Image()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.height = depth_img_normalized.shape[0]
                    msg.width = depth_img_normalized.shape[1]
                    msg.encoding = "mono16"
                    msg.is_bigendian = False
                    msg.step = msg.width * 2
                    msg.data = depth_img_normalized.tobytes()
                    self.depth_pub.publish(msg)

                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data) + len(msg.data))
                    previous_time = current_time
                    self.node.cam_bw.publish(bw)

                    jpeg_data = None
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
                    raw = latestPacket["rgb"].getData()
                    jpeg_data = np.array(raw, dtype=np.uint8).tobytes()
                    self.last_jpeg_data = jpeg_data

                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = jpeg_data
                    self.node.cam_pubs.publish(compressed_msg)

                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(jpeg_data))
                    previous_time = current_time
                    self.node.cam_bw.publish(bw)
          except Exception as e:
                self.node.get_logger().error(f"publish_feeds error: {e}", throttle_duration_sec=2.0)