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
        self.node.declare_parameter("fps_depth", 5)
        self.fps_depth = self.node.get_parameter("fps_depth").get_parameter_value().integer_value

        self.depth_change = self.node.create_service(SetBool, self.depth_request, self.depth_callback)
        self.depth_mode = False

        self.node.declare_parameter("flip_camera", False)
        self.flip_camera = self.node.get_parameter("flip_camera").get_parameter_value().bool_value

        self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)

        self.pipeline = dai.Pipeline()
        self.device = dai.Device()
        self.queueNames = []

        camRgb = self.pipeline.create(dai.node.ColorCamera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        depthOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        depthOut.setStreamName("depth")
        self.queueNames.extend(["rgb", "depth"])

        rgbCamSocket = dai.CameraBoardSocket.CAM_A
        camRgb.setBoardSocket(rgbCamSocket)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        # for aligned?
        #camRgb.setIspScale(2, 3)
        camRgb.setFps(self.node.fps)

        try:
            calibData = self.device.readCalibration2()
            lensPosition = calibData.getLensPosition(rgbCamSocket)
            if lensPosition:
                camRgb.initialControl.setManualFocus(lensPosition)
        except:
            raise

        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        left.setCamera("left")
        left.setFps(self.fps_depth)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        right.setCamera("right")
        right.setFps(self.fps_depth)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        self.stereo.setLeftRightCheck(True)
        #self.stereo.setDepthAlign(rgbCamSocket)

        camRgb.isp.link(rgbOut.input)
        left.out.link(self.stereo.left)
        right.out.link(self.stereo.right)
        self.stereo.depth.link(depthOut.input)

        self.device.startPipeline(self.pipeline)
        self.queueEvents = []
        self.depth_pubs = self.node.create_publisher(Image, self.depth_topic_string, qos_profile=self.node.qos_profile)

    def depth_callback(self, request, response):
        self.depth_mode = request.data
        response.success = True
        return response

    def camera_params_callback(self, request, response):
        intrinsics = self.get_intrinsics()
        distortion_coefficients = self.get_coeffs()
        response.depth_scale = 0.001
        response.fx = float(intrinsics[0][0])
        response.fy = float(intrinsics[1][1])
        response.cx = float(intrinsics[0][2])
        response.cy = float(intrinsics[1][2])
        response.distortion_coefficients = distortion_coefficients
        return response

    def get_rgb(self):
        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is not None:
            frame = rgb_frame.getCvFrame()
            return cv2.rotate(frame, cv2.ROTATE_180) if self.flip_camera else frame

    def get_rgbd(self):
        rgb_frame = self.get_rgb()
        depth_frame = self.get_depth()
        if rgb_frame is None or depth_frame is None:
            self.node.get_logger().warn("RGB or depth frame missing.")
            return rgb_frame, depth_frame
        if self.flip_camera:
            rgb_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
            depth_frame = cv2.rotate(depth_frame, cv2.ROTATE_180)
        return rgb_frame, depth_frame

    def get_depth(self):
        depth_queue = self.device.getOutputQueue("depth")
        depth_packet = depth_queue.tryGet()
        if depth_packet is not None:
            frame = depth_packet.getFrame()
            return cv2.rotate(frame, cv2.ROTATE_180) if self.flip_camera else frame

    def get_intrinsics(self):
        calib_data = self.device.readCalibration()
        return calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1280, 720)

    def get_coeffs(self):
        return self.device.readCalibration().getDistortionCoefficients(dai.CameraBoardSocket.RGB)

    def publish_feeds(self, devrule=None):
        self.node.get_logger().info("STARTING TO PUBLISH RGB")
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 25]
        previous_time = 0

        while not self.node.stopped:
            if self.depth_mode:
                rgb_queue = self.device.getOutputQueue("rgb")
                depth_queue = self.device.getOutputQueue("depth")
                # self.node.get_logger().info("DEPTH MODE IS ON")

                rgb_packet = rgb_queue.tryGet()
                depth_packet = depth_queue.tryGet()

                if rgb_packet is not None:
                    # self.node.get_logger().info("--------------RGB NOT NONE --------------")
                    frameRgb = rgb_packet.getCvFrame()
                    success, encoded_image = cv2.imencode('.jpg', frameRgb, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue

                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)

                if depth_packet is not None:
                    # self.node.get_logger().info("--------------DEPTH NOT NONE --------------")
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

                if rgb_packet is not None and depth_packet is not None:
                    current_time = time.time()
                    total_bytes = len(encoded_image.tobytes()) + len(depth_frame.tobytes())
                    # total_bytes = len(encoded_image.tobytes())
                    bw = self.node.calculate_bandwidth(current_time, previous_time, total_bytes)
                    previous_time = current_time
                    self.node.cam_bw.publish(bw)

            else:
                # self.node.get_logger().info("DEPTH MODE IS OFFFFF")
                rgb_queue = self.device.getOutputQueue("rgb")
                rgb_packet = rgb_queue.tryGet()
                if rgb_packet is not None:
                    frameRgb = rgb_packet.getCvFrame()
                    success, encoded_image = cv2.imencode('.jpg', frameRgb, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue

                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)
                    
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                    previous_time = current_time
                    self.node.cam_bw.publish(bw)



