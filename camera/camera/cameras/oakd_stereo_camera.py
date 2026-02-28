import depthai as dai
import cv2
import time
import numpy as np
import threading

from sensor_msgs.msg import CompressedImage, Image
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from std_msgs.msg import Float32, Bool


class OakDStereoCamera:
    def __init__(self, node):
        # ------------------- Defining parameters -------------------
        self.node = node
        self.serial_number = self.node.devrule

        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value

        self.node.declare_parameter("depth_req", self.node.default)
        self.depth_request = (
            self.node.get_parameter("depth_req").get_parameter_value().string_value
        )

        self.node.declare_parameter("depth", self.node.default)
        self.depth_topic_string = (
            self.node.get_parameter("depth").get_parameter_value().string_value
        )

        self.node.declare_parameter("depth_avg", self.node.default)
        self.depth_avg_topic_string = (
            self.node.get_parameter("depth_avg").get_parameter_value().string_value
        )

        self.node.declare_parameter("fps_depth", 5)
        self.fps_depth = (
            self.node.get_parameter("fps_depth").get_parameter_value().integer_value
        )

        self.depth_change = self.node.create_service(
            SetBool, self.depth_request, self.depth_callback
        )
        self.depth_mode = False

        self.node.declare_parameter("state_depth", "")
        self.state_depth_topic = (
            self.node.get_parameter("state_depth").get_parameter_value().string_value
        )

        self.node.declare_parameter("flip_camera", False)
        self.flip_camera = (
            self.node.get_parameter("flip_camera").get_parameter_value().bool_value
        )

        self.camera_info_service = self.node.create_service(
            CameraParams, self.info + self.serial_number, self.camera_params_callback
        )

        # ------------------- Publishers -------------------
        self.state_depth = self.node.create_publisher(Bool, self.state_depth_topic, 1)
        self.depth_pubs = self.node.create_publisher(
            Image, self.depth_topic_string, qos_profile=self.node.qos_profile
        )
        self.depth_avg_pubs = self.node.create_publisher(
            Image, self.depth_avg_topic_string, qos_profile=self.node.qos_profile
        )

        # ------------------- Device and pipeline init -------------------
        self.pipeline = dai.Pipeline()

        # Sources and Outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        depthOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        depthOut.setStreamName("depth")

        # Camera parameters
        self.rgbCamSocket = dai.CameraBoardSocket.CAM_A
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_480_P

        # ------------ For HDS --------------
        subpixel = False
        extended_disparity = True
        self.IRdot = 0  # Only useful indoors and if Oak-D pro is used (between 0 and 1)
        # -----------------------------------

        # Properties
        ## RGB Camera
        self.camRgb.setBoardSocket(self.rgbCamSocket)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        self.camRgb.setFps(self.node.fps)

        ## Mono Cameras
        left.setResolution(monoResolution)
        left.setCamera("left")
        left.setFps(self.fps_depth)

        right.setResolution(monoResolution)
        right.setCamera("right")
        right.setFps(self.fps_depth)

        ## Stereo Properties
        self.stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY
        )

        self.stereo.setDepthAlign(self.rgbCamSocket)
        if monoResolution == dai.MonoCameraProperties.SensorResolution.THE_480_P:
            self.stereo.setOutputSize(640, 480)
        elif monoResolution == dai.MonoCameraProperties.SensorResolution.THE_400_P:
            self.stereo.setOutputSize(640, 400)
        else:
            self.node.get_logger().error(
                "Resolution doesn't match predefined output size"
            )

        self.stereo.setRectification(True)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setExtendedDisparity(extended_disparity)
        self.stereo.setSubpixel(subpixel)

        # linking
        self.camRgb.isp.link(rgbOut.input)
        left.out.link(self.stereo.left)
        right.out.link(self.stereo.right)
        self.stereo.depth.link(depthOut.input)

        # ------------------- Runtime state -------------------
        self.device = None
        self.rgb_queue = None
        self.depth_queue = None
        self._dev_lock = threading.Lock()

        self.alpha = 0.2
        self.depth_frame = None

        # reconnect backoff
        self._reconnect_delay_s = 0.3

    def _open_device(self):
        """
        Open device + queues.
        IMPORTANT: stop forcing SUPER_PLUS; many systems will drop link after a few seconds.
        We'll prefer SUPER and only try SUPER_PLUS second (optional).
        """
        last_err = None

        # Prefer SUPER (USB3) for stability; try SUPER_PLUS as a second attempt.
        for speed in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
            try:
                dev = dai.Device(self.pipeline, maxUsbSpeed=speed)

                # Keep queues small to avoid backlog/latency and reduce host pressure
                self.rgb_queue = dev.getOutputQueue(
                    name="rgb", maxSize=1, blocking=False
                )
                self.depth_queue = dev.getOutputQueue(
                    name="depth", maxSize=1, blocking=False
                )

                # IR dot projector (best effort)
                try:
                    dev.setIrLaserDotProjectorIntensity(self.IRdot)
                except Exception:
                    self.node.get_logger().warn("No laser projector found on device.")

                self.device = dev
                self.node.get_logger().info(
                    f"DepthAI device opened with USB speed request: {speed}"
                )
                return

            except Exception as e:
                last_err = e
                try:
                    dev.close()
                except Exception:
                    pass

        raise RuntimeError(
            f"Failed to open DepthAI device / start pipeline: {last_err}"
        )

    def _reconnect(self, reason: str):
        # Close and reopen with a small delay
        self.node.get_logger().warn(f"DepthAI reconnecting ({reason}) ...")
        self.close()
        time.sleep(self._reconnect_delay_s)
        with self._dev_lock:
            if self.device is None:
                self._open_device()

    def close(self):
        with self._dev_lock:
            try:
                if self.device is not None:
                    self.device.close()
            except Exception as e:
                self.node.get_logger().warn(f"DepthAI close() failed: {e}")
            finally:
                self.device = None
                self.rgb_queue = None
                self.depth_queue = None
                self.depth_frame = None

    def stop(self):
        self.close()

    def depth_callback(self, request, response):
        self.depth_mode = request.data
        response.success = True
        self.state_depth.publish(Bool(data=self.depth_mode))
        return response

    def camera_params_callback(self, request, response):
        with self._dev_lock:
            if self.device is None:
                return response

            calib = self.device.readCalibration()
            intrinsics = calib.getCameraIntrinsics(
                dai.CameraBoardSocket.RGB, (1920, 1080)
            )
            response.depth_scale = 0.001
            distortion_coefficients = calib.getDistortionCoefficients(
                dai.CameraBoardSocket.RGB
            )
            response.fx = float(intrinsics[0][0])
            response.fy = float(intrinsics[1][1])
            response.cx = float(intrinsics[0][2])
            response.cy = float(intrinsics[1][2])
            response.distortion_coefficients = distortion_coefficients

        return response

    def pubslish_rgb(self):
        if self.rgb_queue is None:
            return 0, False

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 25]

        try:
            rgb_packet = self.rgb_queue.tryGet()
        except RuntimeError as e:
            # X_LINK_ERROR etc.
            self.node.get_logger().error(f"DepthAI RGB stream error: {e}")
            self._reconnect("rgb stream error")
            return 0, False

        if rgb_packet is not None:
            frameRgb = rgb_packet.getCvFrame()
            frameRgb = (
                cv2.rotate(frameRgb, cv2.ROTATE_180) if self.flip_camera else frameRgb
            )

            success, encoded_image = cv2.imencode(".jpg", frameRgb, encode_param)
            if not success:
                self.node.get_logger().warn("Failed to compress RGB frame.")
                return 0, False

            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_image.tobytes()
            self.node.cam_pubs.publish(compressed_msg)
            return len(compressed_msg.data), True

        return 0, False

    def publish_image(self, frame):
        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "16UC1"
        msg.is_bigendian = False
        msg.step = msg.width * 2
        msg.data = frame.tobytes()
        return msg

    def publish_depths(self):
        if self.depth_queue is None:
            return

        try:
            depth_packet = self.depth_queue.tryGet()
        except RuntimeError as e:
            self.node.get_logger().error(f"DepthAI depth stream error: {e}")
            self._reconnect("depth stream error")
            return

        if depth_packet is not None:
            depth_frame = np.ascontiguousarray(depth_packet.getFrame())

            if self.depth_frame is not None:
                self.depth_frame = (
                    self.alpha * depth_frame + (1 - self.alpha) * self.depth_frame
                ).astype(np.uint16)
            else:
                self.depth_frame = depth_frame

            msg_depth_avg = self.publish_image(self.depth_frame)
            self.depth_avg_pubs.publish(msg_depth_avg)

    def publish_feeds(self, devrule=None, stop_event=None):
        self.node.get_logger().info("STARTING TO PUBLISH RGB!!")

        previous_time = 0
        period_s = 1.0 / max(1, int(self.node.fps))
        next_tick = time.time()

        with self._dev_lock:
            if self.device is None:
                self._open_device()

        try:
            while not self.node.stopped and (
                stop_event is None or not stop_event.is_set()
            ):
                # Throttle loop to requested FPS (prevents busy-spin and reduces USB/CPU pressure)
                now = time.time()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick = max(next_tick + period_s, time.time())

                bytes_rgb, rgb_packet_state = self.pubslish_rgb()

                if self.depth_mode:
                    self.publish_depths()

                if rgb_packet_state:
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(
                        current_time, previous_time, bytes_rgb
                    )
                    previous_time = current_time
                    self.node.cam_bw.publish(bw)
        finally:
            self.close()
