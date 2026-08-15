import depthai as dai
import cv2
import time
import datetime
import numpy as np
from collections import deque
import threading

from sensor_msgs.msg import CompressedImage, Image
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from std_msgs.msg import Float32, Bool, String

class OakDStereoCamera:
    def __init__(self, node):
        # ------------------- Defining parameters -------------------
        self.node = node
        self.serial_number = self.node.devrule

        self.RGB_RESOLUTION_MAP = {
            "1080P": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "4K": dai.ColorCameraProperties.SensorResolution.THE_4_K,
        }

        self.MONO_RESOLUTION_MAP = {
            "400P": dai.MonoCameraProperties.SensorResolution.THE_400_P,
            "480P": dai.MonoCameraProperties.SensorResolution.THE_480_P,
        }

        self.string_array = ["", ""]
        self.int_array = [0, 0]

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

        self.node.declare_parameter("topic_internal_pub", self.node.default)
        self.topic_internal_pub = (
            self.node.get_parameter("topic_internal_pub").get_parameter_value().string_value
        )

        self.node.declare_parameter("rgb_resolution", self.string_array)
        self.rgbResolution_name = (
            self.node.get_parameter("rgb_resolution").get_parameter_value().string_array_value
        )

        self.node.declare_parameter("mono_resolution", self.string_array)
        self.monoResolution_name = (
            self.node.get_parameter("mono_resolution").get_parameter_value().string_array_value
        )

        self.node.declare_parameter("depth_avg", self.node.default)
        self.depth_avg_topic_string = (
            self.node.get_parameter("depth_avg").get_parameter_value().string_value
        )

        self.node.declare_parameter("topic_resolution", self.node.default)
        self.resolution_topic = (
            self.node.get_parameter("topic_resolution").get_parameter_value().string_value
        )

        self.node.declare_parameter("fps_depth", self.int_array)
        self.fps_depth = (
            self.node.get_parameter("fps_depth").get_parameter_value().integer_array_value
        )

        self.node.declare_parameter("fps_external", self.int_array)
        self.fps_external = (
            self.node.get_parameter("fps_external").get_parameter_value().integer_array_value
        )

        self.node.declare_parameter("number_of_frames_to_average", self.int_array)
        self.number_of_frames_to_average = (
            self.node.get_parameter("number_of_frames_to_average").get_parameter_value().integer_array_value
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
        # we set max exposure time to avoid motion blur when the arm moves
        # WARNING: too low --> too much pepper noise, too high --> motion blur
        self.node.declare_parameter("max_exposure_us", 4600)
        self.max_exposure_us = (
            self.node.get_parameter("max_exposure_us").get_parameter_value().integer_value
        )

        self.node.declare_parameter("current_resolution", "1080P")
        current_resolution = (
            self.node.get_parameter("current_resolution").get_parameter_value().string_value
        )

        self.node.declare_parameter("fps_internal", self.int_array)
        self.fps_internal = (
            self.node.get_parameter("fps_internal").get_parameter_value().integer_array_value
        )

        self.camera_info_service = self.node.create_service(
            CameraParams, self.info + self.serial_number, self.camera_params_callback
        )

        self.resolution_sub = self.node.create_subscription(
            String, self.resolution_topic, self.switch_resolution_callback, 10
        )

        # ------------------- Publishers -------------------
        self.state_depth = self.node.create_publisher(Bool, self.state_depth_topic, 1)
        self.depth_pubs = self.node.create_publisher(
            Image, self.depth_topic_string, qos_profile=self.node.qos_profile
        )
        self.depth_avg_pubs = self.node.create_publisher(
            Image, self.depth_avg_topic_string, qos_profile=self.node.qos_profile
        )
        self.cam_pubs = self.node.create_publisher(
            CompressedImage, self.node.publisher_topic, qos_profile=self.node.qos_profile
        )
        self.cam_internal_pubs = self.node.create_publisher(
            CompressedImage, self.topic_internal_pub, qos_profile=self.node.qos_profile
        )

        self.cam_params_switch_pub = self.node.create_publisher(
            Bool, "/HD/camera/params_switch", qos_profile=self.node.qos_profile
        )

        # ------------------- Device and pipeline init -------------------
        self.pipeline = dai.Pipeline() # Pipeline built with _build_pipeline()

        ## ---------- Camera parameters ----------
        self.rgbCamSocket = dai.CameraBoardSocket.CAM_A
        self._set_current_resolution(current_resolution)
        self.node.get_logger().info(f"Current resolution set to {self.current}")
        self.node.get_logger().info(f"RGB Resolution name: {self.rgbResolution_name}")
        self.set_resolution_from_name() #Converts string to dai.CameraProperties.SensorResolution
        
        self.previous_frames : deque[np.ndarray] = deque(maxlen=self.number_of_frames_to_average[self.current])

        self.CS_resolution = (512, 288)
        self.CS_compression_quality = 15   # now applied by the on-device MJPEG encoder
        if self.rgbResolution == dai.ColorCameraProperties.SensorResolution.THE_4_K:
            self.internal_quality = 90  # limit value before fps drop
        else:
            self.internal_quality = 95  # default value

        ### ------------ For HDS --------------
        #For Nav it should be the other way around
        self.extended_disparity = True # divides minimum depth by 2
        self.subpixel = not self.extended_disparity #incompatible with extended_disparity, better for long range (over 3m) but worse for close objects
        self.IRdot = 0 #Only useful indoors and if Oak-D pro is used (between 0 and 1)
        ### -----------------------------------
        ## ---------- End parameters ----------

        self.rgb_res = None
        self.depth_res = None

        self.RESOLUTION_MAP = {
            dai.MonoCameraProperties.SensorResolution.THE_400_P: (640, 400),
            dai.MonoCameraProperties.SensorResolution.THE_480_P: (640, 480),
            dai.ColorCameraProperties.SensorResolution.THE_1080_P: (1920, 1080),
            dai.ColorCameraProperties.SensorResolution.THE_4_K: (3840, 2160),
        }
        #affect the resolution size to rgb_res and depth_res
        self.set_resolution()

        self._build_pipeline()

        # ------------------- Runtime state -------------------
        self.device = None
        self.rgb_queue = None
        self.rgb_ext_queue = None
        self.depth_queue = None
        self._dev_lock = threading.RLock()

        # reconnect backoff
        self._reconnect_delay_s = 1.0
        self._reconnect_max_delay_s = 10.0
        self._watchdog_timeout_s = 3.0

    def _open_device(self):
        """
        Open device + queues.
        IMPORTANT: stop forcing SUPER_PLUS; many systems will drop link after a few seconds.
        We'll prefer SUPER and only try SUPER_PLUS second (optional).
        """
        last_err = None

        # Prefer SUPER (USB3) for stability; try SUPER_PLUS as a second attempt.
        for speed in (dai.UsbSpeed.SUPER_PLUS, dai.UsbSpeed.SUPER, dai.UsbSpeed.HIGH):
            try:
                dev = dai.Device(self.pipeline, maxUsbSpeed=speed)

                # Keep queues small to avoid backlog/latency and reduce host pressure
                self.rgb_queue = dev.getOutputQueue(
                    name="rgb", maxSize=1, blocking=False
                )
                self.rgb_ext_queue = dev.getOutputQueue(
                    name="rgb_ext", maxSize=1, blocking=False
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
                    f"DepthAI device opened. Requested max USB speed: {speed}, "
                    f"actual negotiated speed: {dev.getUsbSpeed()}"
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
        # Close and reopen with bounded backoff. Keep failures inside the
        # camera thread so one bad reconnect attempt doesn't kill streaming.
        self.node.get_logger().warn(f"DepthAI reconnecting ({reason}) ...")
        self.close()
        time.sleep(self._reconnect_delay_s)
        try:
            with self._dev_lock:
                if self.device is None:
                    self._open_device()
            self._reconnect_delay_s = 1.0
            return True
        except Exception as e:
            self.node.get_logger().error(f"DepthAI reconnect failed: {e}")
            self._reconnect_delay_s = min(
                self._reconnect_delay_s * 2.0, self._reconnect_max_delay_s
            )
            return False

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
                self.rgb_ext_queue = None
                self.depth_queue = None
                self.depth_frame = None

    def stop(self):
        self.close()

    def switch_resolution_callback(self, msg: String):
        if msg.data != self.rgbResolution_name[self.current]:
            with self._dev_lock:
                self.close()
                self._update_pipeline(msg)
                self._open_device()
                self.cam_params_switch_pub.publish(Bool(data=True))

    #---------------------------- Callbacks -------------------
    def depth_callback(self, request, response):
        self.depth_mode = request.data
        response.success = True
        self.state_depth.publish(Bool(data=self.depth_mode))
        if self.depth_mode:
            self.node.get_logger().info(f"Starting to publish depth:")
        return response


    def camera_params_callback(self, request, response):
        with self._dev_lock:
            if self.device is None:
                try:
                    self._open_device()
                except Exception as e:
                    self.node.get_logger().error(f"Failed to open device for camera params: {e}")
                if self.device is None:
                    self.node.get_logger().error("Failed to get camera parameters: device not connected.")
                    return response

            try:
                calib = self.device.readCalibration()
                distortion_coefficients = calib.getDistortionCoefficients(self.rgbCamSocket)
  
            except RuntimeError as e:
                self.node.get_logger().error(f"Device disconnected while reading calibration: {e}")
                self.device = None
                self.rgb_queue = None
                self.rgb_ext_queue = None
                self.depth_queue = None
                self.depth_frame = None
                return response

        if self.rgb_res is not None:
            intrinsics = np.array(
                calib.getCameraIntrinsics(self.rgbCamSocket, (self.rgb_res[0], self.rgb_res[1])),
                dtype=np.float64,
            )

            if (intrinsics[0][0] == 0 or intrinsics[1][1] == 0 or intrinsics[0][2] == 0 or intrinsics[1][2]== 0):
                self.node.get_logger().warn("Camera intrinsics not found, using default values.")
                # default factory setting for calibrations of OAK-D pro not calibrated by hand for 1920/1080 full baka scaled with resolution
                response.fx = 1516.3 * self.rgb_res[0]/1920
                response.fy = 1516.4 * self.rgb_res[1]/1080
                response.cx = 949.3 * self.rgb_res[0]/1920
                response.cy = 564.4 * self.rgb_res[1]/1080
                response.distortion_coefficients = [1.23231707e+01, -1.15954918e+02, 7.17240968e-04, 1.20075652e-04, 4.35855652e+02, 1.20713158e+01, -1.14148094e+02, 4.28597443e+02, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.37381395e-03, -5.79341940e-05]
            
            else:
                self.node.get_logger().info("Camera intrinsics found, using them.")
                self.node.get_logger().info(f"fx ={intrinsics[0][0]}, fy ={intrinsics[1][1]}, cx ={intrinsics[0][2]}, cy ={intrinsics[1][2]}")
                response.fx = float(intrinsics[0][0])
                response.fy = float(intrinsics[1][1])
                response.cx = float(intrinsics[0][2])
                response.cy = float(intrinsics[1][2])
                response.distortion_coefficients = distortion_coefficients

                self.node.get_logger().info(f"distortion_coefficients: {distortion_coefficients}")

            response.rgb_w = self.rgb_res[0]
            response.rgb_h = self.rgb_res[1]

        else:
            self.node.get_logger().error("Failed to get intrinsics, resolution not in map or not set")
            return response

        if self.depth_res is not None:
            response.depth_w = self.depth_res[0]
            response.depth_h = self.depth_res[1]
            response.depth_scale = 0.001
        else:
            self.node.get_logger().error("Depth resolution not in map or not set")


        return response

    @staticmethod
    #---------------------------- Publishing -------------------
    def _latest(queue):
        """Drain a queue and return only the freshest packet (or None)."""
        packets = queue.tryGetAll()
        return packets[-1] if packets else None

    def _encode_jpeg_msg(self, packet, quality):
        try:
            frame = packet.getCvFrame()
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
            )
        except RuntimeError as e:
            self.node.get_logger().error(f"DepthAI RGB frame conversion error: {e}")
            self._reconnect("rgb frame conversion error")
            return None
        except cv2.error as e:
            self.node.get_logger().error(f"OpenCV JPEG encode error: {e}")
            return None

        if not ok:
            return None

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        compressed_msg.data = encoded.tobytes()
        return compressed_msg

    def publish_rgb_external(self):
        """
        External low-res feed. The frame is downscaled on the camera, then
        JPEG-encoded on the host to reduce OAK compute load.
        """
        if self.rgb_ext_queue is None:
            return 0, False

        try:
            pkt = self._latest(self.rgb_ext_queue)
        except RuntimeError as e:
            self.node.get_logger().error(f"DepthAI external RGB stream error: {e}")
            self._reconnect("external rgb stream error")
            return 0, False

        if pkt is None:
            return 0, False

        compressed_msg = self._encode_jpeg_msg(pkt, self.CS_compression_quality)
        if compressed_msg is None:
            self.node.get_logger().error("Failed to JPEG-encode external RGB frame.")
            return 0, False

        self.cam_pubs.publish(compressed_msg)
        return len(compressed_msg.data), True

    def publish_rgb_internal(self):
        """
        Internal full-res feed. Raw frames come from the OAK and are
        JPEG-encoded on the host to reduce OAK compute load.
        """
        if self.rgb_queue is None:
            return

        try:
            rgb_packet = self._latest(self.rgb_queue)
        except RuntimeError as e:
            # X_LINK_ERROR etc.
            self.node.get_logger().error(f"DepthAI RGB stream error: {e}")
            self._reconnect("rgb stream error")
            return

        if rgb_packet is not None:
            compressed_msg = self._encode_jpeg_msg(rgb_packet, self.internal_quality)
            if compressed_msg is None:
                self.node.get_logger().error("Failed to JPEG-encode internal RGB frame.")
                return

            self.cam_internal_pubs.publish(compressed_msg)

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
            depth_packet = self._latest(self.depth_queue)
        except RuntimeError as e:
            self.node.get_logger().error(f"DepthAI depth stream error: {e}")
            self._reconnect("depth stream error")
            return

        if depth_packet is not None:
            depth_frame = depth_packet.getFrame()
            depth_frame = cv2.rotate(depth_frame, cv2.ROTATE_180) if self.flip_camera else depth_frame
            depth_frame = np.ascontiguousarray(depth_frame)

            # Unused as it's the topic for the raw depth
            #msg_depth = self.publish_image(depth_frame)
            #self.depth_pubs.publish(msg_depth)

            self.previous_frames.append(depth_frame)
            self.depth_frame = np.ascontiguousarray(
                np.mean(self.previous_frames, axis=0).round().astype(np.uint16)
            )

            msg_depth_avg = self.publish_image(self.depth_frame)
            self.depth_avg_pubs.publish(msg_depth_avg)

    def publish_feeds(self, devrule=None, stop_event=None):
        self.node.get_logger().info("STARTING TO PUBLISH RGB!!")

        previous_time = 0

        external_c = 0

        last_event_time = time.monotonic()

        with self._dev_lock:
            if self.device is None:
                try:
                    self._open_device()
                except Exception as e:
                    self.node.get_logger().error(f"Failed to open DepthAI device: {e}")

        try:
            while not self.node.stopped and (
                stop_event is None or not stop_event.is_set()
            ):
                # LATENCY: event-driven instead of a sleep-based timer.
                # getQueueEvents blocks until a frame actually arrives, so the
                # camera drives the loop (no sleep() jitter, no missed frames).
                # The 100 ms timeout keeps the stop flags responsive.
                if self.device is None:
                    self._reconnect("device not open")
                    last_event_time = time.monotonic()
                    time.sleep(0.1)
                    continue

                try:
                    events = self.device.getQueueEvents(
                        ("rgb", "rgb_ext", "depth"),
                        timeout=datetime.timedelta(milliseconds=100),
                    )
                except RuntimeError as e:
                    self.node.get_logger().error(f"DepthAI event wait error: {e}")
                    self._reconnect("event wait error")
                    continue

                if events:
                    last_event_time = time.monotonic()
                elif time.monotonic() - last_event_time > self._watchdog_timeout_s:
                    self._reconnect(
                        f"no frames for {self._watchdog_timeout_s:.1f}s"
                    )
                    last_event_time = time.monotonic()
                    continue

                for name in events:
                    if name == "rgb":
                        self.publish_rgb_internal()

                    elif name == "rgb_ext":
                        # Publish external feed at the requested average FPS.
                        # Counter increments per ext frame received (camera fps),
                        # publishing fps_external out of every node.fps frames.
                        external_c += self.fps_external[self.current]
                        if external_c >= self.fps_internal[self.current]:
                            bytes_rgb, rgb_packet_state = self.publish_rgb_external()
                            if rgb_packet_state:
                                current_time = time.time()
                                bw = self.node.calculate_bandwidth(
                                    current_time, previous_time, bytes_rgb
                                )
                                previous_time = current_time
                                self.node.cam_bw.publish(bw)
                            external_c -= self.fps_internal[self.current]
                        else:
                            # Drop this ext frame so the topic holds fps_external
                            try:
                                if self.rgb_ext_queue is not None:
                                    self.rgb_ext_queue.tryGetAll()
                            except RuntimeError:
                                pass

                    elif name == "depth" and self.depth_mode:
                        self.publish_depths()


        finally:
            self.close()

    #---------------------------- Pipeline management -------------------
    def _set_current_resolution(self, resolution):
        if resolution == "1080P":
            self.current = 0
        elif resolution == "4K":
            self.current = 1
        else:
            self.node.get_logger().error(f"Unknown resolution {resolution}, defaulting to 1080P")
            self.current = 0

    def _update_pipeline(self, rgb_resolution):
        """Rebuild the pipeline with the current resolution settings."""
        self._set_current_resolution(rgb_resolution.data)
        self.node.get_logger().info(f"Switching camera resolution to RGB: {self.rgbResolution_name[self.current]}, Mono: {self.monoResolution_name[self.current]}")
        self.previous_frames : deque[np.ndarray] = deque(maxlen=self.number_of_frames_to_average[self.current])
        self.set_resolution_from_name()
        self.set_resolution()

        self._build_pipeline()
        
    def set_resolution_from_name(self):
        """Converts string to dai.CameraProperties.SensorResolution"""
        if self.rgbResolution_name[self.current] in self.RGB_RESOLUTION_MAP:
            self.rgbResolution = self.RGB_RESOLUTION_MAP[self.rgbResolution_name[self.current]]
        else:
            self.node.get_logger().error(f"RGB Resolution {self.rgbResolution_name[self.current]} not in the map, using default 1080P")
            self.rgbResolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P
                
        if self.monoResolution_name[self.current] in self.MONO_RESOLUTION_MAP:
            self.monoResolution = self.MONO_RESOLUTION_MAP[self.monoResolution_name[self.current]]
        else:
            self.node.get_logger().error(f"Mono Resolution {self.monoResolution_name[self.current]} not in the map, using default 480P")
            self.monoResolution = dai.MonoCameraProperties.SensorResolution.THE_480_P

    def set_resolution(self):
        """Sets the resolution size from the resolution (dai.CameraProperties.SensorResolution)"""
        if self.rgbResolution in self.RESOLUTION_MAP:
            self.rgb_res = self.RESOLUTION_MAP[self.rgbResolution]
        else:
            self.node.get_logger().error(
                "RGB Resolution not in the map"
            )
        
        if self.monoResolution in self.RESOLUTION_MAP:
            self.depth_res = self.RESOLUTION_MAP[self.monoResolution]
        else:
            self.node.get_logger().error(
                "Depth Resolution not in the map"
            )

    
    def _build_pipeline(self):
        """(Re)build the entire pipeline graph from scratch using current
        self.rgbResolution / self.monoResolution / fps settings.
        Must be called before every _open_device(), including on resolution switch."""
        self.pipeline = dai.Pipeline()

        # Sources and Outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.left = self.pipeline.create(dai.node.MonoCamera)
        self.right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        rgbExtOut = self.pipeline.create(dai.node.XLinkOut)
        depthOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        rgbExtOut.setStreamName("rgb_ext")
        depthOut.setStreamName("depth")

        # LATENCY: device-side XLink queues default to blocking with ~8 frames
        # buffered on the camera. That adds up to ~8 frames of standing delay.
        # Non-blocking + size 1 means old frames are overwritten, so the host
        # always receives the newest frame.
        for xout in (rgbOut, rgbExtOut, depthOut):
            xout.input.setBlocking(False)
            xout.input.setQueueSize(1)

        self.pipeline.setXLinkChunkSize(0) # 0 = unlimited, no chunking, recommended for latency

        ## ---------- Camera parameters ----------
        self.camRgb.setBoardSocket(self.rgbCamSocket)
        self.camRgb.setResolution(self.rgbResolution)
        self.camRgb.setFps(self.fps_internal[self.current])
        # Exposure stays auto (ISO/gain included), but the AE is not allowed to
        # buy brightness with a long shutter: past ~4.5 ms the tags smear as soon
        # as the arm moves. Capping the limit makes AE trade time for gain instead.
        self.camRgb.initialControl.setAutoExposureEnable()
        self.camRgb.initialControl.setAutoExposureLimit(max(1, self.max_exposure_us))
        # Flip is done on-sensor so both raw RGB streams arrive already oriented.
        if self.flip_camera:
            self.camRgb.setImageOrientation(
                dai.CameraImageOrientation.ROTATE_180_DEG
            )
        self.node.get_logger().info(f"RGB camera set to {self.rgbResolution} at {self.fps_internal[self.current]} FPS")

        ## Mono Cameras
        self.left.setResolution(self.monoResolution)
        self.left.setCamera("left")
        self.left.setFps(self.fps_depth[self.current])

        self.right.setResolution(self.monoResolution)
        self.right.setCamera("right")
        self.right.setFps(self.fps_depth[self.current])

        ## Stereo Properties
        self.stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY
        )

        self.stereo.setDepthAlign(self.rgbCamSocket)

        if self.depth_res is not None:
            self.stereo.setOutputSize(
                self.depth_res[0],
                self.depth_res[1]
            )
        else:
            self.node.get_logger().error(
                "Resolution doesn't match predefined output size"
            )

        self.stereo.setRectification(True)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setExtendedDisparity(self.extended_disparity)   # divides minimum depth by 2
        self.stereo.setSubpixel(self.subpixel)           # incompatible with extended_disparity, better for long range (over 3m) but worse for close objects

        ## RGB feeds are JPEG-compressed on the host. This saves OAK compute at
        # the cost of higher USB bandwidth.
        self.manip = self.pipeline.create(dai.node.ImageManip)
        self.manip.initialConfig.setResize(
            self.CS_resolution[0], self.CS_resolution[1]
        )
        self.manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
        self.manip.setMaxOutputFrameSize(
            self.CS_resolution[0] * self.CS_resolution[1] * 3
        )
        # LATENCY: don't let the manip node buffer/backpressure either.
        self.manip.inputImage.setBlocking(False)
        self.manip.inputImage.setQueueSize(1)

        # linking
        self.camRgb.video.link(rgbOut.input)                 # Raw NV12 -> "rgb"

        self.camRgb.video.link(self.manip.inputImage)
        self.manip.out.link(rgbExtOut.input)                 # Raw BGR -> "rgb_ext"

        self.left.out.link(self.stereo.left)
        self.right.out.link(self.stereo.right)
        self.stereo.depth.link(depthOut.input)
