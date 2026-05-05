import depthai as dai
import cv2
import sys
import time, os
import threading
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

_C_RESET = "\033[0m"
_C_GREEN = "\033[1;32m"
_C_RED = "\033[1;31m"
_C_YELLOW = "\033[1;33m"

# Multi-camera USB hub: staggered launch + retries reduce X_LINK_DEVICE_NOT_FOUND.
_MAX_DEVICE_ATTEMPTS = 10
_DEVICE_RETRY_DELAY_SEC = 2.0
# DepthAI may list fewer devices until siblings finish booting; wait before dai.Device().
_MXID_WAIT_TIMEOUT_SEC = 30.0
_MXID_WAIT_POLL_SEC = 0.5
_MXID_WAIT_LOG_INTERVAL_SEC = 5.0


def _device_mxid(device_info) -> str:
    """Serial / MXID string from depthai.DeviceInfo for error logs (API varies by depthai version)."""
    get_mxid = getattr(device_info, "getMxId", None) or getattr(device_info, "getMxid", None)
    if callable(get_mxid):
        try:
            return str(get_mxid())
        except Exception:
            pass
    for attr in ("mxid", "deviceId", "name"):
        if hasattr(device_info, attr):
            try:
                return str(getattr(device_info, attr))
            except Exception:
                pass
    return repr(device_info)


def _devices_serial_summary() -> str:
    devs = dai.Device.getAllAvailableDevices()
    if not devs:
        return "(none)"
    return ", ".join(_device_mxid(d) for d in devs)


def _mxid_list_from_scan() -> list:
    return [_device_mxid(d) for d in dai.Device.getAllAvailableDevices()]


def _wait_until_mxid_in_depthai_scan(mxid: str, ros_name: str, timeout_sec: float) -> bool:
    """Return True once getAllAvailableDevices() includes this MXID, else False after timeout."""
    mxid = (mxid or "").strip()
    if not mxid:
        return True
    deadline = time.monotonic() + timeout_sec
    last_log = 0.0
    while time.monotonic() < deadline:
        if mxid in _mxid_list_from_scan():
            return True
        now = time.monotonic()
        if now - last_log >= _MXID_WAIT_LOG_INTERVAL_SEC:
            last_log = now
            vis = _mxid_list_from_scan()
            print(
                f"{_C_YELLOW}Oak1W: waiting for MXID {mxid!r} (node {ros_name!r}) in DepthAI scan; "
                f"visible_mxids={vis}{_C_RESET}",
                flush=True,
            )
        time.sleep(_MXID_WAIT_POLL_SEC)
    return False


class Oak1WStereoCamera():
    def __init__(self, node):
        self.node = node
        self.serial = str(node.cam_id) if getattr(node, "cam_id", "") else ""
        self.ros_name = node.get_name()
        self.device_lock = threading.Lock()
        self.device = None
        self.rgbQueue = None
        self.queueEvents = []
        self.last_jpeg_data = None
        self.last_open_time = None
        self.last_frame_time = None
        devices = dai.Device.getAllAvailableDevices()
        if devices:
            print(
                f"{_C_GREEN}Oak1W: DepthAI USB scan — {len(devices)} device(s) visible here "
                f"(MXIDs already opened in another camera node are usually omitted): {devices}{_C_RESET}",
                flush=True,
            )
        else:
            print(
                f"{_C_RED}Oak1W: DepthAI USB scan — no devices found (check USB / power). "
                f"requested_serial/MXID={self.serial!r} ROS node={self.ros_name!r}{_C_RESET}",
                flush=True,
            )
        self.bridge = CvBridge()
        self.frameRgb = None
        
        self.node.declare_parameter("screenshot", self.node.default)
        self.screenshot_topic = self.node.get_parameter("screenshot").get_parameter_value().string_value
        self.path_images = self.screenshot_topic[5:]
        
        self.take_screenshot = self.node.create_service(SetBool, 
                            self.screenshot_topic, self.take_screenshot, callback_group=MutuallyExclusiveCallbackGroup())
        # self.serial_number = self.node.get_parameter("serial_number").get_parameter_value().string_value
        # self.serial_number = "19443010714B177E00"  # hardcoded for test

        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value

        self.node.declare_parameter("flip_camera", False)
        self.flip_camera = self.node.get_parameter("flip_camera").get_parameter_value().bool_value
        
        # self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)
        self.camera_info_service = self.node.create_service(CameraParams, self.info, self.camera_params_callback)

        self.pipeline = dai.Pipeline()
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
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        # ISP size must match ROS params (x,y) used for getCameraIntrinsics and for JPEG decode.
        # Luxonis convention: setIspScale(2, 3) on THE_1080_P → 1280×720.
        tw, th = int(self.node.x), int(self.node.y)
        if tw == 1280 and th == 720:
            camRgb.setIspScale(2, 3)
        # else: leave default ISP size for THE_1080_P (typically 1920×1080); intrinsics use (tw, th).

        camRgb.setFps(self.node.fps)

        rgbOut.input.setBlocking(False)
        rgbOut.input.setQueueSize(1)
        enc.setDefaultProfilePreset(self.node.fps, dai.VideoEncoderProperties.Profile.MJPEG)
        enc.setLossless(False)
        enc.setQuality(95) # 
        enc.setNumFramesPool(2)

        # Oak1W hardware encoder for the RGB JPEG stream.
        camRgb.video.link(enc.input)
        enc.bitstream.link(rgbOut.input)
        
        # camRgb.isp.link(rgbOut.input) # FOR OTHER TASKS

        # ------------------ end RGB ------------------
	    ### -- NEW STUFF FROM 13TH AUGUST TO LIMIT THE DATA RATE OF THE CAMERA -- ###
        self.target_mbps = 10.0
        self.min_quality = 15
        self.max_quality = 95
        self.quality = 40                 # start point
        self.scale = 1.0                  # downscale factor if needed
        self.ema_alpha = 0.2              # smoothing for size/bw

        self.bytes_per_frame_budget = max(1, int((self.target_mbps * 1e6) / 8.0 / max(1, self.node.fps)))
        self.ema_bytes = self.bytes_per_frame_budget

	    ### ------------------------------------------------------------------- ###

    def open_device(self):
        with self.device_lock:
            if self.device is not None and self.rgbQueue is not None:
                return

            if self.serial and not _wait_until_mxid_in_depthai_scan(self.serial, self.ros_name, _MXID_WAIT_TIMEOUT_SEC):
                print(
                    f"{_C_YELLOW}Oak1W: MXID {self.serial!r} still not listed after {_MXID_WAIT_TIMEOUT_SEC}s; "
                    f"proceeding to open anyway (visible_mxids={_mxid_list_from_scan()}){_C_RESET}",
                    flush=True,
                )

            # _open_kw = {"deviceInfo": dai.DeviceInfo(str(self.node.cam_id))}
            # if hasattr(dai, "UsbSpeed") and hasattr(dai.UsbSpeed, "SUPER"):
            #     _open_kw["maxUsbSpeed"] = dai.UsbSpeed.SUPER

            _open_kw = {
                "deviceInfo": dai.DeviceInfo(str(self.node.cam_id)),
                "maxUsbSpeed": dai.UsbSpeed.HIGH
            }

            for attempt in range(1, _MAX_DEVICE_ATTEMPTS + 1):
                try:
                    self.device = dai.Device(self.pipeline, **_open_kw)
                    self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
                    self.last_open_time = time.monotonic()
                    break
                except Exception as e:
                    self.device = None
                    self.rgbQueue = None
                    if attempt < _MAX_DEVICE_ATTEMPTS:
                        print(
                            f"{_C_YELLOW}Oak1W: open failed serial/MXID={self.serial!r} "
                            f"ROS node={self.ros_name!r} (attempt {attempt}/{_MAX_DEVICE_ATTEMPTS}): {e}; "
                            f"retry in {_DEVICE_RETRY_DELAY_SEC}s{_C_RESET}",
                            file=sys.stderr,
                            flush=True,
                        )
                        time.sleep(_DEVICE_RETRY_DELAY_SEC)
                    else:
                        print(
                            f"{_C_RED}Oak1W: failed to open after {_MAX_DEVICE_ATTEMPTS} attempts — "
                            f"serial/MXID={self.serial!r} ROS node={self.ros_name!r}: {e}{_C_RESET}",
                            file=sys.stderr,
                            flush=True,
                        )
                        print(
                            f"{_C_RED}Oak1W: visible DeviceInfo list: "
                            f"{dai.Device.getAllAvailableDevices()}{_C_RESET}",
                            file=sys.stderr,
                            flush=True,
                        )
                        print(
                            f"{_C_RED}Oak1W: visible serials/MXIDs (parsed): "
                            f"{_devices_serial_summary()}{_C_RESET}",
                            file=sys.stderr,
                            flush=True,
                        )
                        raise

            print(
                f"{_C_GREEN}Oak1W: opened DepthAI device serial/MXID={self.serial!r} "
                f"ROS node={self.ros_name!r}.{_C_RESET}",
                flush=True,
            )

    def close_device(self):
        with self.device_lock:
            self.rgbQueue = None
            if self.device is not None:
                try:
                    self.device.close()
                except Exception:
                    pass
                self.device = None
            self.last_open_time = None
            self.last_frame_time = None

    def destroy_ros_entities(self):
        if self.take_screenshot is not None:
            self.node.destroy_service(self.take_screenshot)
            self.take_screenshot = None
        if self.camera_info_service is not None:
            self.node.destroy_service(self.camera_info_service)
            self.camera_info_service = None

    def is_open(self):
        with self.device_lock:
            return self.device is not None and self.rgbQueue is not None

    def is_healthy(self, timeout_sec):
        if not self.is_open():
            return False

        now = time.monotonic()
        if self.last_frame_time is not None:
            return (now - self.last_frame_time) <= timeout_sec

        if self.last_open_time is None:
            return False

        # Give a newly opened device one timeout window to deliver its first frame.
        return (now - self.last_open_time) <= timeout_sec

    def restart_device(self):
        self.close_device()
        self.open_device()

    def take_screenshot(self, request, response):
        jpeg = self.last_jpeg_data
        if jpeg is not None:
            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                response.success = False
                return response

            name = str(time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())) + '.png'
            image_dir  = os.path.join(
                '/home/xplore/dev_ws/photos_competition',
                self.path_images
            )
            os.makedirs(image_dir, exist_ok=True)

            image_path = os.path.join(image_dir, name)            
            cv2.imwrite(image_path, frame)
            
            response.success = True
        else:
            response.success = False
            
        return response

    def camera_params_callback(self, request, response):
        opened_for_calibration = False
        try:
            if not self.is_open():
                self.open_device()
                opened_for_calibration = True
            intrinsics = self.get_intrinsics()
            distortion_coefficients = self.get_coeffs()
        except Exception as e:
            self.node.get_logger().error(
                f"Failed to read camera calibration for MXID={self.serial!r}: {e}",
                throttle_duration_sec=1.0,
            )
            response.fx = 0.0
            response.fy = 0.0
            response.cx = 0.0
            response.cy = 0.0
            response.distortion_coefficients = []
            return response
        finally:
            if opened_for_calibration and getattr(self.node, "stopped", True):
                self.close_device()

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
            frame = cv2.imdecode(np.frombuffer(rgb_frame.getData(), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return None

            if self.flip_camera:
                rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)
                return rotated_frame
            else:
                return frame

    def get_intrinsics(self):
        calib_data = self.device.readCalibration()
        # Must match the actual encoded RGB resolution (same as camera node x,y / ISP scale).
        w = max(1, int(self.node.x))
        h = max(1, int(self.node.y))
        intrinsics = calib_data.getCameraIntrinsics(
            dai.CameraBoardSocket.RGB,
            w,
            h,
        )
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
        """Publish RGB feeds."""

        self.node.get_logger().info("STARTING TO PUBLISH FRAMES")
        print(
            f"{_C_GREEN}Oak1W: publishing RGB frames (JPEG).{_C_RESET}",
            flush=True,
        )
        previous_time = 0
        reconnect_delay_sec = 1.0
        max_reconnect_delay_sec = 10.0



########################################################

        while not self.node.stopped:
            try:
                if self.device is None or self.rgbQueue is None:
                    self.open_device()
                    reconnect_delay_sec = 1.0

                self.queueEvents = self.device.getQueueEvents(("rgb"))
                                
                latestPacket = {}
                latestPacket["rgb"] = None
                
                for queueName in self.queueEvents:
                    packets = self.rgbQueue.tryGetAll() if queueName == "rgb" else self.device.getOutputQueue(queueName).tryGetAll()
                    if len(packets) > 0:
                        latestPacket[queueName] = packets[-1]
                    else:
                        # Avoid CPU usage with TryGetAll all the time
                        time.sleep(0.001)
                        continue
                
                if latestPacket["rgb"] is not None:
                    raw = latestPacket["rgb"].getData()
                    jpeg_data = np.array(raw, dtype=np.uint8).tobytes()
                    self.last_jpeg_data = jpeg_data

                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_msg.format = "bgr8; jpeg compressed"
                    compressed_msg.data = jpeg_data
                    self.node.cam_pubs.publish(compressed_msg)
                    self.last_frame_time = time.monotonic()
                    
                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)
                    
                    self.frameRgb = None
            except Exception as e:
                self.node.get_logger().error(
                    f"publish_feeds error, reconnecting in {reconnect_delay_sec:.1f}s: {e}",
                    throttle_duration_sec=1.0,
                )
                self.close_device()
                time.sleep(reconnect_delay_sec)
                reconnect_delay_sec = min(max_reconnect_delay_sec, reconnect_delay_sec * 2.0)