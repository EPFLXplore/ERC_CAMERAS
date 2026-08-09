import os
import sys
import time
import threading

import cv2
import depthai as dai
import numpy as np
from math import gcd
from cv_bridge import CvBridge
from custom_msg.srv import CameraParams
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

_C_RESET = "\033[0m"
_C_GREEN = "\033[1;32m"
_C_RED = "\033[1;31m"
_C_YELLOW = "\033[1;33m"

_MAX_DEVICE_ATTEMPTS = 10
_DEVICE_RETRY_DELAY_SEC = 2.0
_MXID_WAIT_TIMEOUT_SEC = 30.0
_MXID_WAIT_POLL_SEC = 0.5
_MXID_WAIT_LOG_INTERVAL_SEC = 5.0


def _device_mxid(device_info) -> str:
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


class Oak1WStereoCamera:
    def __init__(self, node):
        self.node = node
        self.serial = str(node.cam_id) if getattr(node, "cam_id", "") else ""
        self.ros_name = node.get_name()
        self.device_lock = threading.Lock()
        self.device = None
        self.rgbQueue = None
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


        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value
        self.camera_info_service = self.node.create_service(CameraParams, self.info, self.camera_params_callback)

        self.node.declare_parameter("flip_camera", False)
        self.flip_camera = self.node.get_parameter("flip_camera").get_parameter_value().bool_value

        self.pipeline = dai.Pipeline()
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        enc = self.pipeline.create(dai.node.VideoEncoder)
        rgb_out = self.pipeline.create(dai.node.XLinkOut)

        rgb_out.setStreamName("rgb")
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        width = int(self.node.get_parameter("x").value)
        height = int(self.node.get_parameter("y").value)
        g = gcd(height, 1080)
        scale_num = height // g
        scale_den = 1080 // g
        cam_rgb.setIspScale(scale_num, scale_den)

        cam_rgb.setVideoSize(width, height)
        cam_rgb.setFps(self.node.fps)

        rgb_out.input.setBlocking(False)
        rgb_out.input.setQueueSize(1)

        enc.setDefaultProfilePreset(self.node.fps, dai.VideoEncoderProperties.Profile.MJPEG)
        enc.setLossless(False)
        quality = int(self.node.get_parameter("jpeg_quality").value)
        enc.setQuality(quality)
        enc.setNumFramesPool(2)

        cam_rgb.video.link(enc.input)
        enc.bitstream.link(rgb_out.input)

    def open_device(self):
        with self.device_lock:
            if self.device is not None and self.rgbQueue is not None:
                return

            if self.serial and not _wait_until_mxid_in_depthai_scan(self.serial, self.ros_name, _MXID_WAIT_TIMEOUT_SEC):
                print(
                    f"{_C_YELLOW}Oak1W: MXID {self.serial!r} still not listed after "
                    f"{_MXID_WAIT_TIMEOUT_SEC}s; proceeding anyway "
                    f"(visible_mxids={_mxid_list_from_scan()}){_C_RESET}",
                    flush=True,
                )

            open_kw = {
                "deviceInfo": dai.DeviceInfo(str(self.node.cam_id)),
                "maxUsbSpeed": dai.UsbSpeed.HIGH,
            }

            for attempt in range(1, _MAX_DEVICE_ATTEMPTS + 1):
                try:
                    self.device = dai.Device(self.pipeline, **open_kw)
                    self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)
                    self.last_open_time = time.monotonic()
                    self.last_frame_time = None
                    print(
                        f"{_C_GREEN}Oak1W: opened DepthAI device serial/MXID={self.serial!r} "
                        f"ROS node={self.ros_name!r}.{_C_RESET}",
                        flush=True,
                    )
                    return
                except Exception as e:
                    self.device = None
                    self.rgbQueue = None
                    if attempt < _MAX_DEVICE_ATTEMPTS:
                        print(
                            f"{_C_YELLOW}Oak1W: open failed serial/MXID={self.serial!r} "
                            f"(attempt {attempt}/{_MAX_DEVICE_ATTEMPTS}): {e}; "
                            f"retry in {_DEVICE_RETRY_DELAY_SEC}s{_C_RESET}",
                            file=sys.stderr,
                            flush=True,
                        )
                        time.sleep(_DEVICE_RETRY_DELAY_SEC)
                    else:
                        raise

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
        if self.camera_info_service is not None:
            self.node.destroy_service(self.camera_info_service)
            self.camera_info_service = None

    def is_open(self) -> bool:
        with self.device_lock:
            return self.device is not None and self.rgbQueue is not None

    def is_healthy(self, timeout_sec: float) -> bool:
        if not self.is_open():
            return False
        now = time.monotonic()
        if self.last_frame_time is not None:
            return (now - self.last_frame_time) <= timeout_sec
        if self.last_open_time is None:
            return False
        return (now - self.last_open_time) <= timeout_sec

    def restart_device(self):
        self.close_device()
        self.open_device()



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
            response.fx = response.fy = response.cx = response.cy = 0.0
            response.distortion_coefficients = []
            return response
        finally:
            if opened_for_calibration and getattr(self.node, "stopped", True):
                self.close_device()

        response.fx = float(intrinsics[0][0])
        response.fy = float(intrinsics[1][1])
        response.cx = float(intrinsics[0][2])
        response.cy = float(intrinsics[1][2])
        response.distortion_coefficients = distortion_coefficients
        return response

    def get_intrinsics(self):
        calib_data = self.device.readCalibration()
        return calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1280, 720)

    def get_coeffs(self):
        return self.device.readCalibration().getDistortionCoefficients(dai.CameraBoardSocket.RGB)

    def publish_feeds(self, devrule=None):
        self.node.get_logger().info("STARTING TO PUBLISH FRAMES")
        print(f"{_C_GREEN}Oak1W: publishing RGB frames (JPEG).{_C_RESET}", flush=True)

        previous_time = 0.0
        reconnect_delay_sec = 1.0
        max_reconnect_delay = 10.0

        while not self.node.stopped:
            try:
                if self.device is None or self.rgbQueue is None:
                    self.open_device()
                    reconnect_delay_sec = 1.0

                queue_events = self.device.getQueueEvents(("rgb",))
                latest_rgb = None
                for queue_name in queue_events:
                    packets = self.rgbQueue.tryGetAll() if queue_name == "rgb" else []
                    if packets:
                        latest_rgb = packets[-1]
                    else:
                        time.sleep(0.001)

                if latest_rgb is None:
                    continue

                jpeg_data = bytes(np.array(latest_rgb.getData(), dtype=np.uint8))
                self.last_jpeg_data = jpeg_data
                self.last_frame_time = time.monotonic()

                msg = CompressedImage()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.format = "bgr8; jpeg compressed"
                msg.data = jpeg_data
                self.node.cam_pubs.publish(msg)

                current_time = time.time()
                bw = self.node.calculate_bandwidth(current_time, previous_time, len(jpeg_data))
                previous_time = current_time
                self.node.cam_bw.publish(bw)

            except Exception as e:
                self.node.get_logger().error(
                    f"publish_feeds error, reconnecting in {reconnect_delay_sec:.1f}s: {e}",
                    throttle_duration_sec=1.0,
                )
                self.close_device()
                time.sleep(reconnect_delay_sec)
                reconnect_delay_sec = min(max_reconnect_delay, reconnect_delay_sec * 2.0)
