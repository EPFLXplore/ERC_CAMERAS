import depthai as dai
import cv2
import sys
import time, os
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
_MAX_DEVICE_ATTEMPTS = 5
_DEVICE_RETRY_DELAY_SEC = 1.5


class Oak1WStereoCamera():
    def __init__(self, node):
        self.node = node
        devices = dai.Device.getAllAvailableDevices()
        if devices:
            print(
                f"{_C_GREEN}Oak1W: DepthAI USB scan — {len(devices)} device(s): {devices}{_C_RESET}",
                flush=True,
            )
        else:
            print(
                f"{_C_RED}Oak1W: DepthAI USB scan — no devices found (check USB / power).{_C_RESET}",
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
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
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
        # info = dai.DeviceInfo(self.serial_number)  # devrule from node params
        # self.device = dai.Device(self.pipeline, info, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)  #10Gbps USB3.2 gen2
        # self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)  #10Gbps USB3.2 gen2
        for attempt in range(1, _MAX_DEVICE_ATTEMPTS + 1):
            try:
                self.device = dai.Device(
                    self.pipeline, deviceInfo=dai.DeviceInfo(str(node.cam_id))
                )  # 10Gbps USB3.2 gen2
                break
            except Exception as e:
                last_err = e
                if attempt < _MAX_DEVICE_ATTEMPTS:
                    print(
                        f"{_C_YELLOW}Oak1W: open failed cam_id={node.cam_id!r} "
                        f"(attempt {attempt}/{_MAX_DEVICE_ATTEMPTS}): {e}; "
                        f"retry in {_DEVICE_RETRY_DELAY_SEC}s{_C_RESET}",
                        file=sys.stderr,
                        flush=True,
                    )
                    time.sleep(_DEVICE_RETRY_DELAY_SEC)
                else:
                    print(
                        f"{_C_RED}Oak1W: failed to open device cam_id={node.cam_id!r} "
                        f"after {_MAX_DEVICE_ATTEMPTS} attempts: {e}{_C_RESET}",
                        file=sys.stderr,
                        flush=True,
                    )
                    print(
                        f"{_C_RED}Oak1W: currently visible devices: "
                        f"{dai.Device.getAllAvailableDevices()}{_C_RESET}",
                        file=sys.stderr,
                        flush=True,
                    )
                    raise
        print(
            f"{_C_GREEN}Oak1W: opened DepthAI device cam_id={node.cam_id!r}.{_C_RESET}",
            flush=True,
        )
        self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)

        self.queueEvents = []
        self.last_jpeg_data = None

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
        intrinsics = self.get_intrinsics()
        distortion_coefficients = self.get_coeffs()
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
        """Publish RGB feeds."""

        self.node.get_logger().info("STARTING TO PUBLISH FRAMES")
        print(
            f"{_C_GREEN}Oak1W: publishing RGB frames (JPEG).{_C_RESET}",
            flush=True,
        )
        encoded_image = None
        previous_time = 0



########################################################

        while not self.node.stopped:
            self.queueEvents = self.device.getQueueEvents(("rgb"))
                            
            latestPacket = {}
            latestPacket["rgb"] = None
            
            for queueName in self.queueEvents:
                packets = self.device.getOutputQueue(queueName).tryGetAll()
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
                
                current_time = time.time()
                bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                previous_time = current_time 
                self.node.cam_bw.publish(bw)
                
                self.frameRgb = None
    