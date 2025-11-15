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

class Oak1WStereoCamera():
    def __init__(self, node):
        print("Found Oak1W Stereo Camera : ", dai.Device.getAllAvailableDevices())
        self.node = node
        self.bridge = CvBridge()
        self.frameRgb = None
        
        self.node.declare_parameter("screenshot", self.node.default)
        self.screenshot_topic = self.node.get_parameter("screenshot").get_parameter_value().string_value
        self.path_images = self.screenshot_topic[5:]
        
        self.take_screenshot = self.node.create_service(SetBool, 
                            self.screenshot_topic, self.take_screenshot, callback_group=MutuallyExclusiveCallbackGroup())
        # self.serial_number = self.node.get_parameter("devrule").get_parameter_value().string_value
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
        # enc = self.pipeline.create(dai.node.VideoEncoder) # FOR NAV TASK
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
        camRgb.video.link(rgbOut.input) 
        ## -------------- FOR NAV TASK -------------
        # enc.setDefaultProfilePreset(15, dai.VideoEncoderProperties.Profile.MJPEG)
        # enc.setLossless(False)
        # enc.setQuality(100)# --> revert to 30 for other tasks
        # enc.setNumFramesPool(2)
        # enc.setFrameRate(self.node.fps)

        # #Oak1W Hardware encoder for rgb jpeg stream
        # camRgb.video.link(enc.input)
        # enc.bitstream.link(rgbOut.input)# already-compressed JPEG
        ## --------------------------------
        
        # camRgb.isp.link(rgbOut.input) # FOR OTHER TASKS

        # ------------------ end RGB ------------------
        # info = dai.DeviceInfo(self.serial_number)  # devrule from node params
        # self.device = dai.Device(self.pipeline, info, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)  #10Gbps USB3.2 gen2
        self.device = dai.Device(self.pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS)  #10Gbps USB3.2 gen2

        self.rgbQueue = self.device.getOutputQueue("rgb", maxSize=1, blocking=False)

        self.queueEvents = []

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
        if self.frameRgb is not None:
            self.node.get_logger().info(f"reiugf")
            name = str(time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())) + '.png'
            image_dir  = os.path.join(
                '/home/xplore/dev_ws/photos_competition',
                self.path_images
            )
            os.makedirs(image_dir, exist_ok=True)

            image_path = os.path.join(image_dir, name)            
            cv2.imwrite(image_path, self.frameRgb)
            
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
        
            if self.flip_camera:
                rotated_frame = cv2.rotate(rgb_frame.getCvFrame(), cv2.ROTATE_180)
                return rotated_frame
            else:
                return rgb_frame.getCvFrame()

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
        encoded_image = None
        previous_time = 0



########################################################

        while not self.node.stopped:
            ### -- NOT FOR NAV,  NEW STUFF FROM 13TH AUGUST TO LIMIT THE DATA RATE OF THE CAMERA -- ###
            # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            ### --------------------------------------------------------------------- ###
            


            self.queueEvents = self.device.getQueueEvents(("rgb"))
                            
            latestPacket = {}
            latestPacket["rgb"] = None
            
            for queueName in self.queueEvents:
                packets = self.device.getOutputQueue(queueName).tryGetAll()
                if len(packets) > 0:
                    latestPacket[queueName] = packets[-1]
            
            if latestPacket["rgb"] is not None:
                self.frameRgb = latestPacket["rgb"].getCvFrame()
                
            if self.frameRgb is not None:
                                    
                ### -- NOT NAV TASK NEW STUFF FROM 13TH AUGUST TO LIMIT THE DATA RATE OF THE CAMERA -- ###
                # if self.scale < 0.999:
                #     new_w = max(64, int(self.frameRgb.shape[1] * self.scale))
                #     new_h = max(64, int(self.frameRgb.shape[0] * self.scale))
                #     frameRgb = cv2.resize(self.frameRgb, (new_w, new_h), interpolation=cv2.INTER_AREA)
                
                # ok, buf = cv2.imencode('.jpg', self.frameRgb, encode_param)
                # if not ok:
                #     continue
            
                # data = buf.tobytes()
                #self.node.get_logger().info(f"encodeeee paramm: {encode_param}")
                ##########################################

                # Convert encoded bytes to ROS-compressed message
                compressed_msg = CompressedImage()
                compressed_msg.format = "jpeg"
                compressed_msg.data = bytearray(self.frameRgb) # FOR NAV TASK
                # compressed_msg.data = data # NOT FOR NAV TASK
                ### -------------------------------------------------------------------- ###
                self.node.cam_pubs.publish(compressed_msg)
                
                current_time = time.time()
                bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                previous_time = current_time 
                self.node.cam_bw.publish(bw)
                
                self.frameRgb = None

                # ---- NOT NAV TASK Adaptive control ------------------------------------------------
                # size_bytes = len(data)
                # # Smooth the measurement
                # self.ema_bytes = (1 - self.ema_alpha) * self.ema_bytes + self.ema_alpha * size_bytes

                # # Error vs. per-frame budget
                # err = self.ema_bytes - self.bytes_per_frame_budget
                # rel = err / float(self.bytes_per_frame_budget)

                # # Adjust JPEG quality (simple proportional controller)
                # # Negative rel => under budget => increase quality; positive => decrease.
                # k_q = 12.0  # aggressiveness; tune 6–20
                # self.quality -= k_q * rel
                # self.quality = max(self.min_quality, self.quality)
                # #self.node.get_logger().info(f"P controller cam min: {self.quality}")
                # self.quality = max(self.min_quality, min(self.max_quality, int(round(self.quality))))
                # #self.node.get_logger().info(f"P controller cam AFTER: {self.quality}")


                # # If quality bottomed out and still over budget, start downscaling a bit
                # if self.quality <= self.min_quality and self.ema_bytes > 1.15 * self.bytes_per_frame_budget:
                #     # reduce scale by small steps, but not below, say, 0.5
                #     self.scale = max(0.5, self.scale * 0.6)
                # # If comfortably under budget and quality near top, gently upscale back
                # elif self.ema_bytes < 0.7 * self.bytes_per_frame_budget and self.quality >= self.max_quality - 2:
                #     self.scale = min(1.0, self.scale * 1.01)
                # # # ----------------------------------------------------------------------