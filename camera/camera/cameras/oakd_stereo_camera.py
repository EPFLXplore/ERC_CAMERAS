import depthai as dai
import cv2
import time
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from custom_msg.srv import CameraParams
from custom_msg.msg import CompressedRGBD
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image



class OakDStereoCamera():
    def __init__(self, node):
        # raise NotImplementedError(f"__init__ not implemented for {self._name}")
        self.node = node
        self.bridge = CvBridge()

        self.serial_number = self.node.devrule 
        self.node.declare_parameter("info", self.node.default)
        self.info = self.node.get_parameter("info").get_parameter_value().string_value
        self.node.declare_parameter("depth_req", self.node.default)
        self.depth_request = self.node.get_parameter("depth_req").get_parameter_value().string_value

        self.depth_change = self.node.create_service(SetBool, self.depth_request, self.depth_callback)
        self.depth_mode = 0

        # Service to retrieve the parameters of the camera
        self.camera_info_service = self.node.create_service(CameraParams, self.info + self.serial_number, self.camera_params_callback)

        self.pipeline = dai.Pipeline()

        # mono = black and white image from the left and right cams

        self.mono_left = self.pipeline.createMonoCamera()
        self.mono_right = self.pipeline.createMonoCamera()

        self.mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        self.mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        self.mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)


        # ColorCamera = rgb output from the oakd  
        self.color_cam = self.pipeline.createColorCamera()
        self.color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        if self.node.x == 1280:
            self.color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
            
        # THIS DOES NOT EXIST IN THE OAKD
        elif self.node.x == 640:
            self.color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_480_P)
            
        self.color_cam.setFps(self.node.fps)
        self.color_cam.setInterleaved(False)  
        # Non-interleaved frames for better compatibility with ros2 and opencv

        # stereo depth
        self.stereo = self.pipeline.createStereoDepth()
        self.stereo.setOutputDepth(True)
        self.stereo.setOutputRectified(True)
        self.stereo.setConfidenceThreshold(200)

        # Link Mono Cameras to Stereo Depth
        self.mono_left.out.link(self.stereo.left)
        self.mono_right.out.link(self.stereo.right)

        # Link ColorCamera video output to XLinkOut for streaming rgb frames
        self.rgb_out = self.pipeline.createXLinkOut()
        self.rgb_out.setStreamName("rgb")
        self.color_cam.video.link(self.rgb_out.input)

        # Link StereoDepth depth output to XLinkOut for streaming
        self.depth_out = self.pipeline.createXLinkOut()
        self.depth_out.setStreamName("depth")
        self.stereo.depth.link(self.depth_out.input)

        # try to connect to oakd
        try:
            self.device = dai.Device(self.pipeline)
            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            self.node.get_logger().info("OAK-D pipeline started successfully.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to initialize OAK-D: {e}")
            raise

        self.depth_pubs = self.node.create_publisher(CompressedRGBD, self.node.publisher_topic + "/depth", 1)

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
        if rgb_frame is None:
            #self.node.get_logger().warn("No RGB frame received.")
            return None
        return rgb_frame.getCvFrame()

    def get_rgbd(self):
        """Retrieve both RGB and Depth frames."""
        rgb_frame = self.get_rgb()
        depth_frame = self.get_depth()
        if rgb_frame is None or depth_frame is None:
            #self.node.get_logger().warn("No RGB or Depth frame received.")
            return None, None
        return rgb_frame, depth_frame

    def get_depth(self):
        """Retrieve a Depth frame from the Depth queue."""
        depth_frame = self.depth_queue.tryGet()
        if depth_frame is None:
            #self.node.get_logger().warn("No depth frame received.")
            return None
        return depth_frame.getFrame()

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

        previous_time = 0
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 10]  # Lower quality to save bandwidth
        while not self.node.stopped:

            compressed_msg = None

            if self.depth_mode == True or self.depth_mode == 1:
                rgb_frame, depth_img = self.get_rgbd()

                if depth_img is not None and rgb_frame is not None:

                    success, encoded_image = cv2.imencode('.jpg', rgb_img, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue

                    # Convert encoded bytes to ROS-compressed message
                    compressed_msg = CompressedImage()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)

                    depth_img_normalized = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)


                    msg = Image()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.height = depth_img_normalized.shape[0]
                    msg.width = depth_img_normalized.shape[1]
                    msg.encoding = "mono16"  # Encoding for uint16 depth images
                    msg.is_bigendian = False
                    msg.step = msg.width * 2  # 2 bytes per pixel
                    msg.data = depth_img_normalized.tobytes()
                    
                    full_msg = CompressedRGBD()
                    full_msg.color = compressed_msg
                    full_msg.depth = msg
                    # self.color_depth_pub.publish(full_msg)
                    self.depth_pubs.publish(full_msg)

                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data) + len(depth_img_normalized.tobytes()))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)

            else:

                rgb_img = self.get_rgb()
                if rgb_img is not None:

                    # Compress RGB frame with lower quality
                    success, encoded_image = cv2.imencode('.jpg', rgb_img, encode_param)
                    if not success:
                        self.node.get_logger().warn("Failed to compress RGB frame.")
                        continue

                    # Convert encoded bytes to ROS-compressed message
                    compressed_msg = CompressedImage()
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.node.cam_pubs.publish(compressed_msg)

                    current_time = time.time()
                    bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_msg.data))
                    previous_time = current_time 
                    self.node.cam_bw.publish(bw)

        self.device.close()