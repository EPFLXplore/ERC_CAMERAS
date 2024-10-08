# Import the necessary libraries
import rclpy
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from custom_msg.msg import CompressedRGBD
from custom_msg.srv import CameraParams
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


import yaml
from .camera_factory import CameraFactory
from .get_interfaces import GetInterfaces

class CameraNode(Node):
    """
    Create an CameraNode class
    """

    def __init__(self):
        super().__init__("camera_node")

        # parameters
        self.declare_parameter("camera_type")
        camera_type = self.get_parameter("camera_type").get_parameter_value().string_value

         # object camera
        self.camera =  CameraFactory.create_camera(self, camera_type)
        timer_period = 1.0 / self.camera.fps

        # Service to activate the camera. For now we hardcode the parameters so we use just a SetBool
        self.service_activation = self.create_service(SetBool, self.camera.topic, self.camera.start_cameras_callback)

        '''
        if self.config['publish_straigth_to_cs']:
            self.publisher_ = self.create_publisher(CompressedImage, GetInterfaces.get('hd_camera_rgb'), 1)
            self.timer = self.create_timer(timer_period, self.rgb_callback)
        else:
            self.publisher_ = self.create_publisher(CompressedRGBD, GetInterfaces.get('hd_camera_rgbd'), 1)
            self.timer = self.create_timer(timer_period, self.rgbd_callback)
        
        self.publisher = self.create_publisher(CompressedImage, self.rover_names['rover_cameras_cs_prefix'] + str(i), qos_profile) for i in range(len(self.camera_ids))

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        '''
        self.get_logger().info("Cameras ready")

    def rgbd_callback(self):
        """
        Callback function.
        Publishes a frame to the video_frames topic
        """
        color, depth = self.camera.get_rgbd()

        msg = CompressedRGBD()

        depth_msg = self.bridge.cv2_to_imgmsg(depth, "mono16")
        msg.depth = depth_msg
        msg.color = self.bridge.cv2_to_compressed_imgmsg(color)

        self.publisher_.publish(msg)


    def rgb_callback(self):
        """
        Callback function.
        Publishes a frame to the video_frames topic
        """
        color = self.camera.get_rgb()
        msg = self.bridge.cv2_to_compressed_imgmsg(color)

        self.publisher_.publish(msg)
    
    def load_config(self):
        with open(self.config_path, "r") as file:
            config = yaml.safe_load(file)['/**']['ros__parameters']
        
        return config