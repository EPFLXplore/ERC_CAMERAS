import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from .camera_factory import CameraFactory
from sensor_msgs.msg import CompressedImage
import cv2, threading
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32


class CameraNode(Node):
    """
    Create a CameraNode class
    """

    def __init__(self):
        super().__init__("camera_node")

        self.callback_group = ReentrantCallbackGroup()
        self.default = ""

        # parameters
        self.declare_parameter("camera_type", self.default)
        self.declare_parameter("topic_service", self.default)
        self.declare_parameter("topic_pub", self.default)
        self.declare_parameter("bw_pub", self.default)
        self.declare_parameter("devrule", self.default)
        self.camera_type = self.get_parameter("camera_type").get_parameter_value().string_value
        self.service_topic = self.get_parameter("topic_service").get_parameter_value().string_value
        self.publisher_topic = self.get_parameter("topic_pub").get_parameter_value().string_value
        self.publisher_topic_bw = self.get_parameter("bw_pub").get_parameter_value().string_value
        self.devrule = self.get_parameter("devrule").get_parameter_value().string_value

         # object camera
        self.camera =  CameraFactory.create_camera(self)
        self.stopped = True

        # Service to activate the camera. For now we hardcode the parameters so we use just a SetBool
        self.service_activation = self.create_service(SetBool, self.service_topic, self.start_cameras_callback)
        self.cam_pubs = self.create_publisher(CompressedImage, self.publisher_topic, qos_profile=self.camera.qos_profile, callback_group=self.callback_group)
        self.cam_bw = self.create_publisher(Float32, self.publisher_topic_bw, 10)

        self.thread = threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,))

        self.get_logger().info("Cameras ready")
    
    def start_cameras_callback(self, request, response):
        if request.data:
            self.stopped = False # the timer will start sending inside the camera object
            self.thread = threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,))
            self.thread.start()
            response.success = True
            response.message = "Cameras started"
        else:
            self.stopped = True # the timer will stop sending inside the camera object
            self.thread.join()
            response.success = True
            response.message = "Cameras stopped"

        return response


def main(args=None):
    
    rclpy.init(args=args)

    cameras_publisher = CameraNode()
    rclpy.spin(cameras_publisher)

    cameras_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()