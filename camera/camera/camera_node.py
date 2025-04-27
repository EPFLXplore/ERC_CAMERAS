import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from .camera_factory import CameraFactory
from sensor_msgs.msg import CompressedImage
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, Bool
import time

class CameraNode(Node):
    """
    Create a CameraNode
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
        self.declare_parameter("state", self.default)
        self.declare_parameter("fps", 10)
        self.declare_parameter("x", 640)
        self.declare_parameter("y", 480)
        self.camera_type = self.get_parameter("camera_type").get_parameter_value().string_value
        self.service_topic = self.get_parameter("topic_service").get_parameter_value().string_value
        self.publisher_topic = self.get_parameter("topic_pub").get_parameter_value().string_value
        self.publisher_topic_bw = self.get_parameter("bw_pub").get_parameter_value().string_value
        self.devrule = self.get_parameter("devrule").get_parameter_value().string_value
        self.state_topic = self.get_parameter("state").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value
        self.x = self.get_parameter("x").get_parameter_value().integer_value
        self.y = self.get_parameter("y").get_parameter_value().integer_value

        # To be used for any camera
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT: message will attempt to send message but if it fails it will not try again
            durability=QoSDurabilityPolicy.VOLATILE, # VOLATILE: if no subscribers are listening, the message sent is not saved
            history=QoSHistoryPolicy.KEEP_LAST, # KEEP_LAST: only the last n = depth messages are stored in the queue
            depth=1,
        )

        # Initialize publishers before creating the camera instance
        self.cam_pubs = self.create_publisher(CompressedImage, self.publisher_topic, 1, callback_group=self.callback_group)
        self.cam_bw = self.create_publisher(Float32, self.publisher_topic_bw, 1)
        self.state = self.create_publisher(Bool, self.state_topic, 1)

         # object camera
        self.camera =  CameraFactory.create_camera(self)
        self.stopped = True

        # Service to activate the camera. For now we hardcode the parameters so we use just a SetBool
        self.service_activation = self.create_service(SetBool, self.service_topic, self.start_cameras_callback, callback_group=self.callback_group)

        self.thread = threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,))
    
        self.get_logger().info("Cameras ready")
    
    def start_cameras_callback(self, request, response):
        if request.data:
            self.stopped = False

            # Only start the thread if it's not already running
            if not hasattr(self, 'thread') or not self.thread.is_alive():
                if self.camera_type == "oakd_stereo": 
                    self.thread = threading.Thread(target=self.camera.publish_feeds)
                elif self.camera_type == "realsense_stereo":
                    self.thread = threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,))
                else:
                    self.thread = threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,))
                
                self.get_logger().info("before starting thread")
                self.thread.start()
                response.success = True
                response.message = "Cameras started"
                msg = Bool()
                msg.data = True
                self.state.publish(msg)
            else:
                response.success = False
                response.message = "Cameras are already running"
        else:
            # Stop the thread only if it has been started and is still running
            if hasattr(self, 'thread') and self.thread.is_alive():
                self.stopped = True
                self.thread.join()
                response.success = True
                response.message = "Cameras stopped"
                msg = Bool()
                msg.data = False
                self.state.publish(msg)
            else:
                response.success = False
                response.message = "No camera thread to stop"

        return response

    def calculate_bandwidth(self, current_time, previous_time, compressed_image_len):
        elapsed_time = current_time - previous_time
        bw = Float32()
        bw.data = float(compressed_image_len * 8) / (elapsed_time * 1_000_000) # Bandwidth in Mbps
        
        return bw

def main(args=None):
    
    rclpy.init(args=args)

    cameras_publisher = CameraNode()
    rclpy.spin(cameras_publisher)

    cameras_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()