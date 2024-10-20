import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )


        # Create a subscriber to the camera topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/HD/feed_camera_hd_rgb',  # Change this to match your topic
            self.image_callback,
            self.qos_profile # QoS history depth
        )
        
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the compressed image to an OpenCV image
        image = self.bridge.compressed_imgmsg_to_cv2(msg)


        # Display the image
        cv2.imshow('Camera Image', image)
        cv2.waitKey(1)  # Required to update the OpenCV window

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    main()
