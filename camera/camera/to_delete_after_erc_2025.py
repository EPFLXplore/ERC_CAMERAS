import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        # Initialize the QRCodeDetector from OpenCV
        self.qr_detector = cv2.QRCodeDetector()
        
        # To be used for any camera
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT: message will attempt to send message but if it fails it will not try again
            durability=QoSDurabilityPolicy.VOLATILE, # VOLATILE: if no subscribers are listening, the message sent is not saved
            history=QoSHistoryPolicy.KEEP_LAST, # KEEP_LAST: only the last n = depth messages are stored in the queue
            depth=1,
        )
        
        # Attributes to store detection results
        self.decoded_info = None  # The decoded text from the QR code
        
        self.rgb_sub = self.create_subscription(CompressedImage, "/NAV/feed_camera_nav_0", self.rgb_callback, qos_profile=self.qos_profile)


    def rgb_callback(self, rgb):
        np_arr = np.frombuffer(rgb.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        data, points, _ = self.qr_detector.detectAndDecode(cv_image)
        self.get_logger().info(f"data: {data}")
    
    
def main(args=None):
    
    rclpy.init(args=args)

    qr_code = QRDetector()
    rclpy.spin(qr_code)

    qr_code.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()