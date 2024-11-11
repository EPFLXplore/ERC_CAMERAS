import rclpy
from rclpy.node import Node
import time
import sys
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class BandwidthMonitor(Node):
    def __init__(self, topic_name_1, topic_name_2, topic_name_3, topic_type):
        super().__init__('camera_health_checker')
        self.get_logger().info("Camera health checker ready")

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.subscription = self.create_subscription(
            topic_type,
            topic_name_1,
            self.listener_callback,
            self.qos_profile
        )
        self.total_bytes = 0
        self.start_time = time.time()
        self.bandwidth = 0.0

    def listener_callback(self, msg):
        # Estimate the size of the message in bytes
        message_size = sys.getsizeof(msg)
        self.total_bytes += message_size

        # Calculate elapsed time and bandwidth
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0:
            self.bandwidth = (self.total_bytes * 8) / elapsed_time / 1e6  # Convert to Mbps

        # Print bandwidth every 1 second or so
        if elapsed_time >= 0:
            self.get_logger().info(f"Current bandwidth: {self.bandwidth:.7f} Mbps")
            # Reset counters
            self.start_time = time.time()
            self.total_bytes = 0

def main(args=None):
    rclpy.init(args=args)
    node = BandwidthMonitor('/ROVER/feed_camera_cs_0', '/ROVER/feed_camera_cs_1', '/ROVER/feed_camera_cs_2', CompressedImage)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()