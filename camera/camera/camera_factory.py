from .cameras.realsense_stereo_camera import RealSenseStereoCamera
from .cameras.mock_monocular_camera import MockMonocularCamera
from .cameras.mock_stereo_camera import MockStereoCamera
from .cameras.monocular_camera import MonocularCamera
from .cameras.oakd_stereo_camera import OakDStereoCamera
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv2_enumerate_cameras import enumerate_cameras

class CameraFactory(Node):

    def __init__(self):
        super().__init__("camera_factory")

        self.cameras = []
        self.cameras_objects = self.get_all_cameras()

        self.publish_ids = self.create_publisher(String, 'test', 1) # todo custom message
        self.timer = self.create_timer(5, self.timer_callback)

    @staticmethod
    def create_camera(node):
        if node.camera_type == "realsense_stereo":
            return RealSenseStereoCamera(node)
        elif node.camera_type == "oakd_stereo":
            return OakDStereoCamera(node)
        elif node.camera_type == "monocular":
            return MonocularCamera(node)
        elif node.camera_type == "mock_stereo":
            return MockStereoCamera(node)
        elif node.camera_type == "mock_monocular":
            return MockMonocularCamera(node)
        else:
            raise ValueError(f"Unknown camera type: {node.camera_type}")
    
    def get_id_vendor():
        pass

    def get_all_cameras(self):
        for camera_info in enumerate_cameras():
            self.get_logger().info(f'{camera_info.pid}:{camera_info.vid} {camera_info.name} {camera_info.backend} {camera_info.index}')

    def timer_callback(self):
        pass # will puslisher the ids

def main(args=None):
    
    rclpy.init(args=args)

    camera_factory = CameraFactory()
    rclpy.spin(camera_factory)

    camera_factory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()