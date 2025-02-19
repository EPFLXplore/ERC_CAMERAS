from .cameras.mock_monocular_camera import MockMonocularCamera
from .cameras.mock_stereo_camera import MockStereoCamera
from .cameras.monocular_camera import MonocularCamera
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraFactory():
            
    @staticmethod
    def create_camera(node):
        if node.camera_type == "realsense_stereo":
            try:
                from .cameras.realsense_stereo_camera import RealSenseStereoCamera
                return RealSenseStereoCamera(node)
            except ModuleNotFoundError:
                node.get_logger().error("RealSenseStereoCamera requires 'pyrealsense2' which is not installed.")
                raise

        elif node.camera_type == "oakd_stereo":
            try:
                from .cameras.oakd_stereo_camera import OakDStereoCamera
                return OakDStereoCamera(node)
            except ModuleNotFoundError:
                node.get_logger().error("OakDStereoCamera requires 'depthai' which is not installed.")
                raise

        elif node.camera_type == "monocular":
            return MonocularCamera(node)
        elif node.camera_type == "mock_stereo":
            return MockStereoCamera(node)
        elif node.camera_type == "mock_monocular":
            return MockMonocularCamera(node)
        else:
            raise ValueError(f"Unknown camera type: {node.camera_type}")
