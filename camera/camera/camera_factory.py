from .cameras.realsense_stereo_camera import RealSenseStereoCamera
from .cameras.mock_monocular_camera import MockMonocularCamera
from .cameras.mock_stereo_camera import MockStereoCamera
from .cameras.monocular_camera import MonocularCamera
from .cameras.oakd_stereo_camera import OakDStereoCamera
import yaml
import rclpy
from rclpy.node import Node

class CameraFactory(Node):

    def __init__(self):
        super.__init__("camera_factory")

        self.cameras = []
        self.cameras_objects = self.get_all_cameras()

    @staticmethod
    def create_camera(node, camera_type: str):
        if camera_type == "realsense_stereo":
            return RealSenseStereoCamera(node)
        elif camera_type == "oakd_stereo":
            return OakDStereoCamera(node)
        elif camera_type == "monocular":
            return MonocularCamera(node)
        elif camera_type == "mock_stereo":
            return MockStereoCamera(node)
        elif camera_type == "mock_monocular":
            return MockMonocularCamera(node)
        else:
            raise ValueError(f"Unknown camera type: {camera_type}")
    
    def get_id_vendor():
        pass

    def get_all_cameras(self):
        pass