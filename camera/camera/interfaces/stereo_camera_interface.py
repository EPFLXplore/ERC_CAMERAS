from .monocular_camera_interface import (
    MonocularCameraInterface,
)
from abc import abstractmethod

class StereoCameraInterface(MonocularCameraInterface):
    @abstractmethod
    def get_depth(self):
        pass

    @abstractmethod
    def get_intrinsics(self):
        pass

    @abstractmethod
    def get_coeffs(self):
        pass

    @abstractmethod
    def get_rgbd(self):
        pass

# def set_camera_mode(mode):
#     # ROS2 client to call the set_camera_mode service
#     client = node.create_client(SetMode, 'set_camera_mode')
#     request = SetMode.Request()
#     request.mode = mode  # "RGB" or "RGB-D"
#     future = client.call_async(request)