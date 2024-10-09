from abc import ABC, abstractmethod

from .camera_interface import CameraInterface

class MonocularCameraInterface(CameraInterface):
    @abstractmethod
    def get_rgb(self):
        pass