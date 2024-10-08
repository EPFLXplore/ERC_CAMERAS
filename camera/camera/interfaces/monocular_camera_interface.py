from abc import ABC, abstractmethod

class MonocularCameraInterface(ABC):
    @abstractmethod
    def get_image(self):
        pass

    @abstractmethod
    def get_rgb(self):
        pass

    @abstractmethod
    def start_cameras_callback(self, request, response):
        pass
