from abc import ABC, abstractmethod

class CameraInterface(ABC):
    @abstractmethod
    def publish_feeds(self):
        pass