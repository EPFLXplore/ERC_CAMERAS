from .cameras.monocular_camera import MonocularCamera

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
        elif node.camera_type == "oak1w_stereo":
            try:
                from .cameras.oak1w_stereo_camera import Oak1WStereoCamera
                # node.get_logger().error("Oak1WStereoCamera being setup.")
                return Oak1WStereoCamera(node)
            except ModuleNotFoundError:
                node.get_logger().error("Oak1WStereoCamera requires 'depthai' which is not installed.")
                raise

        elif node.camera_type == "oakd_stereo":
            try:
                from .cameras.oakd_stereo_camera import OakDStereoCamera
                return OakDStereoCamera(node)
            except ModuleNotFoundError as e:
                node.get_logger().error(
                    "OakDStereoCamera import failed (missing oakd_stereo_camera.py in this "
                    f"package and/or missing depthai). Details: {e}"
                )
                raise
        elif node.camera_type == "monocular":
            return MonocularCamera(node)
        else:
            raise ValueError(f"Unknown camera type: {node.camera_type}")
