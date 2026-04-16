from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

# 720p @ 30 Hz + JPEG quality for all CS cameras --> 1MB/s per cam
FPS = 30
WIDTH = 1280
HEIGHT = 720
JPEG_QUALITY = 85


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():

    camera_cs_0 = Node(
        package="camera",
        executable="camera",
        name="camera_cs_0",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_0"},
            {"topic_pub": "/ROVER/feed_camera_cs_0"},
            {"bw_pub": "/ROVER/bw_camera_cs_0"},
            {"devrule": "/dev/rover_cam_brio"},
            {"state": "/ROVER/state_camera_cs_0"},
            {"fps": FPS},
            {"x": WIDTH},
            {"y": HEIGHT},
            {"jpeg_quality": JPEG_QUALITY},
        ],
    )

    camera_cs_1 = Node(
        package="camera",
        executable="camera",
        name="camera_cs_1",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_1"},
            {"topic_pub": "/ROVER/feed_camera_cs_1"},
            {"bw_pub": "/ROVER/bw_camera_cs_1"},
            {"devrule": "/dev/video2"},
            {"state": "/ROVER/state_camera_cs_1"},
            {"fps": FPS},
            {"x": WIDTH},
            {"y": HEIGHT},
            {"jpeg_quality": JPEG_QUALITY},
        ],
    )

    camera_cs_2 = Node(
        package="camera",
        executable="camera",
        name="camera_cs_2",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_2"},
            {"topic_pub": "/ROVER/feed_camera_cs_2"},
            {"bw_pub": "/ROVER/bw_camera_cs_2"},
            {"devrule": "/dev/video4"},
            {"state": "/ROVER/state_camera_cs_2"},
            {"fps": FPS},
            {"x": WIDTH},
            {"y": HEIGHT},
            {"jpeg_quality": JPEG_QUALITY},
        ],
    )

    camera_cs_3 = Node(
        package="camera",
        executable="camera",
        name="camera_cs_3",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_3"},
            {"topic_pub": "/ROVER/feed_camera_cs_3"},
            {"bw_pub": "/ROVER/bw_camera_cs_3"},
            {"devrule": "/dev/video6"},
            {"state": "/ROVER/state_camera_cs_3"},
            {"fps": FPS},
            {"x": WIDTH},
            {"y": HEIGHT},
            {"jpeg_quality": JPEG_QUALITY},
        ],
    )

    return LaunchDescription(
        [
            camera_cs_0,
            camera_cs_1,
            camera_cs_2,
            camera_cs_3,
        ]
    )
