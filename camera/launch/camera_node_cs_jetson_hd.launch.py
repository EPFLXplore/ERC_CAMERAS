from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():

    camera_cs_4 = Node(
        package='camera',
        executable='camera',
        name='camera_cs_4',
        namespace='/ROVER',
        parameters=[
            {'camera_type': "monocular"},
            {'topic_service': "/ROVER/req_camera_cs_4"},
            {'topic_pub': "/ROVER/feed_camera_cs_4"},
            {'bw_pub': "/ROVER/bw_camera_cs_4"}, 
            {'devrule': "/dev/video0"},
            {'state': "/ROVER/state_camera_cs_4"},
            {'fps': 15},
            {'x': 640},
            {'y': 480}
        ],
    )
    
    camera_cs_5 = Node(
        package='camera',
        executable='camera',
        name='camera_cs_5',
        namespace='/ROVER',
        parameters=[
            {'camera_type': "monocular"},
            {'topic_service': "/ROVER/req_camera_cs_5"},
            {'topic_pub': "/ROVER/feed_camera_cs_5"},
            {'bw_pub': "/ROVER/bw_camera_cs_5"},
            {'devrule': "/dev/video2"},
            {'state': "/ROVER/state_camera_cs_5"},
            {'fps': 15},
            {'x': 640},
            {'y': 480}
        ],
    )

    
    return LaunchDescription(
        [
            camera_cs_4,
            camera_cs_5
        ]
    )
