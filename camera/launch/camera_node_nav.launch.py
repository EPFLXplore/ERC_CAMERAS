#Author: Arno Laurie
#Date: 22/11/2024

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

    nav_front_camera = Node(
        package='camera',
        executable='camera',
        name='camera_nav_front',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oakd_stereo"},
            {'topic_service': "/ROVER/req_front_camera_nav"},
            {'topic_pub': "/ROVER/feed_front_camera_nav"},
            {'bw_pub': "/ROVER/bw_front_camera_nav"}, 
            {'devrule': "/dev/video3"}
        ],
    )
    return LaunchDescription(
        [
            nav_front_camera
        ]
    )
