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
            {'topic_service': "/NAV/req_camera_nav_0"},
            {'topic_pub': "/NAV/feed_camera_nav_0"},
            {'bw_pub': "/NAV/bw_camera_nav_0"}, 
            #the devrule is already in the dockerfile
        ],
    )

    nav_realsense_aruco_camera_left = Node(
        package='camera',
        executable='camera',
        name='camera_aruco_left',
        namespace='/NAV',
        parameters=[
            {'camera_type': "realsense_stereo"},
            {'topic_service': "/NAV/req_camera_nav_1"},
            {'topic_pub': "/NAV/feed_camera_nav_1"},
            {'bw_pub': "/NAV/bw_camera_nav_1"}, 
            {'devrule': "102122061110"}
        ],
    )

    nav_realsense_aruco_camera_right = Node(
        package='camera',
        executable='camera',
        name='camera_aruco_right',
        namespace='/NAV',
        parameters=[
            {'camera_type': "realsense_stereo"},
            {'topic_service': "/NAV/req_camera_nav_2"},
            {'topic_pub': "/NAV/feed_camera_nav_2"},
            {'bw_pub': "/NAV/bw_camera_nav_2"}, 
            {'devrule': "135322062945"}
        ],
    )

    return LaunchDescription(
        [
            nav_realsense_aruco_camera_left,
            nav_realsense_aruco_camera_right,
            
            #nav_front_camera
        ]
    )
