#Author: Arno Laurie
#Date: 22/11/2024

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
from launch.actions import TimerAction


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
            {'depth': "/NAV/depth_camera_nav_0"},
            {'bw_pub': "/NAV/bw_camera_nav_0"},
            {'depth_req': "/NAV/depth_req_camera_nav_0"},
            {'devrule': ""},
            {'info': "/NAV/camera_info_"}, # we concatenate the devrule
            {'state': "/NAV/state_camera_nav_0"},
            {'screenshot': '/NAV/screenshot_camera_nav_0'},
            {'fps': 15},
            {'x': 1280},
            {'y': 720},
            {'flip_camera':False}
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
            {'devrule': "102122061110"},
            {'info': "/NAV/camera_info_"}, # we concatenate the devrule
            {'depth_req': "/NAV/depth_req_camera_nav_1"},
            {'state': "/NAV/state_camera_nav_1"},
            {'fps': 15},
            {'x': 1280},
            {'y': 720}
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
            {'devrule': "135322062945"},
            {'info': "/NAV/camera_info_"}, # we concatenate the devrule
            {'depth_req': "/NAV/depth_req_camera_nav_2"},
            {'state': "/NAV/state_camera_nav_2"},
            {'fps': 15},
            {'x': 1280},
            {'y': 720}
        ],
    )

    nav_test_camera = Node(
        package='camera',
        executable='camera',
        name='camera_nav_test',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oak1w_stereo"},
            {'topic_service': "/NAV/req_camera_nav_3"},
            {'topic_pub': "/NAV/feed_camera_nav_3"},
            {'depth': "/NAV/depth_camera_nav_3"},
            {'bw_pub': "/NAV/bw_camera_nav_3"},
            {'depth_req': "/NAV/depth_req_camera_nav_3"},
            {'devrule': "19443010714B177E00"}, # To get with print("Found Oak1W Stereo Camera : ", dai.Device.getAllAvailableDevices()) under the name : "deviceId"
            {'info': "/NAV/camera_info_"}, # we concatenate the devrule
            {'state': "/NAV/state_camera_nav_3"},
            {'screenshot': '/NAV/screenshot_camera_nav_3'},
            {'fps': 30},
            {'x': 1280},
            {'y': 720},
            {'flip_camera':False}
        ],
    )

    return LaunchDescription([
        nav_front_camera,
        TimerAction(period=2.0, actions=[nav_realsense_aruco_camera_left]),
        TimerAction(period=4.0, actions=[nav_realsense_aruco_camera_right]),
    ])