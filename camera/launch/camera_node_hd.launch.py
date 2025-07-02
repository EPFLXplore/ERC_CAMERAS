
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

    camera_hd_gripper = Node(
        package='camera',
        executable='camera',
        name='camera_hd_gripper',
        namespace='/HD',
        parameters=[
            {'camera_type': "oakd_stereo"},
            {'topic_service': "/ROVER/req_camera_hd_0"},
            {'topic_pub': "/ROVER/feed_camera_hd_0"},
            {'depth': "/ROVER/depth_camera_hd_0"},
            {'bw_pub': "/HD/bw_camera_hd_0"}, 
            {'devrule': "218622278049"},  # serial number written on the back of the camera
            {'info': "/ROVER/camera_info_"}, # we concatenate the devrule
            {'depth_req': "/ROVER/depth_req_camera_hd_0"}, # To activate the depth
            {'state': "/ROVER/state_camera_hd_0"},
            {'fps': 10},
            {'x': 1280},
            {'y': 720},
            {'flip_camera': False},
            {'fps_depth': 10}
        ],
    )
    return LaunchDescription(
        [
            camera_hd_gripper
        ]
    )
