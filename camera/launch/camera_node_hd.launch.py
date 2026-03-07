
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
import yaml


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    with open('/home/xplore/dev_ws/src/custom_msg/config/rover_interface_names.yaml', 'r') as file:
        rover_names = yaml.safe_load(file)["/**"]["ros__parameters"]
    camera_hd_gripper = Node(
        package='camera',
        executable='camera',
        name='camera_hd_gripper',
        namespace='/HD',
        parameters=[
            {'camera_type': "oakd_stereo"},
            {'topic_service': "/ROVER/req_camera_hd_0"},
            {'topic_pub': rover_names['rover_hd_rgb_feed']}, # we concatenate the devrule"},
            {'depth': "/ROVER/depth_camera_hd_0"},
            {'depth_avg' : rover_names['rover_hd_depth_avg']},
            {'bw_pub': "/HD/bw_camera_hd_0"},
            {'devrule': ""}, # serial number written on the back of the camera
            {'info': rover_names['rover_hd_camera_info']}, # we concatenate the devrule
            {'depth_req': "/ROVER/depth_req_camera_hd_0"}, # To activate the depth
            {'state_depth': "/ROVER/state_depth_camera_hd_0"},
            {'fps': 30},
            {'x': 1920},
            {'y': 1080},
            {'flip_camera': True},
            {'fps_depth': 15}
        ],
    )
    return LaunchDescription(
        [
            camera_hd_gripper
        ]
    )
