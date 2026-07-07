
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
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

    with open('/home/xplore/dev_ws/src/custom_msg/config/hd_interface_names.yaml', 'r') as file:
            hd_names = yaml.safe_load(file)["/**"]["ros__parameters"]

    declare_gst_host = DeclareLaunchArgument(
        "gst_host",
        default_value="169.254.55.166",
        description="Control station IP address for the HD GStreamer video bridge.",
    )
    declare_gst_port = DeclareLaunchArgument(
        "gst_port",
        default_value="5013",
        description="UDP port for the HD GStreamer video bridge.",
    )
    declare_gst_width = DeclareLaunchArgument(
        "gst_width",
        default_value="854",
        description="GStreamer encoding width (resolution fed to x264enc).",
    )
    declare_gst_height = DeclareLaunchArgument(
        "gst_height",
        default_value="480",
        description="GStreamer encoding height.",
    )
    declare_gst_fps = DeclareLaunchArgument(
        "gst_fps",
        default_value="15",
        description="GStreamer encoding framerate.",
    )
    declare_gst_bitrate = DeclareLaunchArgument(
        "gst_bitrate",
        default_value="1000",
        description="x264 target bitrate in kbps (adjustable at runtime via ros2 param set).",
    )

    camera_hd_gripper = Node(
        package='camera',
        executable='camera',
        name='camera_hd_gripper',
        namespace='/HD',
        parameters=[
            {'camera_type': "oakd_stereo"},
            {'topic_service': "/ROVER/req_camera_hd_0"},
            {'topic_internal_pub': hd_names['hd_internal_camera_rgb']},
            {'topic_pub': rover_names['rover_hd_rgb_feed']},
            {'depth': "/ROVER/depth_camera_hd_0"},
            {'depth_avg' : rover_names['rover_hd_depth_avg']},
            {'bw_pub': "/HD/bw_camera_hd_0"}, 
            {'devrule': ""},  # serial number written on the back of the camera
            {'info': rover_names['rover_hd_camera_info']}, # we concatenate the devrule
            {'depth_req': "/ROVER/depth_req_camera_hd_0"}, # To activate the depth
            {'state_depth': "/ROVER/state_depth_camera_hd_0"},
            {'fps': 30},
            {'fps_external': 20},
            {'x': 1920},
            {'y': 1080},
            {'flip_camera': False},
            {'fps_depth': 15},  
            {'number_of_frames_to_average': 5}
        ],
    )

    gst_camera_bridge = Node(
        package='camera',
        executable='gst_camera_bridge',
        name='gst_hd_camera_bridge',
        namespace='/HD',
        parameters=[
            {'topic': hd_names['hd_internal_camera_rgb']},
            {'host': ParameterValue(LaunchConfiguration('gst_host'), value_type=str)},
            {'port': ParameterValue(LaunchConfiguration('gst_port'), value_type=int)},
            {'width': ParameterValue(LaunchConfiguration('gst_width'), value_type=int)},
            {'height': ParameterValue(LaunchConfiguration('gst_height'), value_type=int)},
            {'fps': ParameterValue(LaunchConfiguration('gst_fps'), value_type=int)},
            {'bitrate': ParameterValue(LaunchConfiguration('gst_bitrate'), value_type=int)},
        ],
        output='screen',
    )
    # Give camera_hd_gripper time to come up and start publishing before the
    # bridge subscribes, same staggering pattern used by the NAV/CS bridges.
    gst_camera_bridge_delayed = TimerAction(period=5.0, actions=[gst_camera_bridge])

    return LaunchDescription(
        [
            declare_gst_host,
            declare_gst_port,
            declare_gst_width,
            declare_gst_height,
            declare_gst_fps,
            declare_gst_bitrate,
            camera_hd_gripper,
            gst_camera_bridge_delayed,
        ]
    )