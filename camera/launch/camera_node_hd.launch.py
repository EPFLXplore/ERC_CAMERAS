
from launch import LaunchDescription
from launch.events import matches_action
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node as RosNode
from launch_ros.descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
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

    declare_current_resolution = DeclareLaunchArgument(
        "start_resolution",
        default_value="1080P",
        description="Initial camera resolution (1080P or 4K), settable from the parent launch file.",
    )

    #Don't forget to update in kinematics/hd_fsm/hd_fsm/hds_interface
    camera_hd_gripper = LifecycleNode(
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
            {'topic_resolution': hd_names['hd_camera_resolution']},
            {'bw_pub': "/HD/bw_camera_hd_0"}, 
            {'info': rover_names['rover_hd_camera_info']},
            {'state': "/ROVER/state_camera_hd_0"},
            {'depth_req': "/ROVER/depth_req_camera_hd_0"}, # To activate the depth
            {'state_depth': "/ROVER/state_depth_camera_hd_0"},
            {'fps_internal': [30, 15]},
            {'fps_external': [30, 15]},
            {'rgb_resolution': ["1080P", "4K"]},
            {'mono_resolution': ["480P", "480P"]},
            {'flip_camera': False},
            {'fps_depth': [1, 12]},  
            {'number_of_frames_to_average': [1, 4]},
            {'current_resolution': LaunchConfiguration('start_resolution')},
            {'health_check_period_sec': 1.0},
            {'health_timeout_sec': 3.0},
        ],
        output='screen',
    )

    gst_camera_bridge = RosNode(
        package='camera',
        executable='gst_camera_bridge',
        name='gst_hd_camera_bridge',
        namespace='/HD',
        parameters=[
            {'mode': "hd"},
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

    def change_state(node, transition):
        return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=transition))

    # configure as soon as the node is up (launch_ros waits for its change_state service)
    configure = change_state(camera_hd_gripper, Transition.TRANSITION_CONFIGURE)

    # activate the instant configure succeeds
    activate_when_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=camera_hd_gripper,
        start_state='configuring', goal_state='inactive',
        entities=[change_state(camera_hd_gripper, Transition.TRANSITION_ACTIVATE)],
    ))

    return LaunchDescription(
        [
            declare_gst_width,
            declare_gst_height,
            declare_gst_fps,
            declare_gst_bitrate,
            declare_current_resolution,
            camera_hd_gripper,
            activate_when_inactive,
            configure,
            gst_camera_bridge_delayed,
        ]
    )