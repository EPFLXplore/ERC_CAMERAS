
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node as RosNode
from launch_ros.descriptions import ParameterValue
import os
from ament_index_python import get_package_share_directory
import yaml


def _lifecycle_configure_cmd(node_fqn: str) -> list:
    script = f"""
set -u
N={node_fqn!r}
ready=""
for _ in $(seq 1 60); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  if [[ "$s" == inactive* || "$s" == active* ]]; then
    echo "[camera_node_hd] configure: $N already $s"
    exit 0
  fi
  if [[ "$s" == unconfigured* ]]; then
    ready=1
    break
  fi
  sleep 0.25
done
[[ -n "$ready" ]] || {{ echo "[camera_node_hd] configure: timeout waiting for $N" >&2; exit 1; }}
for _ in $(seq 1 30); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && exit 0
  [[ "$s" == unconfigured* ]] || {{ sleep 0.4; continue; }}
  ros2 lifecycle set "$N" configure >/dev/null 2>&1 || true
  sleep 0.5
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && exit 0
done
echo "[camera_node_hd] configure: gave up on $N" >&2
exit 1
"""
    return ["bash", "-lc", script]


def _lifecycle_activate_cmd(node_fqn: str) -> list:
    script = f"""
set -u
N={node_fqn!r}
for _ in $(seq 1 60); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && break
  [[ "$s" == active* ]] && exit 0
  sleep 0.2
done
for _ in $(seq 1 30); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == active* ]] && exit 0
  [[ "$s" == inactive* ]] || {{ sleep 0.3; continue; }}
  ros2 lifecycle set "$N" activate >/dev/null 2>&1 || true
  sleep 0.35
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == active* ]] && exit 0
done
echo "[camera_node_hd] activate: gave up on $N" >&2
exit 1
"""
    return ["bash", "-lc", script]



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

    #Don't forget to update in kinematics/hd_fsm/hd_fsm/hds_interface (if the modes are changed)
    #During probing we don't need depth and we don't want res switch
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
            {'fps_internal': [30, 30]},
            {'fps_external': [30, 30]},
            {'rgb_resolution': ["1080P", "4K"]},
            {'mono_resolution': ["480P", "480P"]},
            {'flip_camera': False},
            {'fps_depth': [0, 0]},  
            {'number_of_frames_to_average': [0, 0]},
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

    node_fqn = '/HD/camera_hd_gripper'
    configure_camera = ExecuteProcess(
        cmd=_lifecycle_configure_cmd(node_fqn),
        output='screen',
    )
    activate_camera = ExecuteProcess(
        cmd=_lifecycle_activate_cmd(node_fqn),
        output='screen',
    )

    return LaunchDescription(
        [
            declare_gst_width,
            declare_gst_height,
            declare_gst_fps,
            declare_gst_bitrate,
            declare_current_resolution,
            camera_hd_gripper,
            configure_camera,
            activate_camera,
            gst_camera_bridge_delayed,
        ]
    )