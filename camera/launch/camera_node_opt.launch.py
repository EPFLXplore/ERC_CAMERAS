# Oak1W nav cameras: high resolution (default 1280×720), low MJPEG quality for bandwidth.
# Publishes CompressedImage on /CS/feed_camera_nav_{0,1,2}.
# Camera 3 is Oak-D hardware (same MXID as NAV); driven via oak1w_stereo (no oakd_stereo_camera in repo).
#
# Select resolution / compression / rate from the command line, e.g.:
#   ros2 launch camera camera_node_opt.launch.py opt_jpeg_quality:=35 opt_width:=1280 opt_height:=720 opt_fps:=5
#
# After editing this file you must rebuild the camera package (or launch the file
# by absolute path from src) or `ros2 launch camera …` keeps using install/share.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node as RosNode


def _lifecycle_configure_cmd(node_fqn: str) -> list:
    """Wait for the lifecycle node, then retry configure until inactive (CLI may exit 0 on failure)."""
    script = f"""
set -u
N={node_fqn!r}
ready=""
for _ in $(seq 1 480); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  if [[ "$s" == inactive* || "$s" == active* ]]; then
    echo "[camera_node_opt] configure: $N already $s"
    exit 0
  fi
  if [[ "$s" == unconfigured* ]]; then
    ready=1
    break
  fi
  sleep 0.25
done
[[ -n "$ready" ]] || {{ echo "[camera_node_opt] configure: timeout waiting for $N" >&2; exit 1; }}
for _ in $(seq 1 120); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && exit 0
  [[ "$s" == unconfigured* ]] || {{ sleep 0.4; continue; }}
  ros2 lifecycle set "$N" configure >/dev/null 2>&1 || true
  sleep 0.5
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && exit 0
done
echo "[camera_node_opt] configure: gave up on $N" >&2
exit 1
"""
    return ["bash", "-lc", script]


def _lifecycle_activate_cmd(node_fqn: str) -> list:
    script = f"""
set -u
N={node_fqn!r}
for _ in $(seq 1 200); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && break
  [[ "$s" == active* ]] && exit 0
  sleep 0.2
done
for _ in $(seq 1 80); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == active* ]] && exit 0
  [[ "$s" == inactive* ]] || {{ sleep 0.3; continue; }}
  ros2 lifecycle set "$N" activate >/dev/null 2>&1 || true
  sleep 0.35
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == active* ]] && exit 0
done
echo "[camera_node_opt] activate: gave up on $N" >&2
exit 1
"""
    return ["bash", "-lc", script]


def _lifecycle_call_setbool_cmd(node_fqn: str, service_abs: str) -> list:
    script = f"""
set -u
N={node_fqn!r}
S={service_abs!r}
for _ in $(seq 1 200); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == active* ]] && break
  sleep 0.2
done
s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
if [[ "$s" != active* ]]; then
  echo "[camera_node_opt] SetBool: $N not active ($s), skip $S" >&2
  exit 1
fi
exec ros2 service call "$S" std_srvs/srv/SetBool '{{data: true}}'
"""
    return ["bash", "-lc", script]


def generate_launch_description():
    declare_gcs_ip = DeclareLaunchArgument(
    "gcs_ip",
    default_value="169.254.55.164", # IPv4 of CS NUC
    description="Control station IP address",
    )

    gcs_ip = LaunchConfiguration("gcs_ip")
    opt_jpeg_quality = LaunchConfiguration("opt_jpeg_quality")
    opt_width        = LaunchConfiguration("opt_width")
    opt_height       = LaunchConfiguration("opt_height")
    opt_fps          = LaunchConfiguration("opt_fps")

    declare_opt_jpeg_quality = DeclareLaunchArgument(
        "opt_jpeg_quality",
        default_value="15",
        description="MJPEG encoder quality 1-100 (lower = smaller files, more compression).",
    )
    declare_opt_width = DeclareLaunchArgument(
        "opt_width",
        default_value="640",
        description="Capture width (Oak1W: use 1280 with height 720 for 1080p ISP scale 2/3).",
    )
    declare_opt_height = DeclareLaunchArgument(
        "opt_height",
        default_value="360",
        description="Capture height (pair with opt_width).",
    )
    declare_opt_fps = DeclareLaunchArgument(
        "opt_fps",
        default_value="30",
        description="Frames per second for the color camera / encoder.",
    )

    # ── Oak1W cameras (nav 0-2) ───────────────────────────────────────────────

    cs_nav_0 = LifecycleNode(
        package="camera",
        executable="camera",
        name="cs_nav_0_oak1w_21W_T2544_0008",
        namespace="/CS",
        parameters=[
            {"camera_type": "oak1w_stereo"},
            {"topic_service": "/CS/req_camera_nav_0"},
            {"topic_pub": "feed_camera_nav_0"},
            {"depth": "/CS/depth_camera_nav_0"},
            {"bw_pub": "/CS/bw_camera_nav_0"},
            {"depth_req": "/CS/depth_req_camera_nav_0"},
            {"devrule": ""},
            {"info": "/CS/camera_info_0"},
            {"state": "/CS/state_camera_nav_0"},
            {"flip_camera": False},
            {"cam_id": "19443010714B177E00"},
            {"health_check_period_sec": 1.0},
            {"health_timeout_sec": 3.0},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    cs_nav_1 = LifecycleNode(
        package="camera",
        executable="camera",
        name="cs_nav_1_oak1w_21W_T2544_0069",
        namespace="/CS",
        parameters=[
            {"camera_type": "oak1w_stereo"},
            {"topic_service": "/CS/req_camera_nav_1"},
            {"topic_pub": "feed_camera_nav_1"},
            {"depth": "/CS/depth_camera_nav_1"},
            {"bw_pub": "/CS/bw_camera_nav_1"},
            {"depth_req": "/CS/depth_req_camera_nav_1"},
            {"devrule": ""},
            {"info": "/CS/camera_info_1"},
            {"state": "/CS/state_camera_nav_1"},
            {"flip_camera": False},
            {"cam_id": "19443010A19E157E00"},
            {"health_check_period_sec": 1.0},
            {"health_timeout_sec": 3.0},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    cs_nav_2 = LifecycleNode(
        package="camera",
        executable="camera",
        name="cs_nav_2_oak1w_21W_T2544_0035",
        namespace="/CS",
        parameters=[
            {"camera_type": "oak1w_stereo"},
            {"topic_service": "/CS/req_camera_nav_2"},
            {"topic_pub": "feed_camera_nav_2"},
            {"depth": "/CS/depth_camera_nav_2"},
            {"bw_pub": "/CS/bw_camera_nav_2"},
            {"depth_req": "/CS/depth_req_camera_nav_2"},
            {"devrule": ""},
            {"info": "/CS/camera_info_2"},
            {"state": "/CS/state_camera_nav_2"},
            {"flip_camera": False},
            {"cam_id": "19443010816C177E00"},
            {"health_check_period_sec": 1.0},
            {"health_timeout_sec": 3.0},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    # ── Fourth nav camera (Oak-D hardware, same MXID as NAV launch) ───────────
    # Use oak1w_stereo: this repo has no oakd_stereo_camera.py; oakd_stereo fails
    # at import with a misleading "depthai" error. NAV launch drives this MXID via
    # oak1w_stereo as well.

    cs_nav_3 = LifecycleNode(
        package="camera",
        executable="camera",
        name="cs_nav_3_oakd",
        namespace="/CS",
        parameters=[
            {"camera_type": "oak1w_stereo"},
            {"topic_service": "/CS/req_camera_nav_3"},
            {"topic_pub": "feed_camera_nav_3"},
            {"depth": "/CS/depth_camera_nav_3"},
            {"bw_pub": "/CS/bw_camera_nav_3"},
            {"depth_req": "/CS/depth_req_camera_nav_3"},
            {"devrule": ""},
            {"info": "/CS/camera_info_3"},
            {"state": "/CS/state_camera_nav_3"},
            {"flip_camera": False},
            {"cam_id": "19443010F1C5E01200"},          # OakD MXID from NAV launch
            {"health_check_period_sec": 1.0},
            {"health_timeout_sec": 3.0},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    # ── Lifecycle: configure (bash waits + retries; see module helpers) ─────

    n0 = "/CS/cs_nav_0_oak1w_21W_T2544_0008"
    n1 = "/CS/cs_nav_1_oak1w_21W_T2544_0069"
    n2 = "/CS/cs_nav_2_oak1w_21W_T2544_0035"
    n3 = "/CS/cs_nav_3_oakd"

    configure_0 = ExecuteProcess(cmd=_lifecycle_configure_cmd(n0), output="screen")
    configure_1 = ExecuteProcess(cmd=_lifecycle_configure_cmd(n1), output="screen")
    configure_2 = ExecuteProcess(cmd=_lifecycle_configure_cmd(n2), output="screen")
    configure_3 = ExecuteProcess(cmd=_lifecycle_configure_cmd(n3), output="screen")

    # ── Lifecycle: activate ───────────────────────────────────────────────────

    activate_0 = ExecuteProcess(cmd=_lifecycle_activate_cmd(n0), output="screen")
    activate_1 = ExecuteProcess(cmd=_lifecycle_activate_cmd(n1), output="screen")
    activate_2 = ExecuteProcess(cmd=_lifecycle_activate_cmd(n2), output="screen")
    activate_3 = ExecuteProcess(cmd=_lifecycle_activate_cmd(n3), output="screen")

    # ── SetBool: start streaming ──────────────────────────────────────────────

    call_0 = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n0, "/CS/req_camera_nav_0"), output="screen")
    call_1 = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n1, "/CS/req_camera_nav_1"), output="screen")
    call_2 = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n2, "/CS/req_camera_nav_2"), output="screen")
    call_3 = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n3, "/CS/req_camera_nav_3"), output="screen")

    # ── Timers: stagger camera nodes; configure wrappers poll until ready ─────
    delayed_0 = TimerAction(period=0.0,  actions=[cs_nav_0])
    delayed_1 = TimerAction(period=3.0, actions=[cs_nav_1])
    delayed_2 = TimerAction(period=6.0, actions=[cs_nav_2])
    delayed_3 = TimerAction(period=9.0, actions=[cs_nav_3])

    configure_0_delayed = TimerAction(period=0.0,  actions=[configure_0])
    configure_1_delayed = TimerAction(period=3.0, actions=[configure_1])
    configure_2_delayed = TimerAction(period=6.0, actions=[configure_2])
    configure_3_delayed = TimerAction(period=9.0, actions=[configure_3])

    # Activate only after configure command exits for each camera.
    activate_after_configure_0 = RegisterEventHandler(
        OnProcessExit(target_action=configure_0, on_exit=[activate_0])
    )
    activate_after_configure_1 = RegisterEventHandler(
        OnProcessExit(target_action=configure_1, on_exit=[activate_1])
    )
    activate_after_configure_2 = RegisterEventHandler(
        OnProcessExit(target_action=configure_2, on_exit=[activate_2])
    )
    activate_after_configure_3 = RegisterEventHandler(
        OnProcessExit(target_action=configure_3, on_exit=[activate_3])
    )

    # ── Event handlers: call SetBool only after activate exits ────────────────

    call_after_activate_0 = RegisterEventHandler(
        OnProcessExit(target_action=activate_0, on_exit=[call_0])
    )
    call_after_activate_1 = RegisterEventHandler(
        OnProcessExit(target_action=activate_1, on_exit=[call_1])
    )
    call_after_activate_2 = RegisterEventHandler(
        OnProcessExit(target_action=activate_2, on_exit=[call_2])
    )
    call_after_activate_3 = RegisterEventHandler(
        OnProcessExit(target_action=activate_3, on_exit=[call_3])
    )

    #launch gstream feeds to be visualized at the CS
    gst_bridge = RosNode(
    package="camera",
    executable="gst_camera_bridge",
    name="gst_camera_bridge",
    namespace="/CS",
    parameters=[
        {"host":      ParameterValue(gcs_ip,     value_type=str)},
        {"base_port": 5000},
        {"width":     ParameterValue(opt_width,  value_type=int)},
        {"height":    ParameterValue(opt_height, value_type=int)},
        {"fps":       ParameterValue(opt_fps,    value_type=int)},
    ],
    output="screen",
    )

    gst_bridge_delayed = TimerAction(period=15.0, actions=[gst_bridge])

    return LaunchDescription([
        declare_gcs_ip,
        declare_opt_jpeg_quality,
        declare_opt_width,
        declare_opt_height,
        declare_opt_fps,
        delayed_0,
        delayed_1,
        delayed_2,
        delayed_3,
        configure_0_delayed,
        configure_1_delayed,
        configure_2_delayed,
        configure_3_delayed,
        activate_after_configure_0,
        activate_after_configure_1,
        activate_after_configure_2,
        activate_after_configure_3,
        call_after_activate_0,
        call_after_activate_1,
        call_after_activate_2,
        call_after_activate_3,
        gst_bridge_delayed
    ])