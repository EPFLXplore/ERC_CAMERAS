# Monocular V4L2 cameras on the rover: top, right_steer, left_steer.
# Publishes CompressedImage on /ROVER/feed_camera_cs_{top,right_steer,left_steer}.
# Cameras are lifecycle nodes — configure → activate → SetBool to start streaming.
# A GStreamer bridge re-encodes the feeds as H.264 UDP to the CS NUC (ports 5008-5012).
#
# Tune from the command line, e.g.:
#   ros2 launch camera camera_node_cs.launch.py opt_jpeg_quality:=10 opt_width:=1280 opt_height:=720 opt_fps:=15

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node as RosNode


def _lifecycle_configure_cmd(node_fqn: str) -> list:
    script = f"""
set -u
N={node_fqn!r}
ready=""
for _ in $(seq 1 60); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  if [[ "$s" == inactive* || "$s" == active* ]]; then
    echo "[camera_node_cs] configure: $N already $s"
    exit 0
  fi
  if [[ "$s" == unconfigured* ]]; then
    ready=1
    break
  fi
  sleep 0.25
done
[[ -n "$ready" ]] || {{ echo "[camera_node_cs] configure: timeout waiting for $N" >&2; exit 1; }}
for _ in $(seq 1 30); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && exit 0
  [[ "$s" == unconfigured* ]] || {{ sleep 0.4; continue; }}
  ros2 lifecycle set "$N" configure >/dev/null 2>&1 || true
  sleep 0.5
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == inactive* ]] && exit 0
done
echo "[camera_node_cs] configure: gave up on $N" >&2
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
echo "[camera_node_cs] activate: gave up on $N" >&2
exit 1
"""
    return ["bash", "-lc", script]


def _lifecycle_call_setbool_cmd(node_fqn: str, service_abs: str) -> list:
    script = f"""
set -u
N={node_fqn!r}
S={service_abs!r}
for _ in $(seq 1 60); do
  s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
  [[ "$s" == active* ]] && break
  sleep 0.2
done
s=$(ros2 lifecycle get "$N" 2>/dev/null | head -n1 || true)
if [[ "$s" != active* ]]; then
  echo "[camera_node_cs] SetBool: $N not active ($s), skip $S" >&2
  exit 1
fi
exec ros2 service call "$S" std_srvs/srv/SetBool '{{data: true}}'
"""
    return ["bash", "-lc", script]


def generate_launch_description():
    declare_gcs_ip = DeclareLaunchArgument(
        "gcs_ip",
        default_value="169.254.55.166",
        description="Control station IP address",
    )
    declare_opt_jpeg_quality = DeclareLaunchArgument(
        "opt_jpeg_quality",
        default_value="5",
        description="MJPEG encoder quality 1-100.",
    )
    declare_opt_width = DeclareLaunchArgument(
        "opt_width",
        default_value="1920",
        description="Capture width.",
    )
    declare_opt_height = DeclareLaunchArgument(
        "opt_height",
        default_value="1080",
        description="Capture height.",
    )
    declare_opt_fps = DeclareLaunchArgument(
        "opt_fps",
        default_value="24",
        description="Frames per second.",
    )
    declare_gst_width = DeclareLaunchArgument(
        "gst_width",
        default_value="640",
        description="GStreamer encoding width (lower = less artifacts at same bitrate).",
    )
    declare_gst_height = DeclareLaunchArgument(
        "gst_height",
        default_value="360",
        description="GStreamer encoding height.",
    )
    declare_gst_bitrate = DeclareLaunchArgument(
        "gst_bitrate",
        default_value="1200",
        description="x264 target bitrate in kbps (hard-capped via VBV).",
    )

    gcs_ip           = LaunchConfiguration("gcs_ip")
    opt_jpeg_quality = LaunchConfiguration("opt_jpeg_quality")
    opt_width        = LaunchConfiguration("opt_width")
    opt_height       = LaunchConfiguration("opt_height")
    opt_fps          = LaunchConfiguration("opt_fps")
    gst_width        = LaunchConfiguration("gst_width")
    gst_height       = LaunchConfiguration("gst_height")
    gst_bitrate      = LaunchConfiguration("gst_bitrate")

    # ── Lifecycle camera nodes ────────────────────────────────────────────────

    camera_cs_top = LifecycleNode(
        package="camera",
        executable="camera",
        name="camera_cs_top",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_top"},
            {"topic_pub": "feed_camera_cs_top"},
            {"bw_pub": "/ROVER/bw_camera_cs_top"},
            {"devrule": "/dev/top_cam"},
            {"state": "/ROVER/state_camera_cs_top"},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    camera_cs_right_steer = LifecycleNode(
        package="camera",
        executable="camera",
        name="camera_cs_right_steer",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_right_steer"},
            {"topic_pub": "feed_camera_cs_right_steer"},
            {"bw_pub": "/ROVER/bw_camera_cs_right_steer"},
            {"devrule": "/dev/right_steer_cam"},
            {"state": "/ROVER/state_camera_cs_right_steer"},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    camera_cs_left_steer = LifecycleNode(
        package="camera",
        executable="camera",
        name="camera_cs_left_steer",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_left_steer"},
            {"topic_pub": "feed_camera_cs_left_steer"},
            {"bw_pub": "/ROVER/bw_camera_cs_left_steer"},
            {"devrule": "/dev/left_steer_cam"},
            {"state": "/ROVER/state_camera_cs_left_steer"},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )

    camera_cs_drill_inside = LifecycleNode(
        package="camera",
        executable="camera",
        name="camera_cs_drill_inside",
        namespace="/ROVER",
        parameters=[
            {"camera_type": "monocular"},
            {"topic_service": "/ROVER/req_camera_cs_drill_inside"},
            {"topic_pub": "feed_camera_cs_drill_inside"},
            {"bw_pub": "/ROVER/bw_camera_cs_drill_inside"},
            {"devrule": "/dev/drill_cam_inside"},
            {"state": "/ROVER/state_camera_cs_drill_inside"},
            {"jpeg_quality": ParameterValue(opt_jpeg_quality, value_type=int)},
            {"x": ParameterValue(opt_width,  value_type=int)},
            {"y": ParameterValue(opt_height, value_type=int)},
            {"fps": ParameterValue(opt_fps,  value_type=int)},
        ],
        output="screen",
    )


    # ── Node FQNs ─────────────────────────────────────────────────────────────

    n_top   = "/ROVER/camera_cs_top"
    n_right = "/ROVER/camera_cs_right_steer"
    n_left  = "/ROVER/camera_cs_left_steer"
    n_drill_inside = "/ROVER/camera_cs_drill_inside"
    # ── Lifecycle: configure ──────────────────────────────────────────────────

    configure_top   = ExecuteProcess(cmd=_lifecycle_configure_cmd(n_top),   output="screen")
    configure_right = ExecuteProcess(cmd=_lifecycle_configure_cmd(n_right), output="screen")
    configure_left  = ExecuteProcess(cmd=_lifecycle_configure_cmd(n_left),  output="screen")
    configure_drill_inside = ExecuteProcess(cmd=_lifecycle_configure_cmd(n_drill_inside),  output="screen")

    # ── Lifecycle: activate ───────────────────────────────────────────────────

    activate_top   = ExecuteProcess(cmd=_lifecycle_activate_cmd(n_top),   output="screen")
    activate_right = ExecuteProcess(cmd=_lifecycle_activate_cmd(n_right), output="screen")
    activate_left  = ExecuteProcess(cmd=_lifecycle_activate_cmd(n_left),  output="screen")
    activate_drill_inside = ExecuteProcess(cmd=_lifecycle_activate_cmd(n_drill_inside),  output="screen")

    # ── SetBool: start streaming ──────────────────────────────────────────────

    call_top   = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n_top,   "/ROVER/req_camera_cs_top"),         output="screen")
    call_right = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n_right, "/ROVER/req_camera_cs_right_steer"), output="screen")
    call_left  = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n_left,  "/ROVER/req_camera_cs_left_steer"),  output="screen")
    call_drill_inside = ExecuteProcess(cmd=_lifecycle_call_setbool_cmd(n_drill_inside,  "/ROVER/req_camera_cs_drill_inside"),  output="screen")

    # ── Stagger: V4L2 cameras are fast — 0 / 0.5 / 1.0 s ────────────────────

    delayed_top   = TimerAction(period=0.0, actions=[camera_cs_top])
    delayed_right = TimerAction(period=0.5, actions=[camera_cs_right_steer])
    delayed_left  = TimerAction(period=1.0, actions=[camera_cs_left_steer])
    delayed_drill_inside  = TimerAction(period=1.5, actions=[camera_cs_drill_inside])

    configure_top_delayed   = TimerAction(period=0.0, actions=[configure_top])
    configure_right_delayed = TimerAction(period=0.5, actions=[configure_right])
    configure_left_delayed  = TimerAction(period=1.0, actions=[configure_left])
    configure_drill_inside_delayed  = TimerAction(period=1.5, actions=[configure_drill_inside])

    # Activate only after configure exits for each camera.
    activate_after_configure_top = RegisterEventHandler(
        OnProcessExit(target_action=configure_top, on_exit=[activate_top])
    )
    activate_after_configure_right = RegisterEventHandler(
        OnProcessExit(target_action=configure_right, on_exit=[activate_right])
    )
    activate_after_configure_left = RegisterEventHandler(
        OnProcessExit(target_action=configure_left, on_exit=[activate_left])
    )
    activate_after_configure_drill_inside = RegisterEventHandler(
        OnProcessExit(target_action=configure_drill_inside, on_exit=[activate_drill_inside])
    )

    # SetBool only after activate exits.
    call_after_activate_top = RegisterEventHandler(
        OnProcessExit(target_action=activate_top, on_exit=[call_top])
    )
    call_after_activate_right = RegisterEventHandler(
        OnProcessExit(target_action=activate_right, on_exit=[call_right])
    )
    call_after_activate_left = RegisterEventHandler(
        OnProcessExit(target_action=activate_left, on_exit=[call_left])
    )
    call_after_activate_drill_inside = RegisterEventHandler(
        OnProcessExit(target_action=activate_drill_inside, on_exit=[call_drill_inside])
    )

    # ── GStreamer bridge: CS camera feeds → H.264 UDP to CS NUC ──────────────

    gst_bridge = RosNode(
        package="camera",
        executable="gst_camera_bridge",
        name="gst_cs_camera_bridge",
        namespace="/ROVER",
        parameters=[
            {"mode":    "cs"},
            {"host":    ParameterValue(gcs_ip,      value_type=str)},
            {"width":   ParameterValue(gst_width,   value_type=int)},
            {"height":  ParameterValue(gst_height,  value_type=int)},
            {"fps":     ParameterValue(opt_fps,     value_type=int)},
            {"bitrate": ParameterValue(gst_bitrate, value_type=int)},
        ],
        output="screen",
    )

    gst_bridge_delayed = TimerAction(period=5.0, actions=[gst_bridge])

    return LaunchDescription([
        declare_gcs_ip,
        declare_opt_jpeg_quality,
        declare_opt_width,
        declare_opt_height,
        declare_opt_fps,
        declare_gst_width,
        declare_gst_height,
        declare_gst_bitrate,
        delayed_top,
        delayed_right,
        delayed_left,
        delayed_drill_inside,
        configure_top_delayed,
        configure_right_delayed,
        configure_left_delayed,
        configure_drill_inside_delayed,
        activate_after_configure_top,
        activate_after_configure_right,
        activate_after_configure_left,
        activate_after_configure_drill_inside,
        call_after_activate_top,
        call_after_activate_right,
        call_after_activate_left,
        call_after_activate_drill_inside,
        gst_bridge_delayed,
    ])
