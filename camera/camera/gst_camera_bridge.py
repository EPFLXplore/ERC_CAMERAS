import threading
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage

Gst.init(None)

# this node subscribes to ROS2 CompressedImage topics inside the rover, and creates independant 
# Gstreamer feeds

_NAV_CAMERAS = [
    {"topic": "/CS/feed_camera_nav_0", "port": 5000},
    {"topic": "/CS/feed_camera_nav_1", "port": 5002},
    {"topic": "/CS/feed_camera_nav_2", "port": 5004},
    {"topic": "/CS/feed_camera_nav_3", "port": 5006},
]

_CS_CAMERAS = [
    {"topic": "/ROVER/feed_camera_cs_top",         "port": 5008},
    {"topic": "/ROVER/feed_camera_cs_right_steer", "port": 5010},
    {"topic": "/ROVER/feed_camera_cs_left_steer",  "port": 5012},
]


CAM_FPS = 15
CAM_BITRATE_KBPS = 800

class CameraStream:
    def __init__(self, index, topic, host, port, width, height, fps, bitrate, logger):
        self.topic = topic
        self._logger = logger
        self._lock = threading.Lock()
        self._pts = 0
        self._duration = Gst.SECOND // fps

        pipeline_str = (
            f"appsrc name=src{index} "
            f"is-live=true format=time block=false "
            f"max-buffers=1 leaky-type=downstream "
            f"caps=\"image/jpeg,framerate={fps}/1\" "
            f"! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            f"! jpegdec "
            f"! videoconvert "
            f"! videoscale "
            f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
            f"! x264enc name=enc{index} bitrate={bitrate} tune=zerolatency speed-preset=superfast "
            f"key-int-max={fps} vbv-buf-capacity=1000 "
            f"! h264parse "
            f"! rtph264pay pt=96 config-interval=1 mtu=1200 "
            f"! udpsink host={host} port={port} sync=false async=false buffer-size=65536"
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc   = self._pipeline.get_by_name(f"src{index}")
        self._encoder  = self._pipeline.get_by_name(f"enc{index}")

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error",   self._on_error)
        bus.connect("message::warning", self._on_warning)

        self._pipeline.set_state(Gst.State.PLAYING)
        logger.info(f"[GstBridge] {topic} → {host}:{port}")

    def _on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        self._logger.error(f"[GstBridge] {self.topic} ERROR: {err.message} | {dbg}")

    def _on_warning(self, bus, msg):
        warn, dbg = msg.parse_warning()
        self._logger.warning(f"[GstBridge] {self.topic} WARNING: {warn.message} | {dbg}")

    def push(self, jpeg_bytes):
        buf = Gst.Buffer.new_wrapped(jpeg_bytes)
        with self._lock:
            buf.pts = self._pts
            buf.duration = self._duration
            self._pts += self._duration
        ret = self._appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            self._logger.warning(f"[GstBridge] {self.topic} push-buffer returned {ret}")

    def set_bitrate(self, kbps: int):
        self._encoder.set_property("bitrate", kbps)

    def stop(self):
        self._appsrc.emit("end-of-stream")
        self._pipeline.set_state(Gst.State.NULL)


class GstCameraBridgeNode(Node):
    def __init__(self):
        super().__init__("gst_camera_bridge")
        self.declare_parameter("mode", "nav")  # "nav" or "cs"
        self.declare_parameter("host", "169.254.55.164") # Control Station NUC
        self.declare_parameter("base_port", 5000)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 360)
        self.declare_parameter("fps", CAM_FPS)
        self.declare_parameter("bitrate", CAM_BITRATE_KBPS)

        mode      = self.get_parameter("mode").value
        host      = self.get_parameter("host").value
        base_port = self.get_parameter("base_port").value
        width     = self.get_parameter("width").value
        height    = self.get_parameter("height").value
        fps       = self.get_parameter("fps").value
        bitrate   = self.get_parameter("bitrate").value

        camera_list = _CS_CAMERAS if mode == "cs" else _NAV_CAMERAS

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        threading.Thread(target=GLib.MainLoop().run, daemon=True).start()

        self._streams = {}
        for i, cam in enumerate(camera_list):
            topic = cam["topic"]
            port  = base_port + i * 2
            stream = CameraStream(i, topic, host, port, width, height, fps, bitrate, self.get_logger())
            self._streams[topic] = stream
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._streams[t].push(bytes(msg.data)),
                qos,
            )

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "bitrate" and p.type_ == Parameter.Type.INTEGER:
                for s in self._streams.values():
                    s.set_bitrate(p.value)
                self.get_logger().info(f"[GstBridge] bitrate updated to {p.value} kbps")
        return SetParametersResult(successful=True)

    def destroy_node(self):
        for s in self._streams.values():
            s.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GstCameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()