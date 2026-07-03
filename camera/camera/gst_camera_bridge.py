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

# Bridges the HDS gripper camera's internal full-resolution CompressedImage
# feed (/HD/camera/rgb) to a GStreamer UDP/H264 stream for the control
# station, so the operator gets a higher-quality feed than the bandwidth-
# limited /ROVER/feed_camera_hd_0 topic without re-compressing that one.


class CameraStream:
    def __init__(self, topic, host, port, width, height, fps, bitrate, logger):
        self.topic = topic
        self._logger = logger
        self._lock = threading.Lock()
        self._pts = 0
        self._duration = Gst.SECOND // fps

        pipeline_str = (
            f"appsrc name=src "
            f"is-live=true format=time block=false "
            f"max-buffers=1 leaky-type=downstream "
            f"caps=\"image/jpeg,framerate={fps}/1\" "
            f"! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            f"! jpegdec "
            f"! videoconvert "
            f"! videoscale "
            f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
            f"! x264enc name=enc bitrate={bitrate} tune=zerolatency speed-preset=superfast "
            f"key-int-max={fps} vbv-buf-capacity=1000 "
            f"! h264parse "
            f"! rtph264pay pt=96 config-interval=1 mtu=1200 "
            f"! udpsink host={host} port={port} sync=false async=false buffer-size=65536"
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc = self._pipeline.get_by_name("src")
        self._encoder = self._pipeline.get_by_name("enc")

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_error)
        bus.connect("message::warning", self._on_warning)

        self._pipeline.set_state(Gst.State.PLAYING)
        logger.info(f"[GstBridge] {topic} -> {host}:{port}")

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


class GstHDCameraBridgeNode(Node):
    def __init__(self):
        super().__init__("gst_hd_camera_bridge")
        self.declare_parameter("topic", "/HD/camera/rgb")
        self.declare_parameter("host", "169.254.55.165")
        self.declare_parameter("port", 5013)
        self.declare_parameter("width", 854)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("bitrate", 1500)

        topic = self.get_parameter("topic").value
        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fps = self.get_parameter("fps").value
        bitrate = self.get_parameter("bitrate").value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        threading.Thread(target=GLib.MainLoop().run, daemon=True).start()

        self._stream = CameraStream(topic, host, port, width, height, fps, bitrate, self.get_logger())
        self.create_subscription(
            CompressedImage, topic,
            lambda msg: self._stream.push(bytes(msg.data)),
            qos,
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "bitrate" and p.type_ == Parameter.Type.INTEGER:
                self._stream.set_bitrate(p.value)
                self.get_logger().info(f"[GstBridge] bitrate updated to {p.value} kbps")
        return SetParametersResult(successful=True)

    def destroy_node(self):
        self._stream.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GstHDCameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
