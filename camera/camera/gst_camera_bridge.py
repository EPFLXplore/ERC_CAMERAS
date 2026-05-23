import threading
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

Gst.init(None)

_CAMERAS = [
    {"topic": "/CS/feed_camera_nav_0", "port": 5000},
    {"topic": "/CS/feed_camera_nav_1", "port": 5002},
    {"topic": "/CS/feed_camera_nav_2", "port": 5004},
    {"topic": "/CS/feed_camera_nav_3", "port": 5006},
]


class CameraStream:
    def __init__(self, index, topic, host, port, width, height, fps, logger):
        self.topic = topic
        self._lock = threading.Lock()
        self._pts = 0
        self._duration = Gst.SECOND // fps

        pipeline_str = (
            f"appsrc name=src{index} is-live=true block=false format=time "
            f"caps=\"image/jpeg,width={width},height={height},framerate={fps}/1\" "
            f"! rtpjpegpay "
            f"! udpsink host={host} port={port} sync=false buffer-size=524288"
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc = self._pipeline.get_by_name(f"src{index}")
        self._pipeline.set_state(Gst.State.PLAYING)
        logger.info(f"[GstBridge] {topic} → {host}:{port}")

    def push(self, jpeg_bytes):
        buf = Gst.Buffer.new_wrapped(jpeg_bytes)
        with self._lock:
            buf.pts = self._pts
            buf.duration = self._duration
            self._pts += self._duration
        self._appsrc.emit("push-buffer", buf)

    def stop(self):
        self._appsrc.emit("end-of-stream")
        self._pipeline.set_state(Gst.State.NULL)


class GstCameraBridgeNode(Node):
    def __init__(self):
        super().__init__("gst_camera_bridge")
        self.declare_parameter("host", "169.254.55.166")
        self.declare_parameter("base_port", 5000)
        self.declare_parameter("width", 428)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", 15)

        host      = self.get_parameter("host").value
        base_port = self.get_parameter("base_port").value
        width     = self.get_parameter("width").value
        height    = self.get_parameter("height").value
        fps       = self.get_parameter("fps").value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        threading.Thread(target=GLib.MainLoop().run, daemon=True).start()

        self._streams = {}
        for i, cam in enumerate(_CAMERAS):
            topic = cam["topic"]
            port  = base_port + i * 2
            stream = CameraStream(i, topic, host, port, width, height, fps, self.get_logger())
            self._streams[topic] = stream
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._streams[t].push(bytes(msg.data)),
                qos,
            )

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