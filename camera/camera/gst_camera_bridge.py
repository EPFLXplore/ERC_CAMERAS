import threading
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

Gst.init(None)

# this node subscribes to ROS2 CompressedImage topics inside the rover, and creates independant 
# Gstreamer feeds

_CAMERAS = [
    {"topic": "/CS/feed_camera_nav_0", "port": 5000},
    {"topic": "/CS/feed_camera_nav_1", "port": 5002},
    {"topic": "/CS/feed_camera_nav_2", "port": 5004},
    {"topic": "/CS/feed_camera_nav_3", "port": 5006},
]


KBPS_PER_CAM = 800
CAM_FPS = 15

class CameraStream:
    def __init__(self, index, topic, host, port, width, height, fps, logger):
        self.topic = topic
        self._lock = threading.Lock()
        self._pts = 0
        self._duration = Gst.SECOND // fps

        pipeline_str = (
            f"appsrc name=src{index} "
            f"is-live=true format=time block=false "
            f"max-buffers=1 leaky-type=downstream "
            f"caps=\"image/jpeg,width={width},height={height},framerate={fps}/1\" "
            f"! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            f"! jpegdec "
            f"! videoconvert "
            f"! video/x-raw,format=I420,width={width},height={height},framerate={fps}/1 "
            f"! x264enc bitrate={KBPS_PER_CAM} tune=zerolatency speed-preset=ultrafast "
            f"key-int-max={fps} "
            f"! h264parse "
            f"! rtph264pay pt=96 config-interval=1 mtu=1200 "
            f"! udpsink host={host} port={port} sync=false async=false buffer-size=65536"
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
        self.declare_parameter("host", "169.254.55.164") # Control Station NUC
        self.declare_parameter("base_port", 5000)
        self.declare_parameter("width", 428)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", CAM_FPS)

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