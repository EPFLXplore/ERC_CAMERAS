import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_srvs.srv import SetBool
from .camera_factory import CameraFactory
from sensor_msgs.msg import CompressedImage
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, Bool
import time

class CameraNode(LifecycleNode):
    """
    Create a CameraNode
    """

    def __init__(self):
        super().__init__("camera_node")

        self.callback_group = ReentrantCallbackGroup()
        self.default = ""
        self.lifecycle_active = False
        self.stopped = True
        self.thread = None
        self.camera = None
        self.cam_pubs = None
        self.cam_bw = None
        self.state = None
        self.service_activation = None
        self.health_timer = None
        self.health_check_period_sec = 1.0
        self.health_timeout_sec = 3.0
        self._health_reconnect_in_progress = False
        self._camera_operation_lock = threading.RLock()

        # parameters
        self.declare_parameter("camera_type", self.default)
        self.declare_parameter("topic_service", self.default)
        self.declare_parameter("topic_pub", self.default)
        self.declare_parameter("bw_pub", self.default)
        self.declare_parameter("devrule", self.default)
        self.declare_parameter("state", self.default)
        self.declare_parameter("fps", 10)
        self.declare_parameter("x", 640)
        self.declare_parameter("y", 480)
        # MJPEG encoder quality for DepthAI hardware JPEG (1–100); lower = smaller bandwidth.
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("cam_id", self.default)
        self.declare_parameter("health_check_period_sec", 1.0)
        self.declare_parameter("health_timeout_sec", 3.0)

    def on_configure(self, state):
        del state

        self.camera_type = self.get_parameter("camera_type").get_parameter_value().string_value
        self.service_topic = self.get_parameter("topic_service").get_parameter_value().string_value
        self.publisher_topic = self.get_parameter("topic_pub").get_parameter_value().string_value
        self.publisher_topic_bw = self.get_parameter("bw_pub").get_parameter_value().string_value
        self.devrule = self.get_parameter("devrule").get_parameter_value().string_value
        self.state_topic = self.get_parameter("state").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value
        self.x = self.get_parameter("x").get_parameter_value().integer_value
        self.y = self.get_parameter("y").get_parameter_value().integer_value
        self.cam_id = self.get_parameter("cam_id").get_parameter_value().string_value
        self.health_check_period_sec = self.get_parameter("health_check_period_sec").get_parameter_value().double_value
        self.health_timeout_sec = self.get_parameter("health_timeout_sec").get_parameter_value().double_value

        # To be used for any camera
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT: message will attempt to send message but if it fails it will not try again
            durability=QoSDurabilityPolicy.VOLATILE, # VOLATILE: if no subscribers are listening, the message sent is not saved
            history=QoSHistoryPolicy.KEEP_LAST, # KEEP_LAST: only the last n = depth messages are stored in the queue
            depth=1,
        )

        # Initialize publishers before creating the camera instance
        self.cam_pubs = self.create_publisher(CompressedImage, self.publisher_topic, qos_profile=self.qos_profile, callback_group=self.callback_group)
        self.cam_bw = self.create_publisher(Float32, self.publisher_topic_bw, 1)
        self.state = self.create_publisher(Bool, self.state_topic, 1)

        # object camera
        self.camera =  CameraFactory.create_camera(self)

        # Service to activate the camera. For now we hardcode the parameters so we use just a SetBool
        self.service_activation = self.create_service(SetBool, self.service_topic, self.start_cameras_callback, callback_group=self.callback_group)

        self.health_timer = self.create_timer(
            self.health_check_period_sec,
            self.health_check_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Cameras ready")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        del state
        self.lifecycle_active = True
        self.publish_state(False)
        self.get_logger().info("Camera lifecycle node active")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        del state
        self.lifecycle_active = False
        self.stop_camera()
        self.get_logger().info("Camera lifecycle node inactive")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        del state
        self.lifecycle_active = False
        self.stop_camera()
        self.destroy_configured_entities()
        self.camera = None
        self.thread = None
        self.get_logger().info("Camera lifecycle node cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        del state
        self.lifecycle_active = False
        self.stop_camera()
        self.destroy_configured_entities()
        return TransitionCallbackReturn.SUCCESS

    def publish_state(self, is_running):
        if self.state is None:
            return
        msg = Bool()
        msg.data = bool(is_running)
        self.state.publish(msg)

    def destroy_configured_entities(self):
        if self.health_timer is not None:
            self.health_timer.cancel()
            self.destroy_timer(self.health_timer)
            self.health_timer = None

        if self.service_activation is not None:
            self.destroy_service(self.service_activation)
            self.service_activation = None

        if self.camera is not None and hasattr(self.camera, "destroy_ros_entities"):
            self.camera.destroy_ros_entities()

        if self.cam_pubs is not None:
            self.destroy_publisher(self.cam_pubs)
            self.cam_pubs = None
        if self.cam_bw is not None:
            self.destroy_publisher(self.cam_bw)
            self.cam_bw = None
        if self.state is not None:
            self.destroy_publisher(self.state)
            self.state = None

    def make_publish_thread(self):
        if self.camera_type == "oakd_stereo":
            return threading.Thread(target=self.camera.publish_feeds, daemon=True)
        if self.camera_type == "realsense_stereo":
            return threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,), daemon=True)
        if self.camera_type == "oak1w_stereo":
            return threading.Thread(target=self.camera.publish_feeds, daemon=True)
        return threading.Thread(target=self.camera.publish_feeds, args=(self.devrule,), daemon=True)

    def start_camera(self):
        with self._camera_operation_lock:
            if self.camera is None:
                raise RuntimeError("Camera is not configured")

            self.stopped = False
            if self.thread is not None and self.thread.is_alive():
                return False

            if hasattr(self.camera, "open_device"):
                self.camera.open_device()

            self.thread = self.make_publish_thread()
            self.thread.start()
            self.publish_state(True)
            return True

    def stop_camera(self):
        with self._camera_operation_lock:
            self.stopped = True
            if self.thread is not None and self.thread.is_alive():
                self.thread.join(timeout=2.0)
            if self.camera is not None and hasattr(self.camera, "close_device"):
                self.camera.close_device()
            self.publish_state(False)

    def restart_publish_thread_if_needed(self):
        if self.stopped or self.camera is None:
            return
        if self.thread is not None and self.thread.is_alive():
            return
        self.get_logger().warn("Camera publish thread died, restarting it")
        self.thread = self.make_publish_thread()
        self.thread.start()

    def health_check_callback(self):
        if not self.lifecycle_active or self.stopped or self.camera is None:
            return
        if self._health_reconnect_in_progress:
            return
        if not self._camera_operation_lock.acquire(blocking=False):
            return

        try:
            thread_alive = self.thread is not None and self.thread.is_alive()
            if not thread_alive:
                self.restart_publish_thread_if_needed()
                # Give the restarted publisher one health window to reopen hardware
                # and deliver frames before declaring it unhealthy again.
                return

            is_healthy = True
            if hasattr(self.camera, "is_healthy"):
                is_healthy = self.camera.is_healthy(self.health_timeout_sec)
            elif hasattr(self.camera, "is_open"):
                is_healthy = self.camera.is_open()

            if is_healthy:
                self.publish_state(True)
                return

            self._health_reconnect_in_progress = True
            self.publish_state(False)
            self.get_logger().warn("Camera health check failed, reconnecting device")
            # The publish thread already has the reconnect loop. Closing the
            # device here forces its next iteration to reopen the same MXID.
            if hasattr(self.camera, "close_device"):
                self.camera.close_device()
        except Exception as e:
            self.get_logger().error(f"Camera health reconnect failed: {e}")
        finally:
            self._health_reconnect_in_progress = False
            self._camera_operation_lock.release()
    
    def start_cameras_callback(self, request, response):
        if not self.lifecycle_active:
            response.success = False
            response.message = "Camera lifecycle node is not active"
            return response

        if request.data:
            try:
                started = self.start_camera()
            except Exception as e:
                self.stopped = True
                self.publish_state(False)
                response.success = False
                response.message = f"Failed to open camera device: {e}"
                return response

            if not started:
                response.success = False
                response.message = "Cameras are already running"
                return response

            response.success = True
            response.message = "Cameras started"
        else:
            had_thread = self.thread is not None and self.thread.is_alive()
            self.stop_camera()
            response.success = had_thread
            response.message = "Cameras stopped" if had_thread else "No camera thread to stop"

        return response

    def calculate_bandwidth(self, current_time, previous_time, compressed_image_len):
        elapsed_time = current_time - previous_time
        bw = Float32()
        bw.data = float(compressed_image_len * 8) / (elapsed_time * 1_000_000) # Bandwidth in Mbps
        
        return bw

def main(args=None):
    rclpy.init(args=args)
    cameras_publisher = CameraNode()
    executor = MultiThreadedExecutor()
    executor.add_node(cameras_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cameras_publisher.stop_camera()
        executor.remove_node(cameras_publisher)
        cameras_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()