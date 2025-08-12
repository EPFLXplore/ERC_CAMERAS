import cv2
import time
from cv_bridge import CvBridge
import subprocess
from sensor_msgs.msg import CompressedImage

class MonocularCamera():
    def __init__(self, node):

        self.node = node
        self.bridge = CvBridge()
        self.frame = None
        
        self.target_mbps = 5.0
        self.min_quality = 15
        self.max_quality = 95
        self.quality = 30                 # start point
        self.scale = 1.0                  # downscale factor if needed
        self.ema_alpha = 0.2              # smoothing for size/bw

        self.bytes_per_frame_budget = max(1, int((self.target_mbps * 1e6) / 8.0 / max(1, self.node.fps)))
        self.ema_bytes = self.bytes_per_frame_budget

    def publish_feeds(self, camera_id):
        
        result = subprocess.run(['realpath', camera_id], capture_output=True, text=True)
        path_id = result.stdout.strip()
        
        camera = cv2.VideoCapture(path_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        camera.set(cv2.CAP_PROP_FPS, self.node.fps)

        previous_time = 0
        
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        
        while not self.node.stopped:
            ret, self.frame = camera.read()
            
            if self.frame is not None:
                
                if self.scale < 0.999:
                    new_w = max(64, int(self.frame.shape[1] * self.scale))
                    new_h = max(64, int(self.frame.shape[0] * self.scale))
                    self.frame = cv2.resize(self.frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
                    
                ok, buf = cv2.imencode('.jpg', self.frame, encode_params)
                if not ok:
                    continue
                
                data = buf.tobytes()
            
                #compressed_image = self.bridge.cv2_to_compressed_imgmsg(data)
                compressed_image = CompressedImage()
                compressed_image.format = "jpeg"
                compressed_image.data = data
                
                current_time = time.time()
                bw = self.node.calculate_bandwidth(current_time, previous_time, len(compressed_image.data))
                previous_time = current_time

                self.node.cam_pubs.publish(compressed_image)
                self.node.cam_bw.publish(bw) 
                                
                # ---- Adaptive control ------------------------------------------------
                size_bytes = len(data)
                # Smooth the measurement
                self.ema_bytes = (1 - self.ema_alpha) * self.ema_bytes + self.ema_alpha * size_bytes

                # Error vs. per-frame budget
                err = self.ema_bytes - self.bytes_per_frame_budget
                rel = err / float(self.bytes_per_frame_budget)

                # Adjust JPEG quality (simple proportional controller)
                # Negative rel => under budget => increase quality; positive => decrease.
                k_q = 12.0  # aggressiveness; tune 6–20
                self.quality -= k_q * rel
                self.quality = max(self.min_quality, min(self.max_quality, int(round(self.quality))))

                # If quality bottomed out and still over budget, start downscaling a bit
                if self.quality <= self.min_quality and self.ema_bytes > 1.15 * self.bytes_per_frame_budget:
                    # reduce scale by small steps, but not below, say, 0.5
                    self.scale = max(0.5, self.scale * 0.97)
                # If comfortably under budget and quality near top, gently upscale back
                elif self.ema_bytes < 0.7 * self.bytes_per_frame_budget and self.quality >= self.max_quality - 2:
                    self.scale = min(1.0, self.scale * 1.01)
                # ----------------------------------------------------------------------
        
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        camera.set(cv2.CAP_PROP_FPS, self.node.fps)