import cv2
import time
from cv_bridge import CvBridge

class MonocularCamera():
    def __init__(self, node):

        self.node = node
        self.bridge = CvBridge()

    def publish_feeds(self, camera_id):
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        camera.set(cv2.CAP_PROP_FPS, self.node.fps)

        previous_time = 0
        while not self.node.stopped:
            ret, frame = camera.read()
            
            if frame is not None:
            
                compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame)
                
                current_time = time.time()
                bw = self.node.calculate_bandwidth(current_time, previous_time, compressed_image)
                previous_time = current_time 

                self.node.cam_pubs.publish(compressed_image)
                self.node.cam_bw.publish(bw) 
        
        camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        camera.set(cv2.CAP_PROP_FPS, self.node.fps)