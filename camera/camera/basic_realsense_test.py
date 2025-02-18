import pyrealsense2 as rs
import numpy as np
import cv2

SERIAL_NUMBER = "102122061110"

pipeline = rs.pipeline()
config = rs.config()
config.enable_device(SERIAL_NUMBER)

# 1280x720, 30 FPS
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue 

        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RealSense Color Feed", color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
