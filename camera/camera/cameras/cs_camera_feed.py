import depthai as dai
import gi, threading, time
gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

DEST_IP   = "192.168.1.100"   # base station IP
BASE_PORT = 5000              # each camera gets its own port (+1, +2, ...)
WIDTH, HEIGHT, FPS = 1920, 1080, 30

def build_depthai_pipeline():
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(FPS)
    cam.setInterleaved(False)

    encoder = pipeline.create(dai.node.VideoEncoder)
    # Encode ON the MyriadX — zero CPU cost
    encoder.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
    encoder.setBitrateKbps(4000)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("h264")

    cam.video.link(encoder.input)
    encoder.bitstream.link(xout.input)
    return pipeline

def build_gst_pipeline(port):
    # appsrc → mark as H264 → parse → packetize → UDP
    return (
        f"appsrc name=src is-live=true block=true format=time "
        f"caps=video/x-h264,stream-format=byte-stream,alignment=au ! "
        f"h264parse ! rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host={DEST_IP} port={port} sync=false"
    )

def stream_device(device_info, port):
    gst = Gst.parse_launch(build_gst_pipeline(port))
    appsrc = gst.get_by_name("src")
    gst.set_state(Gst.State.PLAYING)

    with dai.Device(build_depthai_pipeline(), device_info) as device:
        q = device.getOutputQueue("h264", maxSize=30, blocking=False)
        pts = 0
        while True:
            packet = q.get()
            data = packet.getData()

            buf = Gst.Buffer.new_wrapped(bytes(data))
            buf.pts = pts
            buf.duration = Gst.SECOND // FPS
            pts += buf.duration

            appsrc.emit("push-buffer", buf)

# Launch one thread per camera
devices = dai.Device.getAllAvailableDevices()
print(f"Found {len(devices)} OAK cameras")

threads = []
for i, dev in enumerate(devices):
    t = threading.Thread(target=stream_device, args=(dev, BASE_PORT + i), daemon=True)
    t.start()
    threads.append(t)

for t in threads:
    t.join()