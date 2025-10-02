import cv2

class UDPStreamer:
    def __init__(self, w, h, host, port):
        self.width = w
        self.height = h
        self.host = host
        self.port = port
        self.out: cv2.VideoWriter = None

    def create(self):
        fps = 30
        gst_pipeline = (
            f"appsrc format=time is-live=true block=true do-timestamp=true ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=2048 threads=4 ! "
            f"rtph264pay config-interval=1 pt=96 mtu=1400 aggregate-mode=zero-latency ! udpsink host={self.host} port={self.port}"
        )
        self.out = cv2.VideoWriter(
            gst_pipeline, cv2.CAP_GSTREAMER, 0, fps, (self.width, self.height), True
        )

    def write(self, frame):
        # frame_i420 = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
        self.out.write(frame)

    def release(self):
        self.out.release()
