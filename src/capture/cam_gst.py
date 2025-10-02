import cv2


class GSTCamera:
    def __init__(self, gst_pipeline):
        self.default_gst_str = (
            "gst-launch-1.0 autovideosrc "
            "! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert "
            "! video/x-raw,format=BGR ! appsink drop=1"
        )
        self.gst_str = gst_pipeline
        self.cap: cv2.VideoCapture = None

    def connect(self):
        if not self.gst_str:
            self.gst_str = self.default_gst_str
        self.cap = cv2.VideoCapture(self.gst_str, cv2.CAP_GSTREAMER)


    def is_cap(self):
        return self.cap.isOpened()

    def release(self):
        if self.cap:
            self.cap.release()

    def read(self):
        return self.cap.read()
