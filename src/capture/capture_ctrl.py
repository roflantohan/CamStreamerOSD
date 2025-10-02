from capture.cam_gst import GSTCamera

class CaptureController:
    def __init__(self):
        self.camera = None

    def load_param(self, gst_str):
        self.camera = GSTCamera(gst_str)

    def is_cap(self):
        return self.camera.is_cap()

    def connect(self):
        self.camera.connect()

    def get_param(self):
        _, frame = self.camera.read()
        h, w = frame.shape[:2]
        return w, h

    def release(self):
        self.camera.release()

    def read(self):
        return self.camera.read()
