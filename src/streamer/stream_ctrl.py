from streamer.streamer_udp import UDPStreamer


class StreamController:
    def __init__(self):
        self.streamer = None

    def load_param(self, w, h, host, port):
        self.streamer = UDPStreamer(w, h, host, port)

    def create(self):
        self.streamer.create()

    def write(self, frame):
        self.streamer.write(frame)

    def release(self):
        self.streamer.release()
