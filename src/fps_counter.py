import time


class FPSCounter:
    def __init__(self):
        self.fps = 0
        self.frames_count = 0
        self.last_time = time.time()

    def update(self):
        self.frames_count += 1
        if self.frames_count >= 10:
            seconds = time.time() - self.last_time
            self.fps = self.frames_count // seconds
            self.frames_count = 0
            self.last_time = time.time()

        return self.fps
