import math

class SkyLineFinder:
    def __init__(self, width, height, fov_h=33, y_offset=0):
        self.w = width
        self.h = height
        self.fov_h = fov_h
        
        self.y_offset = y_offset
        self.horizon_line = None

        self.min_h = 20
        self.sun_line = None

    def get_line(self):
        return self.horizon_line

    def find_horizon(self, roll, pitch):
        if not (roll and pitch):
            self.horizon_line = None
            return

        y_h = self.y_offset + (self.h / 2) + (self.h / math.radians(self.fov_h)) * pitch
        y_h = max(0, min(self.h, y_h))

        x1, x2 = 0, self.w

        y1 = int(y_h + math.tan(roll) * (self.w / 2))
        y2 = int(y_h - math.tan(roll) * (self.w / 2))

        self.horizon_line = ((x1, y1), (x2, y2))
