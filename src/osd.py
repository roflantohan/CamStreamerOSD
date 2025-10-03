import cv2
import math
from shared_memory import SharedMemory
from fps_counter import FPSCounter


# # --- Гео-вспомогательное ---
def haversine_m(lat1, lon1, lat2, lon2):
    # вход: градусы
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlmb/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2, heading):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(phi2)
    x = math.cos(phi1) * math.cos(phi2) * math.cos(dlmb) - math.sin(phi1) * math.sin(
        phi2
    )
    brg = (math.degrees(math.atan2(y, x)) - heading + 360.0) % 360.0
    return brg

class ElementPrinter:
    def print_rectangle(self, frame, p1, p2, thick=1, color=(0, 255, 0)):
        cv2.rectangle(frame, p1, p2, color, thick)

    def print_border(self, frame, p1, p2, len=5, thick=1, color=(255, 0, 255)):
        # top left corner
        cv2.line(frame, p1, (p1[0] + len, p1[1]), color, thick)
        cv2.line(frame, p1, (p1[0], p1[1] + len), color, thick)
        # top right corner
        cv2.line(frame, (p2[0], p1[1]), (p2[0] - len, p1[1]), color, thick)
        cv2.line(frame, (p2[0], p1[1]), (p2[0], p1[1] + len), color, thick)
        # bottom left corner
        cv2.line(frame, (p1[0], p2[1]), (p1[0] + len, p2[1]), color, thick)
        cv2.line(frame, (p1[0], p2[1]), (p1[0], p2[1] - len), color, thick)
        # bottom right corner
        cv2.line(frame, p2, (p2[0] - len, p2[1]), color, thick)
        cv2.line(frame, p2, (p2[0], p2[1] - len), color, thick)

    def print_circle(self, frame, point, radius=3, thick=1, color=(128, 0, 128)):
        cv2.circle(frame, point, radius, color, thick)

    def print_line(self, frame, p1, p2, color=(255, 153, 204)):
        cv2.line(frame, p1, p2, color, 1)

    def print_text(self, frame, text, pos, color=(0, 255, 0)):
        f = cv2.FONT_HERSHEY_SIMPLEX
        f_scale = 0.35
        f_color = color
        f_thickness = 1
        cv2.putText(frame, text, pos, f, f_scale, f_color, f_thickness, cv2.LINE_AA)


class OSDController(ElementPrinter):
    def __init__(self, shm: SharedMemory):
        self.shm = shm
        self.fps = FPSCounter()
        self.w = shm.config["width"]
        self.h = shm.config["height"]
        self.g_state = None

        self.dist_to_home = None
        self.dir_to_home = None
        self.alt_rel_m = None
        self.sat_count = None
        self.airspeed = None
        self.groudspeed = None

    def add_center_pointer(self, frame):
        p = (self.w // 2, self.h // 2)
        self.print_circle(frame, p, 1, 2)

    def add_fps(self, frame):
        self.print_text(frame, f"FPS: {self.fps.update()}", (2, self.h - 5))

    def add_horizon(self, frame):
        horizon = self.shm.read("horizon")
        if horizon:
            (p1, p2) = horizon
            self.print_line(frame, p1, p2)

    def put_ardu_info(self, frame):
        home_lat = self.shm.read("home_lat")
        home_lon = self.shm.read("home_lon")
        pos_lat = self.shm.read("pos_lat")
        pos_lon = self.shm.read("pos_lon")
        hud_heading = self.shm.read("hud_heading")
        if all(v is not None for v in (home_lat, home_lon, pos_lat, pos_lon)):
            self.dist_to_home = haversine_m(
                pos_lat, pos_lon, home_lat, home_lon
            )  # meters
            self.dir_to_home = bearing_deg(
                pos_lat, pos_lon, home_lat, home_lon, hud_heading
            )  # degree

        pos_alt_rel = self.shm.read("pos_alt_rel")
        if pos_alt_rel is not None:
            self.alt_rel_m = pos_alt_rel / 1000.0  # meters

        hud_airspeed = self.shm.read("hud_airspeed")
        if hud_airspeed is not None:
            self.airspeed = hud_airspeed

        hud_groundspeed = self.shm.read("hud_groundspeed")
        if hud_groundspeed is not None:
            self.groudspeed = hud_groundspeed

        sat_count = self.shm.read("sat_count")
        if sat_count is not None:
            self.sat_count = sat_count

        def draw_home_arrow(frame, center, length, arrow_angle):
            ang = math.radians(arrow_angle)
            x2 = int(center[0] + length * math.sin(ang))
            y2 = int(center[1] - length * math.cos(ang))
            cv2.arrowedLine(frame, center, (x2, y2), (0, 0, 255), 2, tipLength=0.3)
            return frame
        
        self.print_text(frame, f"Sats: {sat_count}", (2, 70))
        self.print_text(
            frame,
            f"Alt: {self.alt_rel_m if self.alt_rel_m is None else round(self.alt_rel_m, 2)} m",
            (2, 80),
        )
        
        self.print_text(
            frame,
            f"AirSpeed: {self.airspeed if self.airspeed is None else round(self.airspeed, 2)} m/s",
            (2, 90),
        )
        self.print_text(
            frame,
            f"GndSpeed: {self.groudspeed if self.groudspeed is None else round(self.groudspeed, 2)} m/s",
            (2, 100),
        )
        self.print_text(
            frame,
            f"To Home: {self.dist_to_home if self.dist_to_home is None else round(self.dist_to_home, 2)} m",
            (2, 110),
        )

        flight_mode = self.shm.read("flight_mode")
        self.print_text(frame, f"Mode: {flight_mode}", (2, 120))

        armed = self.shm.read("armed")
        time_armed_us = self.shm.read("time_armed_us")
        time_boot_us = self.shm.read("time_boot_us")
        timer = (time_boot_us - time_armed_us) // 1000.0 if armed else 0
        self.print_text(frame, f"{'Armed' if armed else 'Disarmed'}", (2, 130))
        self.print_text(frame, f"FlyTime: {timer} sec" , (2, 140))

        if self.dir_to_home is not None:
            draw_home_arrow(frame, (50, 220), 20, self.dir_to_home)


    
    def put_turbine_info(self, frame):
        t_telem = self.shm.read("turbine")

        if t_telem is not None:
            status = t_telem["status"]
            error = t_telem["error"]
            temp = t_telem["temp_C"]
            rpm = t_telem["rpm"]
            time_armed = t_telem["startup_time_s"]

            self.print_text(frame, f"T_STAT: {status}", (2, 150))
            self.print_text(frame, f"T_ERR: {error}", (2, 160))
            self.print_text(frame, f"T_TEMP: {temp}", (2, 170))
            self.print_text(frame, f"T_RPM: {rpm}", (2, 180))
            self.print_text(frame, f"T_TIME: {time_armed}", (2, 190))
            
    def put_info(self, frame):
        self.add_horizon(frame)
        self.add_center_pointer(frame)
        self.add_fps(frame)
        try:
            self.put_ardu_info(frame)
            self.put_turbine_info(frame)
        except Exception as err:
            print(err)
