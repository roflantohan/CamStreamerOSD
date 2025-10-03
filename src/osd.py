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

def bearing_deg(lat1, lon1, lat2, lon2):
    # азимут из точки1 в точку2 (0..360)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(phi2)
    x = math.cos(phi1)*math.cos(phi2)*math.cos(dlmb) - math.sin(phi1)*math.sin(phi2)
    brg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
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

    def add_current_state(self, frame):
        if not self.g_state:
            return

        self.print_text(frame, f"{self.g_state}", (2, self.h - 20))

    def add_range(self, frame):
        range = self.shm.read("range")
        txt = "Range: "
        if range:
            range = round(range, 2)
            txt = f"Range: {range} m"
        self.print_text(frame, txt, (2, self.h - 35))

        is_range_alarm = self.shm.read("is_range_alarm")

        if is_range_alarm:
            txt = f"Alarm: {'True' if is_range_alarm else 'False'}"
            self.print_text(frame, txt, (2, self.h - 50))

    def add_detect_strength(self, frame):
        detect_strength = self.shm.read("detect_strength")
        if detect_strength is None:
            return

        detect_strength = "Strong" if detect_strength else "Weak"
        self.print_text(frame, f"{detect_strength} detection", (2, 20))

    def add_detect_spot(self, frame):
        detect_spot = self.shm.read("detect_spot")
        if not detect_spot:
            return

        # whole target
        (spot, point, value, square) = detect_spot
        p1 = (spot[0], spot[1])
        p2 = (spot[0] + spot[2], spot[1] + spot[3])
        self.print_rectangle(frame, p1, p2)

        # center of target
        cx = spot[0] + spot[2] // 2
        cy = spot[1] + spot[3] // 2
        cspot = (cx - 15, cy - 15, 30, 30)
        p1 = (cspot[0], cspot[1])
        p2 = (cspot[0] + cspot[2], cspot[1] + cspot[3])
        self.print_border(frame, p1, p2, 5, 1, (139, 0, 0))

        # local min of target
        roi = (point[0] - 15, point[1] - 15, 30, 30)
        p1 = (roi[0], roi[1])
        p2 = (roi[0] + roi[2], roi[1] + roi[3])
        self.print_border(frame, p1, p2, 5, 1, (0, 255, 0))

        # square of rect
        self.print_text(frame, f"Square: {round(square, 2)}", (2, 30))
        # value of local min
        self.print_text(frame, f"Value: {int(value)}", (2, 40))

    def add_detect_spots(self, frame):
        detect_spots = self.shm.read("detect_spots")
        if not detect_spots:
            return

        for spot in detect_spots:
            rect, _, _, _ = spot
            p1 = (rect[0], rect[1])
            p2 = (rect[0] + rect[2], rect[1] + rect[3])
            self.print_border(frame, p1, p2, 5, 1, (0, 255, 0))

    def add_track_roi(self, frame):
        self.track_roi = self.shm.read("track_roi")
        if self.track_roi:
            (x, y, w, h) = self.track_roi
            self.print_border(frame, (x, y), (x + w, y + h), 5, 1, (0, 0, 255))

    def add_result_roi(self, frame):
        result_roi = self.shm.read("result_roi")
        if result_roi:
            (x, y, w, h) = result_roi
            self.print_border(frame, (x, y), (x + w, y + h), 5, 1, (0, 255, 255))

    def add_track_error(self, frame):
        track_error = self.shm.read("track_error")
        if track_error:
            (x, y) = track_error
            x = round(x, 2)
            y = round(y, 2)
            txt = f"Error: ({x}; {y})"
            self.print_text(frame, txt, (70, self.h - 5))

        abs_roll = self.shm.read("abs_roll")
        abs_half = self.shm.read("abs_half")
        abs_pitch = self.shm.read("abs_pitch")

        if abs_roll and abs_pitch:
            txt = f"Devtn: ({round(abs_roll, 2)}; {round(abs_pitch, 2)})"
            self.print_text(frame, txt, (70, self.h - 20))

        new_roll = self.shm.read("new_roll")
        new_pitch = self.shm.read("new_pitch")

        if new_roll and new_pitch:
            txt = f"NPath: ({round(new_roll, 2)}; {round(new_pitch, 2)})"
            self.print_text(frame, txt, (70, self.h - 35))

    def add_reached_status(self, frame):
        is_reached = self.shm.read("is_reached")
        if is_reached:
            self.print_text(frame, "C4 activated", (self.w - 68, self.h - 35))

    def add_horizon(self, frame):
        horizon = self.shm.read("horizon")
        if horizon:
            (p1, p2) = horizon
            self.print_line(frame, p1, p2)

    def add_attitude(self, frame):
        roll = self.shm.read("roll")
        if roll:
            self.print_text(frame, f"Roll: {round(math.degrees(roll), 2)}", (10, 100))

        pitch = self.shm.read("pitch")
        if pitch:
            self.print_text(frame, f"Pitch: {round(math.degrees(pitch), 2)}", (10, 120))

    def put_ardu_info(self, frame):
        home = self.shm.read("home")
        pos = self.shm.read("pos")
        hud = self.shm.read("hud")
        sat_count = self.shm.read("sat_count")

        

        if home is not None and pos is not None:
            if pos["alt_rel_mm"] is not None:
                self.alt_rel_m = pos["alt_rel_mm"]/1000.0 # meters
            if all(v is not None for v in (home["lat"], home["lon"], pos["lat"], pos["lon"])):
                self.dist_to_home = haversine_m(pos["lat"], pos["lon"], home["lat"], home["lon"]) # meters
                self.dir_to_home  = bearing_deg(pos["lat"], pos["lon"], home["lat"], home["lon"]) # degree
                


        if hud is not None:
            self.airspeed = hud['airspeed']
            self.groudspeed = hud['groundspeed']
            # print(hud)
            pass

        if sat_count is not None:
            self.sat_count = sat_count

        def draw_home_arrow(frame, center, length, arrow_angle):
            # угол в радианах
            ang = math.radians(arrow_angle)
            x2 = int(center[0] + length * math.sin(ang))
            y2 = int(center[1] - length * math.cos(ang))
            cv2.arrowedLine(frame, center, (x2, y2), (0, 0, 255), 2, tipLength=0.3)
            return frame
        
        self.print_text(frame, f"Sats: {self.sat_count}", (2, 70))
        self.print_text(frame, f"Alt: {self.alt_rel_m if self.alt_rel_m is None else round(self.alt_rel_m, 2)}", (2, 80))
        self.print_text(frame, f"To Home: {self.dist_to_home if self.dist_to_home is None else round(self.dist_to_home, 2)}", (2, 95))
        self.print_text(frame, f"AirSpeed: {self.airspeed if self.airspeed is None else round(self.airspeed, 2)}", (2, 115))
        self.print_text(frame, f"GndSpeed: {self.groudspeed if self.groudspeed is None else round(self.groudspeed, 2)}", (2, 130))

        if self.dir_to_home is not None:
            draw_home_arrow(frame, (50, 200), 20, self.dir_to_home)


    
    def put_turbine_info(self, frame):
        t_telem = self.shm.read("turbine")

        if t_telem is not None:
            status = t_telem["status"]
            error = t_telem["error"]
            temp = t_telem["temp_C"]
            rpm = t_telem["rpm"]

            self.print_text(frame, f"T_STAT: {status}", (2, 70))
            self.print_text(frame, f"T_ERR: {error}", (2, 70))
            self.print_text(frame, f"T_TEMP: {temp}", (2, 70))
            self.print_text(frame, f"T_RPM: {rpm}", (2, 70))
            
    def put_info(self, frame):
        self.add_horizon(frame)
        self.add_center_pointer(frame)
        self.add_fps(frame)
        try:
            self.put_ardu_info(frame)
            self.put_turbine_info(frame)
        except Exception as err:
            print(err)
