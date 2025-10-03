import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase


class ArduPilotBase:
    def __init__(self, path: str, baud: int):
        self.path = path
        self.baud = baud

        self.master = None
        self.boot_t = 0
        self.flight_mode = ""
        self.is_autopilot = False

        self.attitude = [0, 0, 0]
        self.attitude_deg = [0, 0, 0]
        self.pitch_limit = [0, 0]
        self.roll_limit = [0, 0]
        self.yaw_limit = [0, 0]

        self.count_chans = 18
        self.channels = [0] * self.count_chans

        self.custom_name = ""
        self.custom_value = ""

        self.armed = False
        self.time_armed_us = 0
        self.time_boot_us = 0
        # #OSD

        self.home_lat = None
        self.home_lon = None
        self.home_alt = None  # mm

        self.pos_lat = None
        self.pos_lon = None
        self.pos_alt_rel = None  # mm

        self.hud_airspeed = None
        self.hud_groundspeed = None
        self.hud_heading = None
        self.hud_alt = None
        self.hud_climb = None
        self.hud_throttle = None

        self.sat_count = None

    def connect(self):
        self.master = mavutil.mavlink_connection(self.path, self.baud)
        self.master.wait_heartbeat()
        self.boot_t = time.time()

        # msg_id = mavutil.mavlink.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT
        # self.request_msg_stream(msg_id)

        # msg_id = 264
        # self.request_msg_stream(msg_id)

    def wait_params(self):
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component
        )

        param_count = 3
        is_params = [False] * param_count
        received_params = {}
        while True:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True).to_dict()
            (id, value) = (msg["param_id"], msg["param_value"])
            received_params[msg["param_index"]] = id

            if len(received_params) == msg["param_count"]:
                break

            if all(is_params):
                break

            if id == "ROLL_LIMIT_DEG":
                self.roll_limit = [-value, value]
                is_params[0] = True
            if id == "PTCH_LIM_MIN_DEG":
                self.pitch_limit = [value, self.pitch_limit[1]]
                is_params[1] = True
            if id == "PTCH_LIM_MAX_DEG":
                self.pitch_limit = [self.pitch_limit[0], value]
                is_params[2] = True

    def get_flight_mode(self):
        return self.flight_mode

    def get_attitude(self):
        return self.attitude

    def get_custom_param(self):
        return (self.custom_name, self.custom_value)

    def get_rc_channels(self):
        return self.channels

    def listen_feed(self):
        msg = self.master.recv_match(blocking=False)

        if not msg:
            return

        msg_type = msg.get_type()

        if msg_type == "SYSTEM_TIME":
            self.time_boot_us = msg.time_boot_ms * 1000
            return msg_type

        if msg_type == "HEARTBEAT":
            self.flight_mode = self.master.flightmode
            is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if is_armed and not self.armed:
                self.armed = True
                self.time_armed_us = self.time_boot_us
            elif not is_armed and self.armed:
                self.armed = False
                self.time_armed_us = None
            return msg_type

        if msg_type == "ATTITUDE":
            self.attitude = [msg.roll, msg.pitch, msg.yaw]
            self.attitude_deg = [
                math.degrees(msg.roll),
                math.degrees(msg.pitch),
                math.degrees(msg.yaw),
            ]
            return msg_type

        if msg_type == "VFR_HUD":
            self.hud_airspeed = msg.airspeed  # m/s
            self.hud_groundspeed = msg.groundspeed  # m/s
            self.hud_heading = msg.heading  # deg
            self.hud_alt = msg.alt  # m (AMSL ?)
            self.hud_climb = msg.climb  # m/s
            self.hud_throttle = msg.throttle  # (0 to 100)
            return msg_type

        if msg_type == "HOME_POSITION":
            self.home_lat = msg.latitude * 1e-7
            self.home_lon = msg.longitude * 1e-7
            self.home_alt = msg.altitude
            return msg_type

        if msg_type == "GLOBAL_POSITION_INT":
            self.pos_lat = msg.lat * 1e-7
            self.pos_lon = msg.lon * 1e-7
            self.pos_alt_rel = msg.relative_alt  # relative home
            return msg_type

        if msg_type == "GPS_RAW_INT":
            self.sat_count = msg.satellites_visible
            return msg_type

        if msg_type == "RC_CHANNELS":
            self.channels = list(
                getattr(msg, f"chan{i+1}_raw") for i in range(self.count_chans)
            )
            return msg_type

        if msg_type == "NAMED_VALUE_FLOAT":
            self.custom_name = msg.name
            self.custom_value = msg.value
            return msg_type

    def request_msg_stream(self, msg_id, interval=20000):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval,  # frequency in microseconds (200000 = 5 Hz)
            0,
            0,
            0,
            0,
            0,
        )

    def set_target_attitude(self, thrust, roll, pitch, yaw=0):
        """Sets the target attitude while in depth-hold mode.
        'roll', 'pitch', and 'yaw' are angles in radians.
        """
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_t)),
            self.master.target_system,
            self.master.target_component,
            0b00000111,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([roll, pitch, yaw]),
            0,
            0,
            0,
            thrust,
        )

    def set_target_servo(self, servo_chan=1, pwm_value=1500):
        """Sets the target pwm for n-servo.
        'servo_chan' - [1; 16], 'pwm_value' - [1000; 2000]
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_chan,
            pwm_value,
            0,
            0,
            0,
            0,
            0,
        )

    def set_target_rc(self, channel_id, pwm=1500):
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system,  # target_system
            self.master.target_component,  # target_component
            *rc_channel_values,
        )  # RC channel list, in microseconds.

    def send_float(self, key, value):
        self.master.mav.named_value_float_send(
            int(self.time_boot_us / 1000), key.encode("ascii")[:10], float(value)
        )
