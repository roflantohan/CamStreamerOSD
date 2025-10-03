import logging
from logging.handlers import RotatingFileHandler

from multiprocessing import Process
from shared_memory import SharedMemory
from capture.capture_ctrl import CaptureController
from streamer.stream_ctrl import StreamController
from osd import OSDController
from ardupilot_telem import ArduPilotBase
from turbine_telem import TurbineTelem
from skyline_finder import SkyLineFinder
import time


def setup_logger(name):
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    log_filename = f"./{name}_log.log"

    handler = RotatingFileHandler(log_filename, maxBytes=1000000, backupCount=10)
    handler.setLevel(logging.DEBUG)

    formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    handler.setFormatter(formatter)

    logger.addHandler(handler)
    return logger


def listen_turbine(shm: SharedMemory, port, baud):
    turbine = TurbineTelem(port, baud)

    try:
        turbine.connect()
    except:
        time.sleep(5)
        turbine.connect()

    while True:
        turbine.step()
        shm.write("turbine", turbine.telem)


def listen_telem(shm: SharedMemory, port, baud):
    logger = setup_logger("ardu")
    drone = ArduPilotBase(port, baud)

    try:
        drone.connect()
    except:
        time.sleep(5)
        drone.connect()

    while True:
        drone.listen_feed()

        shm.write("flight_mode", drone.flight_mode)
        shm.write("roll", drone.attitude[0])
        shm.write("pitch", drone.attitude[1])
        shm.write("home_lat", drone.home_lat)
        shm.write("home_lon", drone.home_lon)
        shm.write("home_alt", drone.home_alt)
        shm.write("pos_lat", drone.pos_lat)
        shm.write("pos_lon", drone.pos_lon)
        shm.write("pos_alt_rel", drone.pos_alt_rel)
        shm.write("hud_airspeed", drone.hud_airspeed)
        shm.write("hud_groundspeed", drone.hud_groundspeed)
        shm.write("hud_heading", drone.hud_heading)
        shm.write("hud_alt", drone.hud_alt)
        shm.write("hud_climb", drone.hud_climb)
        shm.write("hud_throttle", drone.hud_throttle)
        shm.write("sat_count", drone.sat_count)
        shm.write("armed", drone.armed)
        shm.write("time_armed_us", drone.time_armed_us)
        shm.write("time_boot_us", drone.time_boot_us)

        t_telem = shm.read("turbine")

        if t_telem is not None:
            status_code = t_telem["status_code"]
            error_code = t_telem["error_code"]
            rpm = t_telem["rpm"]
            temp_c = t_telem["temp_C"]
            rc_V = t_telem["rc_V"]
            pwr_V = t_telem["pwr_V"]
            pump_V = t_telem["pump_V"]
            thro_per = t_telem["throttle_%"]
            current_A = t_telem["current_A"]
            ecu_temp_C = t_telem["ecu_temp_C"]
            startup_time_s = t_telem["startup_time_s"]

            # drone.send_float("t_stat", status_code)
            # drone.send_float("t_err", error_code)
            # drone.send_float("t_rpm", rpm)
            # drone.send_float("t_temp", temp_c)
            # drone.send_float("t_rc", rc_V)
            # drone.send_float("t_pwr", pwr_V)
            # drone.send_float("t_pump", pump_V)
            # drone.send_float("t_thr", thro_per)
            # drone.send_float("t_curr", current_A)
            # drone.send_float("t_t_ecu", ecu_temp_C)
            # drone.send_float("t_time", startup_time_s)

            logger.info(
                f"{drone.time_boot_us};{status_code};{error_code};{rpm};{temp_c};{rc_V};{pwr_V};{pump_V};{thro_per};{current_A};{ecu_temp_C};{startup_time_s}"
            )


def launch():
    shm = SharedMemory()

    cap = CaptureController()

    # For Test: "gst-launch-1.0 -v videotestsrc pattern=ball ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    # For MacOS: "gst-launch-1.0 avfvideosrc device-index=0 ! videoconvert ! video/x-raw,width=640,height=480,format=BGR ! appsink drop=1"
    # For Linux: "gst-launch-1.0 v4l2src device=/dev/video1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    gst_str = "gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    cap.load_param(gst_str)
    cap.connect()

    if not cap.is_cap():
        logging.error("[ CAPTURE ] Not able to connect camera!. Exit...")
        exit()

    (w, h) = cap.get_param()
    shm.config["width"] = w
    shm.config["height"] = h

    drone_port = "/dev/ttyAMA0"
    drone_baud = 115200
    autopilot = Process(target=listen_telem, args=(shm, drone_port, drone_baud))
    autopilot.start()

    turbine_port = "/dev/ttyUSB0"
    turbine_baud = 9600
    turbine = Process(target=listen_turbine, args=(shm, turbine_port, turbine_baud))
    turbine.start()

    out = StreamController()
    host = "127.0.0.1"
    # host = "192.168.0.101"
    port = 5602
    out.load_param(w, h, host, port)
    out.create()

    osd = OSDController(shm)

    lvl = SkyLineFinder(w, h)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            roll = shm.read("roll")
            pitch = shm.read("pitch")
            lvl.find_horizon(roll, pitch)
            shm.write("horizon", lvl.get_line())
            osd.put_info(frame)
            out.write(frame)
    except NameError as err:
        print(f"VideoStream ERR: {err}")
    finally:
        cap.release()
        out.release()
        # autopilot.terminate()
        # turbine.terminate()


if __name__ == "__main__":
    launch()
