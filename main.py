import logging
from multiprocessing import Process
from shared_memory import SharedMemory
from capture.capture_ctrl import CaptureController
from streamer.stream_ctrl import StreamController
from osd import OSDController
from ardupilot_telem import ArduPilotBase
from turbine_telem import TurbineTelem
import time


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
    drone = ArduPilotBase(port, baud)

    try:
        drone.connect()
    except:
        time.sleep(5)
        drone.connect()

    while True:
        drone.listen_feed()

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

        # drone.takeoff_time_utc_us


def launch():
    shm = SharedMemory()

    cap = CaptureController()

    # For Test: "gst-launch-1.0 -v videotestsrc pattern=ball ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    # For MacOS: "gst-launch-1.0 avfvideosrc device-index=0 ! videoconvert ! video/x-raw,width=640,height=480,format=BGR ! appsink drop=1"
    # For Linux: "gst-launch-1.0 v4l2src device=/dev/video1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    gst_str = "gst-launch-1.0 -v videotestsrc pattern=ball ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    cap.load_param(gst_str)
    cap.connect()

    if not cap.is_cap():
        logging.error("[ CAPTURE ] Not able to connect camera!. Exit...")
        exit()

    (w, h) = cap.get_param()
    shm.config["width"] = w
    shm.config["height"] = h

    drone_port = "/dev/cu.usbmodem101"
    drone_baud = 115200
    autopilot = Process(target=listen_telem, args=(shm, drone_port, drone_baud))
    autopilot.start()

    turbine_port = "/dev/cu.usbmodem103"
    turbine_baud = 115200
    turbine = Process(target=listen_turbine, args=(shm, turbine_port, turbine_baud))
    turbine.start()

    out = StreamController()
    host = "127.0.0.1"
    port = 5602
    out.load_param(w, h, host, port)
    out.create()

    osd = OSDController(shm)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            osd.put_info(frame)
            out.write(frame)
    except NameError as err:
        print(f"VideoStream ERR: {err}")
    finally:
        cap.release()
        out.release()
        autopilot.terminate()
        turbine.terminate()


if __name__ == "__main__":
    launch()
