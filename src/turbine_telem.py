#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from dataclasses import dataclass
from typing import Dict
import serial
from shared_memory import SharedMemory

# ----- CRC8 lookup (как в прошивке) -----
CRC8_TABLE = [
    0x00,0x5E,0xBC,0xE2,0x61,0x3F,0xDD,0x83,0xC2,0x9C,0x7E,0x20,0xA3,0xFD,0x1F,0x41,
    0x9D,0xC3,0x21,0x7F,0xFC,0xA2,0x40,0x1E,0x5F,0x01,0xE3,0xBD,0x3E,0x60,0x82,0xDC,
    0x23,0x7D,0x9F,0xC1,0x42,0x1C,0xFE,0xA0,0xE1,0xBF,0x5D,0x03,0x80,0xDE,0x3C,0x62,
    0xBE,0xE0,0x02,0x5C,0xDF,0x81,0x63,0x3D,0x7C,0x22,0xC0,0x9E,0x1D,0x43,0xA1,0xFF,
    0x46,0x18,0xFA,0xA4,0x27,0x79,0x9B,0xC5,0x84,0xDA,0x38,0x66,0xE5,0xBB,0x59,0x07,
    0xDB,0x85,0x67,0x39,0xBA,0xE4,0x06,0x58,0x19,0x47,0xA5,0xFB,0x78,0x26,0xC4,0x9A,
    0x65,0x3B,0xD9,0x87,0x04,0x5A,0xB8,0xE6,0xA7,0xF9,0x1B,0x45,0xC6,0x98,0x7A,0x24,
    0xF8,0xA6,0x44,0x1A,0x99,0xC7,0x25,0x7B,0x3A,0x64,0x86,0xD8,0x5B,0x05,0xE7,0xB9,
    0x8C,0xD2,0x30,0x6E,0xED,0xB3,0x51,0x0F,0x4E,0x10,0xF2,0xAC,0x2F,0x71,0x93,0xCD,
    0x11,0x4F,0xAD,0xF3,0x70,0x2E,0xCC,0x92,0xD3,0x8D,0x6F,0x31,0xB2,0xEC,0x0E,0x50,
    0xAF,0xF1,0x13,0x4D,0xCE,0x90,0x72,0x2C,0x6D,0x33,0xD1,0x8F,0x0C,0x52,0xB0,0xEE,
    0x32,0x6C,0x8E,0xD0,0x53,0x0D,0xEF,0xB1,0xF0,0xAE,0x4C,0x12,0x91,0xCF,0x2D,0x73,
    0xCA,0x94,0x76,0x28,0xAB,0xF5,0x17,0x49,0x08,0x56,0xB4,0xEA,0x69,0x37,0xD5,0x8B,
    0x57,0x09,0xEB,0xB5,0x36,0x68,0x8A,0xD4,0x95,0xCB,0x29,0x77,0xF4,0xAA,0x48,0x16,
    0xE9,0xB7,0x55,0x0B,0x88,0xD6,0x34,0x6A,0x2B,0x75,0x97,0xC9,0x4A,0x14,0xF6,0xA8,
    0x74,0x2A,0xC8,0x96,0x15,0x4B,0xA9,0xF7,0xB6,0xE8,0x0A,0x54,0xD7,0x89,0x6B,0x35,
]

def crc8(buf: bytes) -> int:
    c = 0
    for b in buf:
        c = CRC8_TABLE[c ^ b]
    return c

# ---- Константы команд/состояний ----
CMD_EMPTY   = 0x0  # keep-alive (Cmd ID 0)
CMD_PARAM   = 0x2  # короткие команды/параметры (Cmd ID 2)

# параметры для CMD_PARAM (ID2)
PARAM_SEND_20 = 7
PARAM_SEND_50 = 8
PARAM_SEND_100 = 9

SW_DISABLE = 0
SW_STOP    = 1
SW_READY   = 2
SW_START   = 3

ENGINE_STATE_MAP = {
    0: "Stop",
    1: "Standby/AutoCool",
    2: "Ignition(min throttle)",
    3: "Ignition",
    4: "Preheat",
    5: "Fuel ramp",
    6: "Run(Learn,max thr)",
    7: "Run(Learn,min thr)",
    8: "Run(Idle)",
    9: "Run(min thr)",
    10: "Run(pump limit warn)",
    11: "Run",
    12: "Cooling",
    13: "Restart",
    14: "Test Glow",
    15: "Test MainValve",
    16: "Test IgnValve",
    17: "Test Pump",
    18: "Test Starter",
    19: "Fuel exhaust air",
}

ECODE_MAP = {
    0: "OK",
    1: "Timeout",
    2: "Voltage low",
    3: "Glowplug fail",
    4: "Pump fail",
    5: "Starter fail",
    6: "RPM low",
    7: "RPM unstable",
    8: "Exhaust temp HIGH",
    9: "Exhaust temp LOW",
    10: "EGT sensor fail",
    11: "Ign valve fail",
    12: "Main valve fail",
    13: "Control signal lost",
    14: "Starter Ctrl Temp HIGH",
    15: "Pump Ctrl Temp HIGH",
    16: "Clutch fail",
    17: "Current overload",
    18: "Engine offline",
}

@dataclass
class Telemetry:
    # базовое
    engine_rpm: int = 0
    engine_status: int = 0
    error_code: int = 0
    engine_temp_c: int = 0
    switch_state: int = 0
    # напряжения/ток/газ/давление
    rc_v: int = 0       # сырой байт
    pwr_v: int = 0
    pump_v: int = 0
    throttle_pct: int = 0
    current_deci_a: int = 0
    pressure_raw2: int = 0   # press*2 (Па)
    # параметры
    pump_ign_centivolt: int = 0
    curve_inc: int = 0
    curve_dec: int = 0
    engine_max_rpm: int = 0
    pump_max_v_raw: int = 0
    version: int = 0
    update_rate_code: int = 0
    # доп. кадры
    flow_rate_centilpm: int = 0    # 0.01 L/min
    flow_total_decil: int = 0      # 0.1 L
    idle_rpm: int = 0
    esr_need_pressure: int = 0
    rpm_closed_loop: int = 0
    startup_time_deci_s: int = 0
    ecu_temp_c: int = 0
    propeller_rpm: int = 0
    pump_rpm: int = 0
    thrust: int = 0 # deci kg 1 = 0.1 kg
    # масштаб напряжений (зависит от версии: <=3 -> 0.1V, >=4 -> 0.2V)
    v_scale_deciv: int = 1


    def scaled(self) -> Dict[str, float]:
        V = 0.1 * (2 if self.version >= 4 else 1)

        return {
            "status_code": self.engine_status,
            "error_code": self.error_code,
            "rpm": float(self.engine_rpm),
            "status": ENGINE_STATE_MAP[self.engine_status],
            "error": ECODE_MAP[self.error_code],
            "temp_C": float(self.engine_temp_c),
            "switch_state": self.switch_state,
            "rc_V": self.rc_v * V,
            "pwr_V": self.pwr_v * V,
            "pump_V": self.pump_v * V,
            "throttle_%": float(self.throttle_pct),
            "current_A": self.current_deci_a / 10.0,
            "pressure_kPa": self.pressure_raw2 / 1000.0,
            "pump_ignition_V": self.pump_ign_centivolt / 100.0,
            "curve_inc": self.curve_inc,
            "curve_dec": self.curve_dec,
            "max_rpm": float(self.engine_max_rpm),
            "pump_max_V": self.pump_max_v_raw * V,
            "proto_ver": self.version,
            "srate_code": self.update_rate_code,  # 0=20,1=50,2=100
            "flow_rate_Lmin": self.flow_rate_centilpm / 100.0,
            "flow_total_L": self.flow_total_decil / 10.0,
            "idle_rpm": float(self.idle_rpm),
            "ESR_need_pressure": self.esr_need_pressure,
            "RPM_closed_loop": self.rpm_closed_loop,
            "ecu_temp_C": float(self.ecu_temp_c),
            "propeller_rpm": float(self.propeller_rpm),
            "pump_rpm": float(self.pump_rpm),
            "thrust": self.thrust / 10.0 
        }

class Parser:
    """Парсер 7-байтных кадров ECU: byte0=0xF0|ID, byte1..5 payload, byte6 CRC8(byte0..5)."""
    def __init__(self):
        self.buf = bytearray(6)
        self.cnt = 0
        self.t = Telemetry()

    def _state_comm(self):
        rpm = (self.buf[1] | (self.buf[2] << 8)) & 0xFFFF
        self.t.engine_rpm = rpm * 10  # общая часть для всех ID

    def _id1(self):
        self.t.engine_status = self.buf[3] & 0x1F
        self.t.error_code = ((self.buf[3] >> 5) & 0x07) | ((self.buf[4] & 0x03) << 3)
        self.t.engine_temp_c = (((self.buf[4] & 0x1C) << 6) | self.buf[5]) - 50
        self.t.switch_state = (self.buf[4] >> 5) & 0x03

    def _id2(self):
        self.t.rc_v = self.buf[3]
        self.t.pwr_v = self.buf[4]
        self.t.pump_v = self.buf[5]

    def _id3(self):
        self.t.throttle_pct = min(self.buf[3], 100)
        press = (self.buf[4] | (self.buf[5] << 8)) & 0xFFFF
        self.t.pressure_raw2 = press * 2

    def _id4(self):
        cur = (self.buf[3] | (self.buf[4] << 8)) & 0x1FF
        self.t.current_deci_a = cur
        thrust_deci_kg = (((self.buf[4] & 0xFE) >> 1) | (self.buf[5] << 7)) & 0x7FFF
        self.t.thrust = thrust_deci_kg # 0.1 kg deci kilogram

    def _id5(self):
        self.t.pump_ign_centivolt = int(self.buf[3]) * 2  # 0.02 В * N
        self.t.curve_inc = self.buf[4]
        self.t.curve_dec = self.buf[5]

    def _id6(self):
        self.t.engine_max_rpm = int(self.buf[3]) * 1000
        self.t.pump_max_v_raw = self.buf[4]
        self.t.version = (self.buf[5] >> 2) & 0x3F
        self.t.update_rate_code = self.buf[5] & 0x03

    def _id7(self):
        self.t.flow_rate_centilpm = ((self.buf[4] & 0x03) << 8) | self.buf[3]
        self.t.flow_total_decil = (self.buf[5] << 2) | (self.buf[4] >> 6)

    def _id8(self):
        self.t.idle_rpm = int(self.buf[3]) * 1000
        self.t.esr_need_pressure = (self.buf[4] >> 5) & 0x01
        self.t.rpm_closed_loop   = (self.buf[4] >> 4) & 0x01
        # self.t.startup_time_deci_s = ((self.buf[4] & 0x0F) << 8) | self.buf[5]

    def _id9(self):
        self.t.ecu_temp_c = int(self.buf[3]) - 50
        self.t.propeller_rpm = (self.buf[4] | (self.buf[5] << 8)) & 0xFFFF

    def _id10(self):
        self.t.pump_rpm = (self.buf[4] | (self.buf[5] << 8)) & 0xFFFF

    def feed(self, b: int) -> bool:
        """Вернёт True при принятии валидного кадра."""
        updated = False
        if self.cnt >= len(self.buf):
            if ((self.buf[0] & 0xF0) == 0xF0) and (crc8(self.buf) == b):
                updated = True
                self.cnt = 0
                self._state_comm()
                id_ = self.buf[0] & 0x0F
                {1:self._id1,2:self._id2,3:self._id3,4:self._id4,
                 5:self._id5,6:self._id6,7:self._id7,8:self._id8,
                 9:self._id9,10:self._id10}.get(id_, lambda:None)()
            else:
                self.buf[:-1] = self.buf[1:]
                self.buf[-1] = b
        else:
            self.buf[self.cnt] = b
            self.cnt += 1
        return updated

# ----- Формирование исходящих кадров (4 байта) -----
def pkt4(cmd_id: int, b2: int, b3: int) -> bytes:
    # byte0=0xFF; byte1: верхняя тетрада = cmd_id, нижняя — флаги/старшие биты по команде
    # CRC считается по byte1..byte2 (2 байта)
    b1 = (cmd_id & 0x0F) << 4
    b2 &= 0xFF
    c = crc8(bytes([b1, b2]))
    return bytes([0xFF, b1, b2, c])

def pkt_param(param: int) -> bytes:
    return pkt4(CMD_PARAM, param & 0xFF, 0)

def pkt_empty() -> bytes:
    return pkt4(CMD_EMPTY, 0, 0)

def pkt_set_update_rate(code: int) -> bytes:
    # 0:20Hz / 1:50Hz / 2:100Hz -> в протоколе через CMD_PARAM (7/8/9)
    param = {0:PARAM_SEND_20, 1:PARAM_SEND_50, 2:PARAM_SEND_100}[code]
    return pkt_param(param)

def open_serial(port: str, baud: int, stopbits: int) -> serial.Serial:
    return serial.Serial(
        port=port, baudrate=baud,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO if stopbits==2 else serial.STOPBITS_ONE,
        timeout=0, write_timeout=0
    )

def run(port: str, baud: int, stopbits: int, send_rate_hz: float):
    
    ser = open_serial(port, baud, stopbits)
    prs = Parser()

    next_tx = 0.0
    try:
        while True:
            # RX
            data = ser.read(512)
            for b in data:
                if prs.feed(b):
                    pass  # при желании — событие «кадр принят»

            now = time.monotonic()
            # TX
            if now >= next_tx:
                ser.write(pkt_empty())
                next_tx = now + (1.0 / send_rate_hz)

            # PRINT
            t = prs.t.scaled()

            
            print(
                "RPM={rpm:.0f}  T={temp_C:.0f}°C  Thr={throttle_%:.0f}%  "
                "I={current_A:.1f}A  RC/PWR/PUMP={rc_V:.1f}/{pwr_V:.1f}/{pump_V:.1f}V  "
                "P={pressure_kPa:.3f}kPa  SW={switch_state}  ver={proto_ver} upd={srate_code}  "
                "Flow={flow_rate_Lmin:.2f}L/min  Fuel={flow_total_L:.1f}L  "
                "Idle={idle_rpm:.0f}  ESR={ESR_need_pressure} CL={RPM_closed_loop} "
                "t_start={startup_time_s:.1f}s  ECU_T={ecu_temp_C:.0f}°C  Prop={propeller_rpm:.0f}  PumpRPM={pump_rpm:.0f}"
                .format(**t)
            )

            if prs.t.update_rate_code != 2:
                ser.write(pkt_set_update_rate(2))

            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


class TurbineTelem:

    def __init__(self, port, baud=9600):
        self.port = port
        self.baud = baud
        self.stopbits = 2
        self.send_rate_hz = 20
        self.ser = None
        self.prs = Parser()
        self.next_tx = 0.0
        self.telem = None
        self.time_boot_us = 0
        self.time_armed_us = None
        self.armed = False

    def connect(self):
        try:
            self.ser = open_serial(self.port, self.baud, self.stopbits)
        except:
            time.sleep(5)
            self.connect()

    def step(self):
        self.time_boot_us = time.time()
        # RX
        data = self.ser.read(512)
        for b in data:
            if self.prs.feed(b):
                pass  # при желании — событие «кадр принят»

        now = time.monotonic()
        # TX
        if now >= self.next_tx:
            self.ser.write(pkt_empty())
            self.next_tx = now + (1.0 / self.send_rate_hz)

        # PRINT
        self.telem = self.prs.t.scaled()
        is_armed = "Run" in self.telem["status"]
        if is_armed and not self.armed:
            self.armed = True
            self.time_armed_us = self.time_boot_us
        elif not is_armed and self.armed:
            self.armed = False
            self.time_armed_us = None
        self.telem["startup_time_s"] = self.time_boot_us - self.time_armed_us

        if self.prs.t.update_rate_code != 2:
            self.ser.write(pkt_set_update_rate(2))

        time.sleep(0.001)


