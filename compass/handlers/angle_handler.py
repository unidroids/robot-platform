# angle_handler.py
import struct
import zmq
import time
from typing import Optional

class AngleHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._latest_roll = 0.0
        self._latest_pitch = 0.0
        self._latest_yaw = 0.0
        self._last_print_time = 0.0

    def handle(self, message_bytes: bytes):
        if len(message_bytes) != 11 or message_bytes[1] != 0x53:
            return

        # Data format: 0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
        roll_raw, pitch_raw, yaw_raw, version_raw = struct.unpack('<hhhh', message_bytes[2:10])

        self._latest_roll = roll_raw / 32768.0 * 180.0
        self._latest_pitch = pitch_raw / 32768.0 * 180.0
        self._latest_yaw = yaw_raw / 32768.0 * 180.0

        # Send string to ZMQ topic ANGLE
        msg = f"ANGLE/R,{self._latest_roll:.4f},P,{self._latest_pitch:.4f},Y,{self._latest_yaw:.4f}"
        
        current_time = time.time()
        if current_time - self._last_print_time > 1.0:
            print(f"[AngleHandler] {msg}")
            self._last_print_time = current_time
            
        try:
            self._zmq_pub.send_string(msg)
        except Exception:
            pass

    def get_latest(self) -> str:
        return f"{self._latest_roll:.4f},{self._latest_pitch:.4f},{self._latest_yaw:.4f}"
