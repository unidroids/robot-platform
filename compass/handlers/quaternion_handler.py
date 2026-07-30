# quaternion_handler.py
import struct
import zmq
import time
import json
from typing import Optional

class QuaternionHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._latest_q0 = 0.0
        self._latest_q1 = 0.0
        self._latest_q2 = 0.0
        self._latest_q3 = 0.0
        self._latest_ts = 0.0
        self._last_print_time = 0.0

    def handle(self, message_bytes: bytes):
        if len(message_bytes) != 11 or message_bytes[1] != 0x59:
            return

        # Data format: 0x55 0x59 Q0L Q0H Q1L Q1H Q2L Q2H Q3L Q3H SUM
        q0_raw, q1_raw, q2_raw, q3_raw = struct.unpack('<hhhh', message_bytes[2:10])

        self._latest_q0 = q0_raw / 32768.0
        self._latest_q1 = q1_raw / 32768.0
        self._latest_q2 = q2_raw / 32768.0
        self._latest_q3 = q3_raw / 32768.0

        current_time = time.monotonic()
        self._latest_ts = current_time
        
        json_data = json.dumps({
            "ts": current_time,
            "q0": self._latest_q0,
            "q1": self._latest_q1,
            "q2": self._latest_q2,
            "q3": self._latest_q3
        })
        msg = f"COMPASS/QUATER/{json_data}"
        
        if current_time - self._last_print_time > 1.0:
            print(f"[QuaternionHandler] {msg}")
            self._last_print_time = current_time
            
        try:
            self._zmq_pub.send_string(msg)
        except Exception:
            pass

    def get_latest(self) -> dict:
        return {
            "ts": self._latest_ts,
            "q0": self._latest_q0,
            "q1": self._latest_q1,
            "q2": self._latest_q2,
            "q3": self._latest_q3
        }
