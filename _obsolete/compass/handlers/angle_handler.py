# angle_handler.py
import struct
import zmq
import time
import json
from typing import Optional

class AngleHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._latest_roll = 0.0
        self._latest_pitch = 0.0
        self._latest_yaw = 0.0
        self._latest_ts = 0.0
        self._last_print_time = 0.0

    def handle(self, message_bytes: bytes):
        if len(message_bytes) != 11 or message_bytes[1] != 0x53:
            return

        # Data format: 0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
        roll_raw, pitch_raw, yaw_raw, version_raw = struct.unpack('<hhhh', message_bytes[2:10])

        self._latest_roll = roll_raw / 32768.0 * 180.0
        self._latest_pitch = pitch_raw / 32768.0 * 180.0
        self._latest_yaw = yaw_raw / 32768.0 * 180.0

        current_time = time.monotonic()
        self._latest_ts = current_time
        
        json_data = json.dumps({
            "ts": current_time,
            "roll": self._latest_roll,
            "pitch": self._latest_pitch,
            "yaw": self._latest_yaw
        })
        msg_payload = json_data.encode('utf-8')
        
        if current_time - self._last_print_time > 1.0:
            print(f"[AngleHandler] ANGLE {json_data}")
            self._last_print_time = current_time
            
        try:
            self._zmq_pub.send_multipart([b"ANGLE", msg_payload])
        except Exception:
            pass

    def get_latest(self) -> dict:
        return {
            "ts": self._latest_ts,
            "roll": self._latest_roll,
            "pitch": self._latest_pitch,
            "yaw": self._latest_yaw
        }
