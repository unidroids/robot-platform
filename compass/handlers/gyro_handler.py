# gyro_handler.py
import struct
import zmq
import time
import json

class GyroHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._latest_wx = 0.0
        self._latest_wy = 0.0
        self._latest_wz = 0.0
        self._latest_ts = 0.0
        self._last_print_time = 0.0

    def handle(self, message_bytes: bytes):
        if len(message_bytes) != 11 or message_bytes[1] != 0x52:
            return

        # Data format: 0x55 0x52 WxL WxH WyL WyH WzL WzH TL TH SUM
        wx_raw, wy_raw, wz_raw, temp_raw = struct.unpack('<hhhh', message_bytes[2:10])

        self._latest_wx = wx_raw / 32768.0 * 2000.0
        self._latest_wy = wy_raw / 32768.0 * 2000.0
        self._latest_wz = wz_raw / 32768.0 * 2000.0

        current_time = time.monotonic()
        self._latest_ts = current_time
        
        json_data = json.dumps({
            "ts": current_time,
            "wx": self._latest_wx,
            "wy": self._latest_wy,
            "wz": self._latest_wz
        })
        msg_payload = json_data.encode('utf-8')
        
        if current_time - self._last_print_time > 1.0:
            print(f"[GyroHandler] GYRO {json_data}")
            self._last_print_time = current_time
            
        try:
            self._zmq_pub.send_multipart([b"GYRO", msg_payload])
        except Exception:
            pass

    def get_latest(self) -> dict:
        return {
            "ts": self._latest_ts,
            "wx": self._latest_wx,
            "wy": self._latest_wy,
            "wz": self._latest_wz
        }
