# mag_handler.py
import struct
import zmq
import time
import json

class MagHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._latest_hx = 0
        self._latest_hy = 0
        self._latest_hz = 0
        self._last_print_time = 0.0

    def handle(self, message_bytes: bytes):
        if len(message_bytes) != 11 or message_bytes[1] != 0x54:
            return

        # Data format: 0x55 0x54 HxL HxH HyL HyH HzL HzH TL TH SUM
        hx_raw, hy_raw, hz_raw, temp_raw = struct.unpack('<hhhh', message_bytes[2:10])

        self._latest_hx = hx_raw
        self._latest_hy = hy_raw
        self._latest_hz = hz_raw

        current_time = time.time()
        
        json_data = json.dumps({
            "ts": current_time,
            "hx": self._latest_hx,
            "hy": self._latest_hy,
            "hz": self._latest_hz
        })
        msg = f"COMPASS/MAG/{json_data}"
        
        if current_time - self._last_print_time > 1.0:
            print(f"[MagHandler] {msg}")
            self._last_print_time = current_time
            
        try:
            self._zmq_pub.send_string(msg)
        except Exception:
            pass

    def get_latest(self) -> dict:
        return {
            "ts": time.time(),
            "hx": self._latest_hx,
            "hy": self._latest_hy,
            "hz": self._latest_hz
        }
