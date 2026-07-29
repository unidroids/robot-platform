import time
import zmq
import json

class BestnavaHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._last_log_time = 0

    def handle(self, message: str) -> None:
        json_data = json.dumps({"raw": message})
        self._zmq_pub.send_string(f"BESTNAVA/{json_data}")
        
        current_time = time.time()
        if current_time - self._last_log_time >= 1.0:
            print(f"[BESTNAVA] {message}")
            self._last_log_time = current_time
