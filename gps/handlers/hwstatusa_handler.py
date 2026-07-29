import zmq
import json

class HwstatusaHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub

    def handle(self, message: str) -> None:
        json_data = json.dumps({"raw": message})
        self._zmq_pub.send_string(f"HWSTATUSA/{json_data}")
        print(f"[HWSTATUSA] {message}")
