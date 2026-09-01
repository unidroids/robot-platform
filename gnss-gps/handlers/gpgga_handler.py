import zmq
import json

class GpggaHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._last_json = "{}"

    def get_last_json(self) -> str:
        return self._last_json

    def handle(self, message: str) -> bool:
        parts = message.split(',')
        if not (len(parts) == 15 and len(parts[0]) == 6 and parts[0].startswith("$") and parts[0].endswith("GGA")):
            print(f"[GGA] Invalid format | Msg: {message}")
            return False
            
        try:
            data = {
                "lat": self._parse_coord(parts[2], parts[3]),
                "lon": self._parse_coord(parts[4], parts[5]),
                "fix": int(parts[6]) if parts[6] else 0,
                "sats": int(parts[7]) if parts[7] else 0,
                "raw": message
            }
            json_data = json.dumps(data)
            self._last_json = json_data
            self._zmq_pub.send_multipart([b"GPGGA", json_data.encode('utf-8')])
            print(f"[GGA] {message}")
            return True
            
        except Exception as e:
            print(f"[GGA] Parse error: {e} | Msg: {message}")
            return False

    def _parse_coord(self, coord_str: str, dir_char: str) -> float:
        if not coord_str:
            return 0.0
        dot_idx = coord_str.find('.')
        if dot_idx < 2:
            return 0.0
        deg = float(coord_str[:dot_idx-2])
        mins = float(coord_str[dot_idx-2:])
        val = deg + mins / 60.0
        if dir_char in ['S', 'W']:
            val = -val
        return val
