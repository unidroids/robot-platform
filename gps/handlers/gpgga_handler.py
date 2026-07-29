import zmq
import json

class GpggaHandler:
    def __init__(self, zmq_pub: zmq.Socket, service):
        self._zmq_pub = zmq_pub
        self._service = service

    def handle(self, message: str) -> None:
        parts = message.split(',')
        json_data = "{}"
        try:
            if len(parts) > 10 and parts[0] == "$GPGGA":
                data = {
                    "lat": self._parse_coord(parts[2], parts[3]),
                    "lon": self._parse_coord(parts[4], parts[5]),
                    "fix": int(parts[6]) if parts[6] else 0,
                    "sats": int(parts[7]) if parts[7] else 0,
                    "raw": message
                }
                json_data = json.dumps(data)
                self._service.update_last_gpgga(json_data)
        except Exception as e:
            print(f"[GPGGA] Parse error: {e}")
            json_data = json.dumps({"error": "parse_error", "raw": message})
            
        self._zmq_pub.send_string(f"GPGGA/{json_data}")
        print(f"[GPGGA] {message}")

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
