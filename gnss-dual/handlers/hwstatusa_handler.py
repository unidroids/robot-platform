import zmq
import json

class HwstatusaHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._last_json = "{}"

    def get_last_json(self) -> str:
        return self._last_json

    def _parse_message(self, message: str) -> dict:
        if ";" not in message:
            return {"error": "no_header"}
            
        header, data = message.split(";", 1)
        if "*" in data:
            data = data.split("*")[0]
            
        header_parts = header.split(",")
        data_parts = data.split(",")
        
        if len(header_parts) < 10 or len(data_parts) < 12:
            return {"error": "incomplete_data"}
            
        msg_name = header_parts[0]
        sync = msg_name[0] if msg_name.startswith("#") else ""
        if sync == "#":
            msg_name = msg_name[1:]
            
        return {
            "sync": sync,
            "message": msg_name,
            "cpu_idle": int(header_parts[1]) if header_parts[1] else 0,
            "time_ref": header_parts[2],
            "time_status": header_parts[3],
            "wn": int(header_parts[4]) if header_parts[4] else 0,
            "ms": int(header_parts[5]) if header_parts[5] else 0,
            "version": header_parts[6],
            "reserved_hdr": header_parts[7],
            "leap_sec": int(header_parts[8]) if header_parts[8] else 0,
            "output_delay": int(header_parts[9]) if header_parts[9] else 0,
            
            "reserved_data_1": data_parts[0],
            "dc09": float(data_parts[1]) if data_parts[1] else 0.0,
            "dc10": float(data_parts[2]) if data_parts[2] else 0.0,
            "dc18": float(data_parts[3]) if data_parts[3] else 0.0,
            "clock_flag": int(data_parts[4]) if data_parts[4] else 0,
            "clock_drift": float(data_parts[5]) if data_parts[5] else 0.0,
            "reserved_data_2": float(data_parts[6]) if data_parts[6] else 0.0,
            "hw_flag": data_parts[7],
            "reserved_data_3": data_parts[8],
            "pll_lock": data_parts[9],
            "reserved_data_4": data_parts[10],
            "reserved_data_5": data_parts[11]
        }

    def to_json(self, parsed_data: dict) -> str:
        return json.dumps(parsed_data)

    def handle(self, message: str) -> bool:
        try:
            parsed_data = self._parse_message(message)
            if "error" in parsed_data:
                print(f"[HWSTATUSA] Error: {parsed_data['error']} | Msg: {message}")
                return False
                
            json_data = self.to_json(parsed_data)
            self._last_json = json_data
            self._zmq_pub.send_multipart([b"HWSTATUS", json_data.encode('utf-8')])
            
            print(f"[HWSTATUSA] {message}")
            return True
            
        except Exception as e:
            print(f"[HWSTATUSA] Parse error: {e} | Msg: {message}")
            return False
