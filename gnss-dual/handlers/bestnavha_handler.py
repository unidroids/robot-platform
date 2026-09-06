import time
import zmq
import json

class BestnavhaHandler:
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._last_log_time = 0
        self._last_print_time = 0
        self._last_json = "{}"

    def get_last_json(self) -> str:
        return self._last_json

    def _parse_message(self, message: str) -> dict:
        if ";" not in message:
            return {"error": "no_header", "raw": message}
            
        header, data = message.split(";", 1)
        if "*" in data:
            data = data.split("*")[0]
            
        header_parts = header.split(",")
        data_parts = data.split(",")
        
        if len(header_parts) < 10 or len(data_parts) < 30:
            return {"error": "incomplete_data", "raw": message}
            
        msg_name = header_parts[0]
        sync = msg_name[0] if msg_name.startswith("#") else ""
        if sync == "#":
            msg_name = msg_name[1:]
            
        return {
            "sync": sync,
            "message": msg_name,
            "antenna": "secondary",
            "cpu_idle": int(header_parts[1]) if header_parts[1] else 0,
            "time_ref": header_parts[2],
            "time_status": header_parts[3],
            "wn": int(header_parts[4]) if header_parts[4] else 0,
            "ms": int(header_parts[5]) if header_parts[5] else 0,
            "version": header_parts[6],
            "reserved": header_parts[7],
            "leap_sec": int(header_parts[8]) if header_parts[8] else 0,
            "output_delay": int(header_parts[9]) if header_parts[9] else 0,
            
            "p_sol_status": data_parts[0],
            "pos_type": data_parts[1],
            "lat": float(data_parts[2]) if data_parts[2] else 0.0,
            "lon": float(data_parts[3]) if data_parts[3] else 0.0,
            "hgt": float(data_parts[4]) if data_parts[4] else 0.0,
            "undulation": float(data_parts[5]) if data_parts[5] else 0.0,
            "datum_id": data_parts[6],
            "lat_std": float(data_parts[7]) if data_parts[7] else 0.0,
            "lon_std": float(data_parts[8]) if data_parts[8] else 0.0,
            "hgt_std": float(data_parts[9]) if data_parts[9] else 0.0,
            "stn_id": data_parts[10].strip('"'),
            "diff_age": float(data_parts[11]) if data_parts[11] else 0.0,
            "sol_age": float(data_parts[12]) if data_parts[12] else 0.0,
            "svs": int(data_parts[13]) if data_parts[13] else 0,
            "soln_svs": int(data_parts[14]) if data_parts[14] else 0,
            "ext_sol_stat": data_parts[18],
            "v_sol_status": data_parts[21],
            "vel_type": data_parts[22],
            "latency": float(data_parts[23]) if data_parts[23] else 0.0,
            "vel_diff_age": float(data_parts[24]) if data_parts[24] else 0.0,
            "hor_spd": float(data_parts[25]) if data_parts[25] else 0.0,
            "trk_gnd": float(data_parts[26]) if data_parts[26] else 0.0,
            "vert_spd": float(data_parts[27]) if data_parts[27] else 0.0,
            "ver_spd_std": float(data_parts[28]) if data_parts[28] else 0.0,
            "hor_spd_std": float(data_parts[29]) if data_parts[29] else 0.0,
        }

    def to_json(self, parsed_data: dict) -> str:
        return json.dumps(parsed_data)

    def handle(self, message: str) -> bool:
        try:
            message = message.strip()
            if not message.startswith("#BESTNAVHA"):
                print(f"[BESTNAVHA] Not a #BESTNAVHA message: {message}")
                return False

            parsed_data = self._parse_message(message)
            if "error" in parsed_data:
                print(f"[BESTNAVHA] Error: {parsed_data['error']} | Msg: {message}")
                return False
                
            json_data = self.to_json(parsed_data)
            self._last_json = json_data
            if self._zmq_pub:
                try:
                    self._zmq_pub.send_multipart([b"BESTNAVH", json_data.encode('utf-8')])
                except Exception as e:
                    print(f"[BESTNAVHA] ZMQ send error: {e}")
            
            current_time = time.time()
            if current_time - self._last_log_time >= 1.0:
                print(f"[BESTNAVHA] {message}")
                self._last_log_time = current_time
            return True
            
        except Exception as e:
            print(f"[BESTNAVHA] Parse error: {e} | Msg: {message}")
            return False

if __name__ == "__main__":
    h = BestnavhaHandler(None)
    msg = '#BESTNAVHA,72,GPS,FINE,2435,58412500,0,0,18,21;SOL_COMPUTED,SINGLE,50.06155252318,14.59973008972,251.4485,44.6057,WGS84,1.3945,1.0745,2.1126,"0",0.000,1.000,34,28,28,0,4,12,11,51,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0042,134.641067,-0.0061,0.0182,0.0137*a521b22d'
    success = h.handle(msg)
    print("Success:", success)
    last = h.get_last_json()
    print("Last JSON:", last)
