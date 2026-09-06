import time
import zmq
import json

class MsposaHandler:
    """
    Handler pro zprávy #MSPOSA (Best Position of Dual Antennas - Master & Slave).
    Obsahuje současně vypočtenou pozici primární (Master) i sekundární (Slave) antény.
    """
    def __init__(self, zmq_pub: zmq.Socket):
        self._zmq_pub = zmq_pub
        self._last_log_time = 0
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
        
        if len(header_parts) < 10 or len(data_parts) < 24:
            return {"error": "incomplete_data", "raw": message}
            
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
            "reserved": header_parts[7],
            "leap_sec": int(header_parts[8]) if header_parts[8] else 0,
            "output_delay": int(header_parts[9]) if header_parts[9] else 0,
            
            # Master antenna (primární)
            "master_sol_status": data_parts[0],
            "master_pos_type": data_parts[1],
            "master_lat": float(data_parts[2]) if data_parts[2] else 0.0,
            "master_lon": float(data_parts[3]) if data_parts[3] else 0.0,
            "master_hgt": float(data_parts[4]) if data_parts[4] else 0.0,
            "master_lat_std": float(data_parts[5]) if data_parts[5] else 0.0,
            "master_lon_std": float(data_parts[6]) if data_parts[6] else 0.0,
            "master_hgt_std": float(data_parts[7]) if data_parts[7] else 0.0,
            "master_svs": int(data_parts[8]) if data_parts[8] else 0,
            "master_soln_svs": int(data_parts[9]) if data_parts[9] else 0,
            
            # Slave antenna (sekundární)
            "slave_sol_status": data_parts[11],
            "slave_pos_type": data_parts[12],
            "slave_lat": float(data_parts[13]) if data_parts[13] else 0.0,
            "slave_lon": float(data_parts[14]) if data_parts[14] else 0.0,
            "slave_hgt": float(data_parts[15]) if data_parts[15] else 0.0,
            "slave_lat_std": float(data_parts[16]) if data_parts[16] else 0.0,
            "slave_lon_std": float(data_parts[17]) if data_parts[17] else 0.0,
            "slave_hgt_std": float(data_parts[18]) if data_parts[18] else 0.0,
            "slave_svs": int(data_parts[19]) if data_parts[19] else 0,
            "slave_soln_svs": int(data_parts[20]) if data_parts[20] else 0,
            
            # Stanice a diferenciální věk
            "stn_id": data_parts[22].strip('"'),
            "diff_age": float(data_parts[23]) if data_parts[23] else 0.0,
        }

    def to_json(self, parsed_data: dict) -> str:
        return json.dumps(parsed_data)

    def handle(self, message: str) -> bool:
        try:
            message = message.strip()
            if not message.startswith("#MSPOSA"):
                print(f"[MSPOSA] Not a #MSPOSA message: {message}")
                return False

            parsed_data = self._parse_message(message)
            if "error" in parsed_data:
                print(f"[MSPOSA] Error: {parsed_data['error']} | Msg: {message}")
                return False
                
            json_data = self.to_json(parsed_data)
            self._last_json = json_data
            if self._zmq_pub:
                try:
                    self._zmq_pub.send_multipart([b"MSPOS", json_data.encode('utf-8')])
                except Exception as e:
                    print(f"[MSPOSA] ZMQ send error: {e}")
            
            current_time = time.time()
            if current_time - self._last_log_time >= 1.0:
                print(f"[MSPOSA] {message}")
                self._last_log_time = current_time
            return True
            
        except Exception as e:
            print(f"[MSPOSA] Parse error: {e} | Msg: {message}")
            return False

if __name__ == "__main__":
    h = MsposaHandler(None)
    msg = '#MSPOSA,71,GPS,FINE,2435,59574500,0,0,18,24;SOL_COMPUTED,SINGLE,50.06156261227,14.59972793413,250.2556,1.3835,1.0795,2.2004,35,28,,SOL_COMPUTED,SINGLE,50.06156453982,14.59972702755,255.3900,1.3205,1.0471,2.1158,35,28,,"0",0.000*346d341a'
    success = h.handle(msg)
    print("Success:", success)
    last = h.get_last_json()
    print("Last JSON:", last)
