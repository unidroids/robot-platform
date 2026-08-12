# uniheadinga_handler.py
from __future__ import annotations

import json
import zmq
from typing import Optional

__all__ = ["UniHeadinAHandler"]

class UniHeadinAHandler:
    """
    Handler příjmu UNIHEADINGA zpráv a jejich přeposílání.
    """

    def __init__(self, zmq_pub: zmq.Socket) -> None:
        self._lastest: Optional[bytes] = None  
        self._zmq_pub = zmq_pub

    # --- veřejné API ---

    def _parse_message(self, message: str) -> dict:
        if ";" not in message:
            return {"error": "no_header", "raw": message}
            
        header, data = message.split(";", 1)
        if "*" in data:
            data = data.split("*")[0]
            
        header_parts = header.split(",")
        data_parts = data.split(",")
        
        if len(header_parts) < 10 or len(data_parts) < 17:
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
            "reserved_hdr": header_parts[7],
            "leap_sec": int(header_parts[8]) if header_parts[8] else 0,
            "output_delay": int(header_parts[9]) if header_parts[9] else 0,
            
            "sol_status": data_parts[0],
            "pos_type": data_parts[1],
            "length": float(data_parts[2]) if data_parts[2] else 0.0,
            "heading": float(data_parts[3]) if data_parts[3] else 0.0,
            "pitch": float(data_parts[4]) if data_parts[4] else 0.0,
            "reserved": float(data_parts[5]) if data_parts[5] else 0.0,
            "hdg_std": float(data_parts[6]) if data_parts[6] else 0.0,
            "pitch_std": float(data_parts[7]) if data_parts[7] else 0.0,
            "stn_id": data_parts[8].strip('"'),
            "svs": int(data_parts[9]) if data_parts[9] else 0,
            "soln_svs": int(data_parts[10]) if data_parts[10] else 0,
            "obs": int(data_parts[11]) if data_parts[11] else 0,
            "multi": int(data_parts[12]) if data_parts[12] else 0,
            "sol_source": int(data_parts[13]) if data_parts[13] else 0,
            "ext_sol_stat": data_parts[14],
            "galileo_bds3_mask": data_parts[15],
            "gps_glonass_bds2_mask": data_parts[16]
        }

    def handle(self, message_bytes: bytes):
        """
        Zpracuje jednu syrovou zprávu.
        """
        try:
            message = message_bytes.decode('ascii', errors='ignore').strip()
        except Exception:
            return

        if not message.startswith('#UNIHEADINGA'):
            raise Exception(f"[UNIHEADINGA] Not a #UNIHEADINGA message: {message}")
            
        parsed_data = self._parse_message(message)
        if "error" in parsed_data:
            raise Exception(f"[UNIHEADINGA] Error: {parsed_data['error']} | Msg: {message}")
            
        json_data = json.dumps(parsed_data)
        
        # 1) uložit na _lastest
        self._lastest = json_data.encode('utf-8')

        try:
            self._zmq_pub.send_string(f"UNIHEADING {json_data}")
        except Exception as e:
            pass

    def get_lastest(self) -> Optional[bytes]:
        """Vrátí naposledy přijatá data ve formátu JSON."""
        return self._lastest

    def __del__(self):
        pass


# --- jednoduchý lokální test bez sítě ---
if __name__ == "__main__":
    h = UniHeadinAHandler(None)  
    msg = b'#UNIHEADINGA,92,GPS,FINE,2392,519230000,0,0,18,8;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*f25b9a39\r\n'
    h.handle(msg)
    last = h.get_lastest()
    print("Lastest:", last)
    if last:
        print("Serialized length:", len(last), "bytes")
    msg = b'#UNIHEADINGA,92,GPS,FINE,2392,519238000,0,0,18,8;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*e914be33\r\n'
    h.handle(msg)
    last = h.get_lastest()
    print("Lastest:", last)
    if last:
        print("Serialized length:", len(last), "bytes")
