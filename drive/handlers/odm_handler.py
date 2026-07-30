# odm_handler.py
from __future__ import annotations

from typing import Optional
import zmq
import json

__all__ = ["OdmHandler"]


class OdmHandler:
    """
    Handler příjmu ODM zpráv a jejich přeposílání přes ZMQ.

    Očekávaná testovací věta (NMEA-like):
        b"$ODM<ts_mono>,<leftSteps>,<rightSteps>,<leftSpeed>,<rightSpeed>*CS\r\n"

    - Parsuje zprávu a odesílá rychlost levého a pravého kola jako JSON přes ZMQ.
    """

    def __init__(self) -> None:
        self._latest: Optional[bytes] = None

        # ZMQ publisher pro odometrii
        self._zmq_context = zmq.Context.instance()
        self._zmq_pub = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub.bind("ipc:///tmp/robot-odometry")

    # --- veřejné API ---

    def handle(self, message_bytes: bytes):
        """
        Zpracuje jednu syrovou zprávu přes sériovou linku.
        """
        # odstraníme $ODM a *CS\r\n
        send_message = message_bytes[4:-5] 
        self._latest = send_message

        # poslat na zmq jako json
        self._send_zmq(send_message)

    def get_latest(self) -> Optional[bytes]:
        """Vrátí naposledy přijatá ODM data (nebo None)."""
        return self._latest

    def _send_zmq(self, send_message: bytes):
        """Vyparsuje ts, left, right a pošle jako JSON na ZMQ."""
        try:
            decoded = send_message.decode('ascii', errors='ignore')
            parts = decoded.split(',')
            if len(parts) >= 5:
                ts_mono = parts[0]
                left_steps = parts[1]
                right_steps = parts[2]
                left_speed = parts[3]
                right_speed = parts[4]
                
                msg_data = {
                    "ts": int(ts_mono),
                    "left_steps": int(left_steps),
                    "right_steps": int(right_steps),
                    "left": int(left_speed),
                    "right": int(right_speed)
                }
                
                self._zmq_pub.send_string(f"odometry/{json.dumps(msg_data)}")
        except Exception as e:
            print(f"[OdmHandler] Chyba při odesílání na ZMQ: {e}")

    def __del__(self):
        if hasattr(self, '_zmq_pub') and self._zmq_pub is not None:
            self._zmq_pub.close()


# --- jednoduchý lokální test ---
if __name__ == "__main__":
    h = OdmHandler()  
    msg = b"$ODM123456,-10,456789,120,-130*CS\r\n"
    h.handle(msg)
    last = h.get_latest()
    print("Latest:", last)
    
    msg = b"$ODM123456,-10,456789,120,130*CS\r\n"
    h.handle(msg)
    last = h.get_latest()
    print("Latest:", last)
