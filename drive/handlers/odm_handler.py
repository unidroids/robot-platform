# odm_handler.py
from __future__ import annotations

import socket
import traceback
from typing import Optional
import zmq
import json
import os

__all__ = ["OdmHandler"]


class OdmHandler:
    """
    Handler příjmu ODM zpráv a jejich přeposílání binárně na stream.

    Očekávaná testovací věta (NMEA-like):
        b"$ODM<ts_mono>,<gyroZ_adc>,<accumAngle_adc>,<leftSpeed>,<rightSpeed>*CS\\r\\n"

    - Parsuje pomocí parse_message(msg) -> (code, fields)
    - Po vytvoření OdmData ji uloží do self._lastest
    - Přes otevřený TCP socket (localhost:9009) binárně posílá OdmData.to_bytes()
    - Po prvním připojení odešle jednorázově "PUSH_ODM_DATA_STREAM\\r\\n"
    - Socket zůstává otevřený; při chybě se pokusí o znovupřipojení při další zprávě
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9009,
        timeout: float = 2.0,
        autoconnect: bool = True,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout

        self._sock: Optional[socket.socket] = None
        self._stream_opened = False

        self._lastest: Optional[bytes] = None  # záměrně název podle zadání

        # 0) ZMQ publisher pro odometrii
        self._zmq_context = zmq.Context.instance()
        self._zmq_pub = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub.bind("ipc:///tmp/robot-odometry")
        #try:
        #    os.chmod("/tmp/robot-odometry", 0o777)
        #except Exception as e:
        #    print(f"⚠️ [OdmHandler] Nelze nastavit oprávnění na ZMQ socket: {e}")

        if autoconnect:
            self._ensure_socket()

    # --- veřejné API ---

    def handle(self, message_bytes: bytes, wait: bool = False):
        """
        Zpracuje jednu syrovou zprávu přes sériovou linku.
        """

        send_message = message_bytes[4:-5] # odstraníme $ODM a *CS\r\n

        # 1) uložit na _lastest
        self._lastest = send_message

        # 2) poslat na fusion
        self._send_odm(send_message, wait)

        # 3) poslat na zmq (pilot-vision atd.)
        self._send_zmq(send_message)

    def get_lastest(self) -> Optional[bytes]:
        """Vrátí naposledy přijatá ODM data (nebo None)."""
        return self._lastest

    def _send_zmq(self, send_message: bytes):
        """Vyparsuje ts, left, right a pošle jako JSON na ZMQ s topicem 'speed'."""
        try:
            decoded = send_message.decode('ascii', errors='ignore')
            parts = decoded.split(',')
            if len(parts) >= 5:
                ts_mono = parts[0]
                left_speed = parts[3]
                right_speed = parts[4]
                
                msg_data = {
                    "ts": int(ts_mono),
                    "left": int(left_speed),
                    "right": int(right_speed)
                }
                # Pošleme s prefixem 'speed/' pro snadné filtrování odběratelem
                self._zmq_pub.send_string(f"speed/{json.dumps(msg_data)}")
        except Exception as e:
            print(f"[OdmHandler] Chyba při odesílání na ZMQ: {e}")

    # --- interní pomocné metody ---

    def _ensure_socket(self) -> None:
        """připojení k serveru"""
        if self._sock is not None:
            return

        try:
            # open socket with non waiting for sedning
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM )
            s.settimeout(self._timeout)
            s.connect((self._host, self._port))
            #s.settimeout(None)  # blokující režim po spojení
            s.setblocking(True)  # <<< NEBLOKUJÍCÍ režim
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # <<< nižší latence malých zpráv
            self._sock = s
            self._stream_opened = True  # bude otevřeno po handshake
            print(f"[OdmHandler] Connected to {self._host}:{self._port}")
        except Exception as e:
            # necháme zavřené; zkusíme znovu při další zprávě
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            self._stream_opened = False
            print(f"[OdmHandler] Unable to connect to {self._host}:{self._port}: {e!r}")
            

    def _send_odm(self, odm_message: bytes, wait:bool = False) -> None:
        """Pošle binární ODM data. Při chybě socket zavře (reconnect proběhne při další zprávě)."""
        self._ensure_socket()
        if self._sock is None or not self._stream_opened:
            return
        try:
            self._sock.sendall(b'DRIVE\n' + odm_message + b'\n')
            if wait:
                result = self._sock.recv(128)
                print(f"[OdmHandler] recieved: {result}")
        except Exception as e:
            print(f"[OdmHandler] sendall failed: {e!r}")
            self._close_socket()

    def _close_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._stream_opened = False

    def __del__(self):
        # best-effort úklid
        self._close_socket()
        if hasattr(self, '_zmq_pub') and self._zmq_pub is not None:
            self._zmq_pub.close()


# --- jednoduchý lokální test bez sítě ---
if __name__ == "__main__":
    h = OdmHandler()  
    msg = b"$ODM123456,-10,456789,120,-130*CS\r\n"
    h.handle(msg, True)
    last = h.get_lastest()
    print("Lastest:", last)
    if last:
        print("Serialized length:", len(last), "bytes")
    msg = b"$ODM123456,-10,456789,120,130*CS\r\n"
    h.handle(msg)
    last = h.get_lastest()
    print("Lastest:", last)
    if last:
        print("Serialized length:", len(last), "bytes")
