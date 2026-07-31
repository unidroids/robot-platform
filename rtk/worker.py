import base64
import socket
import threading
import time
import os
import zmq
import json
import traceback
from datetime import datetime
from proto.pointperfect_ntrip_client import NtripClient

class RtkWorker:
    def __init__(self, ntrip_user, ntrip_pass, ntrip_host, ntrip_port, ntrip_mount, tls):
        self.ntrip_user = ntrip_user
        self.ntrip_pass = ntrip_pass
        self.ntrip_host = ntrip_host
        self.ntrip_port = ntrip_port
        self.ntrip_mount = ntrip_mount
        self.tls = tls

        self.running = False
        self._thread = None
        self._stop_event = threading.Event()
        
        self.last_gga = ""
        self.last_rtcm = ""
        self.msg_count_gga = 0
        self.msg_count_rtcm = 0
        
        self.log_file = None
        
        # tcp client port 5001
        self.gps_tcp_host = "127.0.0.1"
        self.gps_tcp_port = 5001
        self.gps_tcp_socket = None
        
        self.ntrip = None

    def start(self):
        if self.running:
            return
        self._stop_event.clear()
        
        # Vytvoření adresáře a logovacího souboru
        now = datetime.now()
        date_dir = f"/data/robot/rtk/{now.strftime('%Y-%m-%d')}"
        os.makedirs(date_dir, exist_ok=True)
        log_path = os.path.join(date_dir, f"rtk-{now.strftime('%H-%M-%S')}.dat")
        self.log_file = open(log_path, "a")
        
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("[RTK] Worker nastartován, čekám na GGA zprávy...")

    def stop(self):
        if not self.running:
            return
        self._stop_event.set()
        self.running = False
        if self.ntrip:
            self.ntrip.stop_stream()
            self.ntrip = None
            
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            
        if self.gps_tcp_socket:
            try:
                self.gps_tcp_socket.close()
            except:
                pass
            self.gps_tcp_socket = None
            
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            
        print("[RTK] Worker zastaven, log uzavřen.")

    def get_status(self):
        status = "RUNNING" if self.running else "IDLE"
        return f"{status} GGA_COUNT:{self.msg_count_gga} RTCM_COUNT:{self.msg_count_rtcm} LAST_GGA:{self.last_gga} LAST_RTCM_B64:{self.last_rtcm}"

    def _run(self):
        ctx = zmq.Context()
        zmq_sub = ctx.socket(zmq.SUB)
        zmq_sub.connect("ipc:///tmp/robot-gps")
        zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "GGA/")
        zmq_sub.setsockopt(zmq.RCVTIMEO, 1000)

        self.ntrip = NtripClient(
            host=self.ntrip_host,
            port=self.ntrip_port,
            user=self.ntrip_user,
            password=self.ntrip_pass,
            tls=self.tls
        )
        ntrip_started = False
        
        while not self._stop_event.is_set():
            try:
                msg = zmq_sub.recv_string()
                # Očekávaný formát: GGA/{"lat": ..., "lon": ..., "fix": ..., "sats": ..., "raw": "$GPGGA,..."}
                prefix, json_str = msg.split("/", 1)
                data = json.loads(json_str)
                raw_gga = data.get("raw", "").strip()
                fix = data.get("fix", 0)
                sats = data.get("sats", 0)
                
                self.msg_count_gga += 1
                self.last_gga = raw_gga
                
                # Zápis GGA do logu
                if self.log_file:
                    self.log_file.write(f"{time.monotonic():.3f} GGA {raw_gga}\n")
                    self.log_file.flush()
                
                print(f"[RTK] Přijata GGA zpráva (fix: {fix}, sats: {sats}): {raw_gga}")
                
                if fix > 0 and sats >= 4:
                    if not ntrip_started:
                        print(f"[RTK] Nalezen validní GPS fix, startuji NTRIP stream na {self.ntrip_host}:{self.ntrip_port} (mount: {self.ntrip_mount})")
                        self.ntrip.start_stream(self.ntrip_mount, self._on_rtcm_data)
                        ntrip_started = True
                        time.sleep(1.0) # počkáme na inicializaci streamu
                    
                    if ntrip_started:
                        gga_to_send = raw_gga + "\r\n"
                        self.ntrip.send_gga(gga_to_send)
                        print("[RTK] Odeslána GGA pozice na PointPerfect server.")

            except zmq.error.Again:
                pass
            except Exception as e:
                print(f"[RTK] Chyba ve smyčce workeru: {e}")
                traceback.print_exc()
                
        # Úklid ZMQ
        zmq_sub.close()
        ctx.term()

    def _on_rtcm_data(self, data: bytes):
        if not self.running:
            return
            
        self.msg_count_rtcm += 1
        b64_data = base64.b64encode(data).decode('ascii')
        self.last_rtcm = b64_data[:20] + "..." 
        
        # Zápis RTCM do logu
        if self.log_file:
            self.log_file.write(f"{time.monotonic():.3f} RTCM {b64_data}\n")
            self.log_file.flush()
            
        print(f"[RTK] Přijato {len(data)} bytů RTCM dat od PointPerfect.")
        self._send_to_gps(data)

    def _send_to_gps(self, data: bytes):
        try:
            if self.gps_tcp_socket is None:
                self.gps_tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.gps_tcp_socket.settimeout(1.0)
                self.gps_tcp_socket.connect((self.gps_tcp_host, self.gps_tcp_port))
                print(f"[RTK] Připojeno k TCP serveru GPS služby na {self.gps_tcp_host}:{self.gps_tcp_port}")
            
            self.gps_tcp_socket.sendall(data)
            print(f"[RTK] Odesláno {len(data)} bytů RTCM dat do služby GPS na port {self.gps_tcp_port}.")
        except socket.timeout:
            print(f"[RTK] Chyba při odesílání na {self.gps_tcp_port} (timeout).")
        except ConnectionRefusedError:
            print(f"[RTK] Služba GPS (port {self.gps_tcp_port}) pravděpodobně neběží nebo je spojení odmítnuto.")
            if self.gps_tcp_socket:
                try:
                    self.gps_tcp_socket.close()
                except:
                    pass
                self.gps_tcp_socket = None
        except Exception as e:
            print(f"[RTK] Chyba při předávání RTCM dat: {e}")
            traceback.print_exc()
            if self.gps_tcp_socket:
                try:
                    self.gps_tcp_socket.close()
                except:
                    pass
                self.gps_tcp_socket = None
