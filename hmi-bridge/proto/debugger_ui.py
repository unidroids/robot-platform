import socket
import threading
import time
import subprocess
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

def watchdog_server(log_callback):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Povolit znovupoužití adresy pro rychlé restarty
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(('127.0.0.1', 9000))
        server.listen(1)
        log_callback("Watchdog (9000): Listening on 127.0.0.1:9000...")
    except Exception as e:
        log_callback(f"Watchdog (9000) Bind Error: {e}")
        return

    while True:
        try:
            conn, addr = server.accept()
            log_callback(f"Watchdog (9000): Android connected from {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                decoded = data.decode('utf-8')
                log_callback(f"Watchdog (9000) RX: {decoded.strip()}")
                if "PING" in decoded:
                    conn.sendall(b"PONG ADB READY\n")
                    log_callback("Watchdog (9000) TX: PONG ADB READY")
            conn.close()
            log_callback("Watchdog (9000): Android disconnected")
        except Exception as e:
            log_callback(f"Watchdog (9000) Error: {e}")
            time.sleep(1)

def zmq_server(log_callback):
    try:
        import zmq
    except ImportError:
        log_callback("ZMQ (8001) ERROR: pyzmq module not found! Please run 'pip install pyzmq'")
        return
        
    try:
        context = zmq.Context()
        sock = context.socket(zmq.SUB)
        sock.bind("tcp://127.0.0.1:8001")
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        
        log_callback("ZMQ (8001): Bound SUB socket to tcp://127.0.0.1:8001 (Topic: All/Any)")
        while True:
            frames = sock.recv_multipart()
            try:
                decoded_frames = [f.decode('utf-8') for f in frames]
                log_callback(f"ZMQ (8001) RX: {decoded_frames}")
            except Exception as decode_err:
                log_callback(f"ZMQ (8001) RX (Raw): {frames} | Decode error: {decode_err}")
    except Exception as e:
        log_callback(f"ZMQ (8001) Fatal Error: {e}")

def zmq_terminal_server(log_callback):
    try:
        import zmq
    except ImportError:
        return
        
    try:
        context = zmq.Context()
        sock = context.socket(zmq.SUB)
        sock.bind("tcp://127.0.0.1:8002")
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        
        log_callback("ZMQ (8002): Bound SUB socket to tcp://127.0.0.1:8002 (Topic: All/Any)")
        while True:
            frames = sock.recv_multipart()
            try:
                decoded_frames = [f.decode('utf-8') for f in frames]
                log_callback(f"ZMQ (8002) RX: {decoded_frames}")
            except Exception as decode_err:
                log_callback(f"ZMQ (8002) RX (Raw): {frames} | Decode error: {decode_err}")
    except Exception as e:
        log_callback(f"ZMQ (8002) Fatal Error: {e}")


class QRClient:
    def __init__(self, log_callback):
        self.sock = None
        self.log_callback = log_callback

    def connect(self):
        if self.sock:
            self.log_callback("QR Client (9001): Already connected or socket exists.")
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0) # timeout pro připojení
            self.log_callback("QR Client (9001): Connecting to 127.0.0.1:9001...")
            self.sock.connect(('127.0.0.1', 9001))
            self.sock.settimeout(None) # zrušit timeout pro čtení
            self.log_callback("QR Client (9001): Connected!")
            
            # Start background thread to read responses
            threading.Thread(target=self._receive_loop, daemon=True).start()
        except Exception as e:
            self.log_callback(f"QR Client (9001) Connect Error: {e}")
            self.sock = None

    def disconnect(self, log_msg="QR Client (9001): Disconnected manually."):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
            if log_msg:
                self.log_callback(log_msg)

    def send_cmd(self, cmd):
        if self.sock:
            try:
                self.sock.sendall(f"{cmd}\n".encode('utf-8'))
                self.log_callback(f"QR Client (9001) TX: {cmd}")
            except Exception as e:
                self.log_callback(f"QR Client (9001) TX Error: {e}")
                self.disconnect(log_msg=None)
        else:
            self.log_callback("QR Client (9001): Not connected! Please connect first.")

    def _receive_loop(self):
        while self.sock:
            try:
                data = self.sock.recv(1024)
                if not data:
                    self.log_callback("QR Client (9001): Connection closed by server")
                    self.disconnect(log_msg=None)
                    break
                
                messages = data.decode('utf-8').split('\n')
                for msg in messages:
                    if msg.strip():
                        self.log_callback(f"QR Client (9001) RX: {msg.strip()}")
            except Exception as e:
                if self.sock: # Pokud už není sock = None (z disconnectu)
                    self.log_callback(f"QR Client (9001) RX Error: {e}")
                    self.disconnect(log_msg=None)
                break


class TerminalClient:
    def __init__(self, log_callback):
        self.sock = None
        self.log_callback = log_callback

    def connect(self):
        if self.sock:
            self.log_callback("Terminal Client (9002): Already connected or socket exists.")
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3.0) # timeout pro připojení
            self.log_callback("Terminal Client (9002): Connecting to 127.0.0.1:9002...")
            self.sock.connect(('127.0.0.1', 9002))
            self.sock.settimeout(None) # zrušit timeout pro čtení
            self.log_callback("Terminal Client (9002): Connected!")
            
            # Start background thread to read responses
            threading.Thread(target=self._receive_loop, daemon=True).start()
        except Exception as e:
            self.log_callback(f"Terminal Client (9002) Connect Error: {e}")
            self.sock = None

    def disconnect(self, log_msg="Terminal Client (9002): Disconnected manually."):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
            if log_msg:
                self.log_callback(log_msg)

    def send_cmd(self, cmd):
        if self.sock:
            try:
                self.sock.sendall(f"{cmd}\n".encode('utf-8'))
                self.log_callback(f"Terminal Client (9002) TX: {cmd}")
            except Exception as e:
                self.log_callback(f"Terminal Client (9002) TX Error: {e}")
                self.disconnect(log_msg=None)
        else:
            self.log_callback("Terminal Client (9002): Not connected! Please connect first.")

    def _receive_loop(self):
        while self.sock:
            try:
                data = self.sock.recv(1024)
                if not data:
                    self.log_callback("Terminal Client (9002): Connection closed by server")
                    self.disconnect(log_msg=None)
                    break
                
                messages = data.decode('utf-8').split('\n')
                for msg in messages:
                    if msg.strip():
                        self.log_callback(f"Terminal Client (9002) RX: {msg.strip()}")
            except Exception as e:
                if self.sock:
                    self.log_callback(f"Terminal Client (9002) RX Error: {e}")
                    self.disconnect(log_msg=None)
                break


class DebuggerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotour HMI Debugger")
        self.root.geometry("1000x800")
        
        # 1. ADB Bridge
        adb_frame = tk.LabelFrame(root, text="1. ADB Bridge Setup")
        adb_frame.pack(fill="x", padx=10, pady=5)
        
        path_frame = tk.Frame(adb_frame)
        path_frame.pack(fill="x", padx=10, pady=(10, 5))
        tk.Label(path_frame, text="Cesta k ADB:").pack(side="left")
        
        import os
        default_adb = r"C:\Users\jarda\AppData\Local\Android\Sdk\platform-tools\adb.exe"
            
        self.adb_path_var = tk.StringVar(value=default_adb)
        tk.Entry(path_frame, textvariable=self.adb_path_var, width=50).pack(side="left", padx=5)

        btn_bridge_frame = tk.Frame(adb_frame)
        btn_bridge_frame.pack(fill="x", padx=10, pady=(0, 10))
        tk.Button(btn_bridge_frame, text="Create Bridge (reverse/forward)", command=self.setup_bridge, bg="#e0f7fa").pack(side="left")
        tk.Button(btn_bridge_frame, text="Sync Sounds", command=self.sync_sounds, bg="#fff9c4").pack(side="left", padx=5)
        tk.Label(btn_bridge_frame, text="(Vytvoří USB tunely a nahraje zvuky)").pack(side="left", padx=5)

        # 2. QR Command Client
        qr_frame = tk.LabelFrame(root, text="2. QR Command Client (TCP 9001)")
        qr_frame.pack(fill="x", padx=10, pady=5)
        
        btn_frame = tk.Frame(qr_frame)
        btn_frame.pack(fill="x", padx=5, pady=5)
        
        tk.Button(btn_frame, text="Connect", command=self.connect_qr, bg="#e8f5e9").pack(side="left", padx=5)
        tk.Button(btn_frame, text="Disconnect", command=self.disconnect_qr, bg="#ffebee").pack(side="left", padx=5)
        tk.Label(btn_frame, text=" | ").pack(side="left")
        tk.Button(btn_frame, text="Send PING", command=lambda: self.qr_client.send_cmd("PING")).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Send START", command=lambda: self.qr_client.send_cmd("START")).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Send STOP", command=lambda: self.qr_client.send_cmd("STOP")).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Send STATUS", command=lambda: self.qr_client.send_cmd("STATUS")).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Send QRCODE", command=lambda: self.qr_client.send_cmd("QRCODE")).pack(side="left", padx=5)
        
        # 3. Terminal Command Client
        term_frame = tk.LabelFrame(root, text="3. Terminal Command Client (TCP 9002)")
        term_frame.pack(fill="x", padx=10, pady=5)
        
        term_btn_frame = tk.Frame(term_frame)
        term_btn_frame.pack(fill="x", padx=5, pady=5)
        
        tk.Button(term_btn_frame, text="Connect", command=self.connect_term, bg="#e8f5e9").pack(side="left", padx=5)
        tk.Button(term_btn_frame, text="Disconnect", command=self.disconnect_term, bg="#ffebee").pack(side="left", padx=5)
        tk.Label(term_btn_frame, text=" | ").pack(side="left")
        tk.Button(term_btn_frame, text="Send PING", command=lambda: self.term_client.send_cmd("PING")).pack(side="left", padx=5)
        tk.Button(term_btn_frame, text="Send SOUND barking", command=lambda: self.term_client.send_cmd("SOUND barking")).pack(side="left", padx=5)
        tk.Button(term_btn_frame, text="Send SOUND game-over", command=lambda: self.term_client.send_cmd("SOUND game-over")).pack(side="left", padx=5)
        tk.Button(term_btn_frame, text="Send SOUND notification", command=lambda: self.term_client.send_cmd("SOUND notification")).pack(side="left", padx=5)
        tk.Button(term_btn_frame, text="Send BLINK", command=lambda: self.term_client.send_cmd("BLINK #FF0000 3 5000")).pack(side="left", padx=5)

        sample_msg_A = '{"header":"Varování","text":"Překážka","buttons":[{"id":"btn_1","text":"OK"}]}'
        tk.Button(term_btn_frame, text="Send MESSAGE", command=lambda: self.term_client.send_cmd(f"MESSAGE {sample_msg_A}")).pack(side="left", padx=5)

        sample_msg_B = '{"header":"Robotour","text":"Jdeme na to!","buttons":[{"id":"scan_qrcode","text":"Scan QR Code"}]}'
        tk.Button(term_btn_frame, text="Scan QR Code", command=lambda: self.term_client.send_cmd(f"MESSAGE {sample_msg_B}")).pack(side="left", padx=5)

        sample_msg_C = '{"header":"Robotour - Potvrzení destinace","text":"Cílové souřadnice jsou geo:12.233,45.345. Vzdušná vzdálenost do cíle je 300m.","buttons":[{"id":"repeat_qrscan","text":"Opakovat QR Scan"}, {"id":"destination_ok","text":"Ano, jedeme"},{"id":"cancel","text":"Ne, zrušit"}]}'
        tk.Button(term_btn_frame, text="Potvrdit destinaci", command=lambda: self.term_client.send_cmd(f"MESSAGE {sample_msg_C}")).pack(side="left", padx=5)

        # Rozdělení na dva logy
        logs_container = tk.Frame(root)
        logs_container.pack(fill="both", expand=True, padx=10, pady=5)
        
        # QR & Terminal Command Log
        qr_log_frame = tk.LabelFrame(logs_container, text="Clients Log (9001, 9002)")
        qr_log_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.qr_log = ScrolledText(qr_log_frame, height=15, bg="#f5f5f5")
        self.qr_log.pack(fill="both", expand=True, padx=5, pady=5)

        # System Log
        sys_log_frame = tk.LabelFrame(logs_container, text="System Log (9000 Watchdog / 8001, 8002 ZMQ / ADB)")
        sys_log_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        self.sys_log = ScrolledText(sys_log_frame, height=15, bg="#fafafa")
        self.sys_log.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.qr_client = QRClient(self.log_qr)
        self.term_client = TerminalClient(self.log_qr)
        
        # Start background servers
        threading.Thread(target=watchdog_server, args=(self.log_sys,), daemon=True).start()
        threading.Thread(target=zmq_server, args=(self.log_sys,), daemon=True).start()
        threading.Thread(target=zmq_terminal_server, args=(self.log_sys,), daemon=True).start()

    def log_sys(self, msg):
        self.root.after(0, self._append_log, self.sys_log, msg)
        
    def log_qr(self, msg):
        self.root.after(0, self._append_log, self.qr_log, msg)
        
    def _append_log(self, text_widget, msg):
        text_widget.insert(tk.END, msg + "\n")
        text_widget.see(tk.END)
        
    def setup_bridge(self):
        def _run():
            adb_cmd = self.adb_path_var.get().strip()
            commands = [
                [adb_cmd, "reverse", "tcp:9000", "tcp:9000"],
                [adb_cmd, "reverse", "tcp:8001", "tcp:8001"],
                [adb_cmd, "reverse", "tcp:8002", "tcp:8002"],
                [adb_cmd, "forward", "tcp:9001", "tcp:9001"],
                [adb_cmd, "forward", "tcp:9002", "tcp:9002"]
            ]
            self.log_sys("--- Nastavuji ADB Bridge ---")
            
            import os
            startupinfo = None
            if os.name == 'nt':
                startupinfo = subprocess.STARTUPINFO()
                startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW

            for cmd_list in commands:
                cmd_str = ' '.join(cmd_list)
                self.log_sys(f"> {cmd_str}")
                try:
                    result = subprocess.run(cmd_list, capture_output=True, text=True, startupinfo=startupinfo)
                    if result.stdout.strip():
                        self.log_sys(f"  {result.stdout.strip()}")
                    if result.stderr.strip():
                        self.log_sys(f"  ERROR: {result.stderr.strip()}")
                except FileNotFoundError:
                    self.log_sys(f"  ERROR: Příkaz '{adb_cmd}' nebyl nalezen.")
                    self.log_sys("  Zadej prosím platnou cestu k adb.exe (např. C:\\Users\\...\\platform-tools\\adb.exe)")
                    break
                except Exception as e:
                    self.log_sys(f"  Exception: {e}")
            self.log_sys("--- Hotovo ---")
        
        threading.Thread(target=_run, daemon=True).start()

    def sync_sounds(self):
        def _run():
            adb_cmd = self.adb_path_var.get().strip()
            local_dir = r"C:\Work\Robotour\adb\sounds"
            remote_dir = "/sdcard/Android/data/com.unidroids.robot_hmi/files/Sounds"
            
            self.log_sys("--- Synchronizuji zvuky ---")
            import os, glob
            if not os.path.exists(local_dir):
                self.log_sys(f"  Složka '{local_dir}' neexistuje.")
                return

            startupinfo = None
            if os.name == 'nt':
                startupinfo = subprocess.STARTUPINFO()
                startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW

            mkdir_cmd = [adb_cmd, "shell", "mkdir", "-p", remote_dir]
            self.log_sys(f"> {' '.join(mkdir_cmd)}")
            try:
                subprocess.run(mkdir_cmd, startupinfo=startupinfo)
            except Exception as e:
                self.log_sys(f"  Exception: {e}")
                return

            files = glob.glob(os.path.join(local_dir, "*"))
            if not files:
                self.log_sys("  Žádné soubory k synchronizaci.")
            else:
                for f in files:
                    if os.path.isfile(f):
                        push_cmd = [adb_cmd, "push", f, remote_dir + "/"]
                        self.log_sys(f"> {' '.join(push_cmd)}")
                        try:
                            res = subprocess.run(push_cmd, capture_output=True, text=True, startupinfo=startupinfo)
                            if res.stdout.strip(): 
                                self.log_sys(f"  {res.stdout.strip()}")
                            if res.stderr.strip():
                                if res.returncode == 0:
                                    self.log_sys(f"  {res.stderr.strip()}")
                                else:
                                    self.log_sys(f"  ERROR: {res.stderr.strip()}")
                        except Exception as e:
                            self.log_sys(f"  Exception: {e}")
            self.log_sys("--- Hotovo ---")
            
        threading.Thread(target=_run, daemon=True).start()

    def connect_qr(self):
        self.qr_client.connect()

    def disconnect_qr(self):
        self.qr_client.disconnect()

    def connect_term(self):
        self.term_client.connect()

    def disconnect_term(self):
        self.term_client.disconnect()

if __name__ == "__main__":
    root = tk.Tk()
    app = DebuggerApp(root)
    root.mainloop()
