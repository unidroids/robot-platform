import os
import time
import datetime
import threading
import zmq

class LoggerService:
    def __init__(self):
        self._lock = threading.Lock()
        self.is_running = False
        self.thread = None
        self.shutdown_event = threading.Event()
        
        self.log_file = None
        self.base_time = 0.0
        self.message_count = 0
        
    def start(self) -> bool:
        with self._lock:
            if self.is_running:
                return False
                
            self.shutdown_event.clear()
            self._open_new_file()
            
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            self.is_running = True
            return True

    def stop(self) -> bool:
        with self._lock:
            if not self.is_running:
                return False
                
            self.shutdown_event.set()
            return True

    def wait_for_stop(self):
        if self.thread:
            self.thread.join(timeout=3.0)
            self.is_running = False
            self.thread = None

    def _open_new_file(self):
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S")
        
        log_dir = f"/data/robot/logger/{date_str}"
        os.makedirs(log_dir, exist_ok=True)
        
        file_path = os.path.join(log_dir, f"logger-{time_str}.dat")
        self.log_file = open(file_path, "a", encoding="utf-8")
        
        self.base_time = time.monotonic()
        self.message_count = 0
        
        # První speciální řádek
        self.log_file.write(f"BASE_TIME: {self.base_time} {now.isoformat()}\n")
        self.log_file.flush()

    def _close_file(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def get_status(self) -> str:
        with self._lock:
            if self.is_running:
                return f"RUNNING {self.message_count}"
            return "IDLE"

    def _run(self):
        filename = self.log_file.name if self.log_file else "žádný"
        print(f"📥 [Logger] Vlákno spuštěno, připojuji se k senzorům. Zápis do: {filename}")
        context = zmq.Context.instance()
        
        endpoints = [
            "ipc:///tmp/robot-camera",
            "ipc:///tmp/robot-vision",
            "ipc:///tmp/robot-lidar",
            "ipc:///tmp/robot-odometry"
        ]
        
        sockets = {}
        poller = zmq.Poller()
        
        for ep in endpoints:
            sub = context.socket(zmq.SUB)
            sub.connect(ep)
            sub.setsockopt_string(zmq.SUBSCRIBE, "")
            sockets[sub] = ep
            poller.register(sub, zmq.POLLIN)
            
        last_print_time = time.time()
        messages_since_last_print = 0
        messages_saved = 0
        
        try:
            while not self.shutdown_event.is_set():
                events = dict(poller.poll(timeout=100))
                
                for sub, event in events.items():
                    if event == zmq.POLLIN:
                        parts = sub.recv_multipart()
                        current_time = time.monotonic() - self.base_time
                        
                        endpoint = sockets[sub]
                        
                        # Sloučení částí zprávy do textu
                        text_parts = []
                        for p in parts:
                            try:
                                text_parts.append(p.decode("utf-8").strip())
                            except:
                                text_parts.append(p.hex())
                                
                        data_str = " ".join(text_parts)
                        
                        if self.log_file:
                            # Formát: [timestamp offset] [kanál/topic] [data]
                            self.log_file.write(f"{current_time:.6f} {endpoint} {data_str}\n")
                            
                        self.message_count += 1
                        messages_since_last_print += 1

                # Vypsání statistik každých 5 vteřin
                now = time.time()
                if now - last_print_time >= 5.0:
                    current_time_str = datetime.datetime.now().strftime("%H:%M:%S")
                    print(f"📊 [Logger] {current_time_str} Uloženo: {self.message_count}, rychlost: {messages_since_last_print/5:.1f} zpráv/s.")
                    messages_since_last_print = 0
                    last_print_time = now
                    
        except Exception as e:
            print(f"❌ [Logger] Chyba ve vláknu: {e}")
        finally:
            for sub in sockets.keys():
                poller.unregister(sub)
                sub.close()
            filename = self.log_file.name if self.log_file else "neznámý"
            self._close_file()
            print(f"🛑 [Logger] Vlákno ukončeno, soubor {filename} uzavřen. Uloženo {self.message_count} zpráv")

service = LoggerService()
