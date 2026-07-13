import os
import time
import datetime

class OowLogger:
    def __init__(self):
        self.log_file = None
        self.base_time = 0.0
    
    def start(self):
        if self.log_file is not None:
            return
            
        now = datetime.datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S")
        
        log_dir = f"/data/robot/oow/{date_str}"
        os.makedirs(log_dir, exist_ok=True)
        
        file_path = os.path.join(log_dir, f"oow-{time_str}.dat")
        self.log_file = open(file_path, "a", encoding="utf-8")
        self.base_time = time.time()
        
        self.log_file.write(f"BASE_TIME: {self.base_time} {now.isoformat()}\n")
        self.log_file.flush()
        print(f"[OowLogger][INFO] Started logging to {file_path}")

    def stop(self):
        if self.log_file:
            print(f"[OowLogger][INFO] Stopped logging to {self.log_file.name}")
            self.log_file.close()
            self.log_file = None

    def log_event(self, mac_address, event_type, data=""):
        if self.log_file:
            current_time = time.time() - self.base_time
            # Formát: [timestamp] [MAC adresa] [HEARTBEAT/COMMAND] [data/hodnota]
            self.log_file.write(f"{current_time:.6f} {mac_address} {event_type} {data}\n")
            self.log_file.flush()
