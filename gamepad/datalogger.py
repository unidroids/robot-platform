import os
import json
import time
from datetime import datetime

class GamepadDataLogger:
    def __init__(self, log_dir="/data/robot/gamepad"):
        self.log_dir = log_dir
        self._log_fp = None
        self.is_logging = False

    def start(self):
        if self.is_logging:
            return True
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.log_dir, f"gamepad_{stamp}.jsonl")
        try:
            self._log_fp = open(path, "a", buffering=1, encoding="utf-8")
            self.is_logging = True
            print(f"[DATALOGER] Log file: {path}")
            return True
        except Exception as e:
            print(f"[DATALOGER] Error opening file: {e}")
            return False

    def stop(self):
        self.is_logging = False
        if self._log_fp:
            try:
                self._log_fp.close()
            except:
                pass
            self._log_fp = None
        print("[DATALOGER] STOP")

    def log_raw_data(self, raw_axes, raw_buttons, raw_hats):
        if self.is_logging and self._log_fp:
            data = {
                "timestamp": time.time(),
                "axes": raw_axes,
                "buttons": raw_buttons,
                "hats": raw_hats
            }
            self._log_fp.write(json.dumps(data) + "\n")
