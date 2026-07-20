# configuration.py
import json
import os

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.json')

class CompassConfig:
    def __init__(self):
        self.device = '/dev/robot-compass'
        self.baudrate = 9600
        self.load()

    def load(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    data = json.load(f)
                    self.device = data.get('device', self.device)
                    self.baudrate = data.get('baudrate', self.baudrate)
            except Exception as e:
                print(f"[CompassConfig] Error loading config: {e}")

    def save(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump({
                    'device': self.device,
                    'baudrate': self.baudrate
                }, f, indent=4)
        except Exception as e:
            print(f"[CompassConfig] Error saving config: {e}")
