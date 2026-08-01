# fusion/service.py

from __future__ import annotations
import threading
import time
import json
import zmq
import math
from dataclasses import dataclass, asdict
from typing import Optional

from data.nav_fusion_data import NavFusionData
from core import FusionCore

__all__ = [
    "FusionService"
]

@dataclass
class FusionState:
    mode: str = "IDLE"                 # IDLE | WAITING | READY
    last_note: str = ""
    ts_mono: float = 0.0               # monotonic timestamp poslední aktualizace

class FusionService:

    VERSION = "2.0.0"

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        
        self._state_lock = threading.Lock()
        self._state = FusionState()

        self._latest: Optional[NavFusionData] = None
        self._latest_lock = threading.Lock()

        self.core: Optional[FusionCore] = None
        self._publish_counter = 0

        # ZMQ Context
        self._zmq_context = zmq.Context.instance()
        self._zmq_pub: Optional[zmq.Socket] = None
        self._zmq_sub: Optional[zmq.Socket] = None
        self._zmq_thread: Optional[threading.Thread] = None
        self._pub_thread: Optional[threading.Thread] = None

        # auto start
        self._start()

    # ---------------------- stavové API ----------------------

    def _set_state(self, **updates) -> None:
        with self._state_lock:
            for k, v in updates.items():
                setattr(self._state, k, v)
            self._state.ts_mono = time.monotonic()

    def get_state(self) -> dict:
        with self._state_lock:
            return asdict(self._state)

    # ---------------------- lifecycle ------------------------

    def _start(self):
        with self._lock:
            if self.running:
                return "OK ALREADY_RUNNING"
            if not self._initialized:
                self.core = FusionCore()
                self._publish_counter = 0
                
                # ZMQ Pub
                self._zmq_pub = self._zmq_context.socket(zmq.PUB)
                self._zmq_pub.bind("ipc:///tmp/robot-fusion")
                
                # ZMQ Sub
                self._zmq_sub = self._zmq_context.socket(zmq.SUB)
                
                self._zmq_sub.connect("ipc:///tmp/robot-gps")
                self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "BESTNAV/")
                
                self._zmq_sub.connect("ipc:///tmp/robot-heading")
                self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "UNIHEADING/")
                
                self._zmq_sub.connect("ipc:///tmp/robot-compass")
                self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "COMPASS/GYRO/")
                self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "COMPASS/ANGLE/")
                
                self._zmq_sub.connect("ipc:///tmp/robot-odometry")
                self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "odometry/")

                self._initialized = True
            
            self.running = True
            self._zmq_thread = threading.Thread(target=self._zmq_listener_loop, daemon=True)
            self._zmq_thread.start()

            self._pub_thread = threading.Thread(target=self._publisher_loop, daemon=True)
            self._pub_thread.start()
            
            self._set_state(mode="WAITING", last_note="SERVICE STARTED")
            print("[SERVICE] STARTED")
            return "OK"

    def _stop(self):
        with self._lock:
            if not self.running:
                return "OK WAS NOT RUNNING"
            
            self.running = False
            
            if self._zmq_sub is not None:
                self._zmq_sub.close()
                self._zmq_sub = None
                
            if self._zmq_pub is not None:
                self._zmq_pub.close()
                self._zmq_pub = None
                
            if self._zmq_thread is not None:
                self._zmq_thread.join(timeout=1.0)
                self._zmq_thread = None

            if self._pub_thread is not None:
                self._pub_thread.join(timeout=1.0)
                self._pub_thread = None

            self.core = None
            self._publish_counter = 0
            self._latest = None
            self._initialized = False
            
            self._set_state(mode="IDLE", last_note="SERVICE STOPPED")
            print("[SERVICE] STOPPED")
            return "OK"

    def restart(self):
        self._stop()
        self._start()
        return "OK"

    # ---------------------- ZMQ Loop -----------------
    def _zmq_listener_loop(self):
        while self.running and self._zmq_sub is not None:
            try:
                # Use polling to be able to stop cleanly
                if self._zmq_sub.poll(100):
                    msg = self._zmq_sub.recv_string()
                    
                    if msg.startswith("BESTNAV/"):
                        data = json.loads(msg[len("BESTNAV/"):])
                        lat = data.get("lat", 0.0)
                        lon = data.get("lon", 0.0)
                        lat_std = data.get("lat_std", 0.0)
                        lon_std = data.get("lon_std", 0.0)
                        hAcc = math.hypot(lat_std, lon_std)
                        gpsSol = data.get("pos_type", "NONE")
                        self.core.update_position(lat, lon, hAcc, gpsSol) 
                        
                    elif msg.startswith("UNIHEADING/"):
                        data = json.loads(msg[len("UNIHEADING/"):])
                        heading = data.get("heading", 0.0)
                        hdg_std = data.get("hdg_std", 180.0)
                        headingSol = data.get("pos_type", "NONE")
                        length = data.get("length", 0.0)
                        self.core.update_heading(heading, hdg_std, headingSol, length)
                        
                    elif msg.startswith("odometry/"):
                        data = json.loads(msg[len("odometry/"):])
                        left = data.get("left_speed", 0.0)
                        right = data.get("right_speed", 0.0)
                        self.core.update_odometry(left, right)

                    elif msg.startswith("COMPASS/GYRO/"):
                        data = json.loads(msg[len("COMPASS/GYRO/"):])
                        wz = data.get("wz", 0.0)
                        self.core.update_gyro(wz)
                        
                    elif msg.startswith("COMPASS/ANGLE/"):
                        data = json.loads(msg[len("COMPASS/ANGLE/"):])
                        yaw = data.get("yaw", 0.0)
                        self.core.update_compass_angle(yaw)
                        
            except zmq.error.ContextTerminated:
                break
            except zmq.error.ZMQError:
                break
            except Exception as e:
                print(f"[ZMQ Listener Error] {e}")

    def _publisher_loop(self):
        while self.running:
            if self.core and self.core.ready:
                self._publish(self.core.get_solution())
                self._set_state(mode="READY", last_note="SOLUTION PUBLISHED")
            time.sleep(0.1)

    # === Odběratelské API ====================================================

    def _publish(self, res: NavFusionData) -> None:
        with self._latest_lock:
            self._latest = res

        # Publish over ZMQ
        if self._zmq_pub is not None:
            try:
                self._zmq_pub.send_string(f"SOLUTION/{res.to_json()}")
            except Exception as e:
                print(f"[FUSION ZMQ PUB] Error: {e}")

        self._publish_counter += 1
        if self._publish_counter % 10 == 0:            
            print("published", res.to_json())

    def get_latest(self) -> Optional[NavFusionData]:
        with self._latest_lock:
            return self._latest

if __name__ == "__main__":
    print("TEST") 
    fusion = FusionService()
    print(fusion.get_state())
    fusion.restart()
    print(fusion.get_state())