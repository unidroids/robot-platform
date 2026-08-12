import threading
import time
import json
import zmq
from dataclasses import asdict
from typing import Optional, Callable

from core import FusionCore
from data.nav_fusion_data import NavFusionData

class FusionPublisher:
    def __init__(self, core: FusionCore, on_publish: Callable[[], None] = None):
        self.core = core
        self.running = False
        self._zmq_context = zmq.Context.instance()
        self._zmq_pub = None
        self._thread = None
        
        self._latest: Optional[NavFusionData] = None
        self._latest_lock = threading.Lock()
        self._publish_counter = 0
        self.on_publish = on_publish

    def start(self):
        if self.running: return
        self._zmq_pub = self._zmq_context.socket(zmq.PUB)
        self._zmq_pub.bind("ipc:///tmp/robot-fusion")
        
        self.running = True
        self._publish_counter = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._zmq_pub:
            self._zmq_pub.close()
            self._zmq_pub = None
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._latest = None

    def get_latest(self) -> Optional[NavFusionData]:
        with self._latest_lock:
            return self._latest

    def _loop(self):
        last_debug_time = time.time()
        while self.running:
            if self.core and self.core.ready:
                res = self.core.get_solution()
                self._publish(res)
                if self.on_publish:
                    self.on_publish()
                
            now = time.time()
            if now - last_debug_time >= 1.0 and self.core:
                self._publish_debug_headings()
                last_debug_time = now
                
            time.sleep(0.1)

    def _publish(self, res: NavFusionData) -> None:
        with self._latest_lock:
            self._latest = res

        if self._zmq_pub is not None:
            try:
                self._zmq_pub.send_multipart([b"SOLUTION", res.to_json().encode('utf-8')])
            except Exception as e:
                print(f"[FUSION ZMQ PUB] Error: {e}")

        self._publish_counter += 1
        if self._publish_counter % 50 == 0:            
            print("published", res.to_json())

    def _publish_debug_headings(self):
        if self._zmq_pub is None or not self.core:
            return
        debug_data = {
            "gps_heading": asdict(self.core.gps_heading),
            "dual_heading": asdict(self.core.dual_heading),
            "compass_heading": asdict(self.core.compass_heading),
            "fused_heading": asdict(self.core.fused_heading)
        }
        try:
            self._zmq_pub.send_multipart([b"DEBUG_HEADING", json.dumps(debug_data).encode('utf-8')])
            print("published debug", res.to_json())
        except Exception as e:
            print(f"[FUSION ZMQ PUB] Debug Error: {e}")
