# fusion/service.py

from __future__ import annotations
import threading
import time
import json
from dataclasses import dataclass, asdict
from typing import Optional

from data.nav_fusion_data import NavFusionData
from core import FusionCore
from poller import FusionPoller
from publisher import FusionPublisher

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

        self.core: Optional[FusionCore] = None
        self.poller: Optional[FusionPoller] = None
        self.publisher: Optional[FusionPublisher] = None

        # auto start
        self._start()

    # ---------------------- stavové API ----------------------

    def _set_state(self, **updates) -> None:
        with self._state_lock:
            for k, v in updates.items():
                setattr(self._state, k, v)
            self._state.ts_mono = time.monotonic()

    def get_state(self) -> str:
        with self._state_lock:
            if not self.running:
                return "IDLE"
            state_dict = asdict(self._state)
            mode = state_dict.pop("mode", "IDLE")
            state_json = json.dumps(state_dict)
            solution = asdict(self.core.get_solution()) if self.core else {}
            solution_json = json.dumps(solution)
            return f"{mode} {state_json} {solution_json}"

    # ---------------------- lifecycle ------------------------

    def _start(self):
        with self._lock:
            if self.running:
                return "OK ALREADY_RUNNING"
            if not self._initialized:
                self.core = FusionCore()
                
                self.poller = FusionPoller(self.core)
                self.publisher = FusionPublisher(self.core, on_publish=self._on_publish_callback)

                self._initialized = True
            
            self.running = True
            self.poller.start()
            self.publisher.start()
            
            self._set_state(mode="WAITING", last_note="SERVICE STARTED")
            print("[SERVICE] STARTED")
            return "OK"

    def _stop(self):
        with self._lock:
            if not self.running:
                return "OK WAS NOT RUNNING"
            
            self.running = False
            
            if self.poller:
                self.poller.stop()
            if self.publisher:
                self.publisher.stop()

            self.core = None
            self._initialized = False
            
            self._set_state(mode="IDLE", last_note="SERVICE STOPPED")
            print("[SERVICE] STOPPED")
            return "OK"

    def restart(self):
        self._stop()
        self._start()
        return "OK"

    def _on_publish_callback(self):
        self._set_state(mode="READY", last_note="SOLUTION PUBLISHED")

    # === Odběratelské API ====================================================

    def get_latest(self) -> Optional[NavFusionData]:
        if self.publisher:
            return self.publisher.get_latest()
        return None

if __name__ == "__main__":
    print("TEST") 
    fusion = FusionService()
    print(fusion.get_state())
    time.sleep(2)  # Give it a moment to run in test
    fusion.restart()
    print(fusion.get_state())
    time.sleep(1)
    fusion._stop()