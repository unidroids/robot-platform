# fusion/core_modules/epoch_sync.py
"""
Synchronizátor GNSS epoch pro shlukování polohových zpráv (BESTNAV, BESTNAVH)
podle GPS týdne a milisekund v týdnu (wn, ms) z modulů UM980 a UM982.
"""
import time
import threading
from typing import Optional, Dict, Any, Tuple, Callable


class EpochSynchronizer:
    """
    Shlukuje polohové GNSS zprávy (BESTNAV, BESTNAVH) ze stejné fyzické epochy (wn, ms)
    z různých přijímačů (UM980, UM982) a odesílá je do jádra fúze jako jeden celek.
    - Okamžité odeslání při kompletaci všech 3 antén (žádné čekání).
    - Mikro-časovač (timeout_s = 25 ms) pro případ výpadku jedné antény.
    - Zprávy bez (wn, ms) odesílá okamžitě.
    """

    def __init__(self, dispatch_fn: Callable[[Dict[str, Any]], None], timeout_s: float = 0.025):
        self.dispatch_fn = dispatch_fn
        self.timeout_s = timeout_s
        self.current_key: Optional[Tuple[int, int]] = None
        self.epoch_start_mono: float = 0.0
        self.pending_msgs: Dict[str, Any] = {}
        self._lock = threading.Lock()

    def has_pending(self) -> bool:
        with self._lock:
            return self.current_key is not None

    def remaining_time_ms(self, now_mono: Optional[float] = None) -> int:
        with self._lock:
            if self.current_key is None:
                return 100
            if now_mono is None:
                now_mono = time.monotonic()
            rem_s = self.timeout_s - (now_mono - self.epoch_start_mono)
            return max(1, min(100, int(rem_s * 1000.0)))

    def add_msg(self, ant_name: str, data: Dict[str, Any], now_mono: Optional[float] = None) -> None:
        if now_mono is None:
            now_mono = time.monotonic()

        wn = data.get("wn")
        ms = data.get("ms")

        to_send = None
        with self._lock:
            if wn is None or ms is None:
                to_send = {ant_name: data}
            else:
                key = (int(wn), int(ms))

                if self.current_key is None:
                    self.current_key = key
                    self.epoch_start_mono = now_mono
                    self.pending_msgs = {ant_name: data}
                elif key == self.current_key:
                    self.pending_msgs[ant_name] = data
                    # Pokud dorazily všechny 3 očekávané antény, okamžitě uzavíráme
                    if ("gps" in self.pending_msgs and
                        "master" in self.pending_msgs and
                        "slave" in self.pending_msgs):
                        to_send = self.pending_msgs
                        self.current_key = None
                        self.pending_msgs = {}
                elif key > self.current_key:
                    # Dorazila novější epocha dříve než timeout staré -> odešleme starou
                    to_send = self.pending_msgs
                    self.current_key = key
                    self.epoch_start_mono = now_mono
                    self.pending_msgs = {ant_name: data}
                else:
                    # Zpožděná zpráva ze staré uzavřené epochy -> ignorujeme
                    pass

        if to_send:
            self._dispatch_epoch_unlocked(to_send)

    def check_timeout(self, now_mono: Optional[float] = None) -> None:
        if now_mono is None:
            now_mono = time.monotonic()

        to_send = None
        with self._lock:
            if self.current_key is not None and (now_mono - self.epoch_start_mono) >= self.timeout_s:
                to_send = self.pending_msgs
                self.current_key = None
                self.pending_msgs = {}

        if to_send:
            self._dispatch_epoch_unlocked(to_send)

    def flush(self) -> None:
        to_send = None
        with self._lock:
            if self.pending_msgs:
                to_send = self.pending_msgs
                self.current_key = None
                self.pending_msgs = {}
        if to_send:
            self._dispatch_epoch_unlocked(to_send)

    def _dispatch_epoch_unlocked(self, msgs: Dict[str, Any]) -> None:
        try:
            self.dispatch_fn(msgs)
        except Exception as e:
            print(f"[EpochSynchronizer Dispatch Error] {e}")
