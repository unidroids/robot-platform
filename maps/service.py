# maps/service.py
"""
Služba MAPS (MapService):
- Správa životního cyklu služby (START, STOP, RESTART, STATUS).
- Načtení mapy (defaut_map.json) a vytvoření runtime kopie do /data/robot/maps/<yyyy-mm-dd>/<HH-MM-SS>/.
- Poskytování byznys logiky vyhledávání tras (find_route).
"""
from __future__ import annotations
import datetime
import json
import os
import shutil
import threading
import time
from typing import Optional, Dict, Any

try:
    from .map_graph import MapGraph
    from .route_planner import RoutePlanner
except (ImportError, ValueError):
    from map_graph import MapGraph
    from route_planner import RoutePlanner

__all__ = ["MapService"]


class MapService:
    """
    Centrální služba pro správu map a vyhledávání tras robota.
    """
    VERSION = "1.0.0"

    def __init__(self, map_file_path: Optional[str] = None):
        self.running: bool = False
        self._lock = threading.Lock()
        self.service_start_time: str = ""
        self.last_query_time: str = ""
        self.last_query_result: str = ""
        self.last_copied_path: str = ""

        # Určení cesty k výchozí mapě
        if map_file_path:
            self.map_file_path = os.path.abspath(map_file_path)
        else:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            self.map_file_path = os.path.join(base_dir, "defaut_map.json")

        self.map_graph = MapGraph()
        self.planner: Optional[RoutePlanner] = None

        # Automatické spuštění při inicializaci
        self._start()

    # ---------------------- Životní cyklus ----------------------

    def _start(self) -> str:
        """Spustí mapovou službu, načte mapu a vytvoří její záložní kopii."""
        with self._lock:
            if self.running:
                return "OK ALREADY_RUNNING"

            if not os.path.exists(self.map_file_path):
                print(f"[MAPS ERROR] Soubor mapy neexistuje: {self.map_file_path}")
                return f"ERR MAP FILE NOT FOUND: {self.map_file_path}"

            # 1. Načtení grafu do paměti
            success = self.map_graph.load_from_file(self.map_file_path)
            if not success:
                print(f"[MAPS ERROR] Selhalo načtení mapy ze souboru: {self.map_file_path}")
                return "ERR FAILED TO LOAD MAP"

            self.planner = RoutePlanner(self.map_graph)

            # 2. Vytvoření instance (kopie) mapy ve složce /data/robot/maps/<yyyy-mm-dd>/<HH-MM-SS>/
            now = datetime.datetime.now()
            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H-%M-%S")
            self.service_start_time = now.isoformat()

            self.last_copied_path = self._copy_map_instance(date_str, time_str)

            self.running = True
            print(f"[MAPS SERVICE] STARTED. Nodes: {len(self.map_graph.nodes)}, Edges: {len(self.map_graph.edges)}")
            return "OK"

    def _stop(self) -> str:
        """Zastaví mapovou službu."""
        with self._lock:
            if not self.running:
                return "OK WAS NOT RUNNING"

            self.running = False
            print("[MAPS SERVICE] STOPPED")
            return "OK"

    def restart(self) -> str:
        """Restartuje mapovou službu."""
        self._stop()
        return self._start()

    def get_state(self) -> str:
        """Vrátí stav služby ve standardním jednořádkovém formátu pro příkaz STATUS."""
        with self._lock:
            mode = "READY" if self.running else "IDLE"
            info = {
                "service": "MAPS",
                "version": self.VERSION,
                "mode": mode,
                "map_file": os.path.basename(self.map_file_path),
                "area_name": self.map_graph.metadata.get("area_name", "") if self.map_graph else "",
                "nodes_count": len(self.map_graph.nodes) if self.map_graph else 0,
                "edges_count": len(self.map_graph.edges) if self.map_graph else 0,
                "started_at": self.service_start_time,
                "instance_copy": self.last_copied_path,
                "last_query_result": self.last_query_result
            }
            return f"{mode} {json.dumps(info)}"

    # ---------------------- Byznys logika ----------------------

    def find_route(
        self,
        start_lat: float,
        start_lon: float,
        goal_lat: float,
        goal_lon: float
    ) -> Dict[str, Any]:
        """
        Vyhledá optimální trasu z místa (start_lat, start_lon) do (goal_lat, goal_lon)
        s ověřením 5m vzdálenosti a pravostranným offsetem.
        """
        with self._lock:
            if not self.running or not self.planner:
                return {
                    "metadata": {
                        "search_result": "cesta nenalezena",
                        "reason": "Služba MAPS není spuštěna (použijte START).",
                        "route_length_m": 0.0
                    },
                    "nodes": [],
                    "edges": []
                }

            self.last_query_time = datetime.datetime.now().isoformat()
            result = self.planner.plan_route(
                start_lat, start_lon,
                goal_lat, goal_lon,
                service_start_time=self.service_start_time
            )
            self.last_query_result = result["metadata"].get("search_result", "")
            return result

    # ---------------------- Pomocné metody ----------------------

    def _copy_map_instance(self, date_str: str, time_str: str) -> str:
        """
        Vytvoří kopii výchozí mapy do cílové složky /data/robot/maps/<yyyy-mm-dd>/<HH-MM-SS>/.
        Při absenci práv zápisu do root /data použije bezpečný fallback do lokální složky.
        """
        # Cílová cesta dle specifikace pro Linux
        target_dir = f"/data/robot/maps/{date_str}/{time_str}"
        copied_path = ""

        try:
            os.makedirs(target_dir, exist_ok=True)
            dest_file = os.path.join(target_dir, "defaut_map.json")
            shutil.copy2(self.map_file_path, dest_file)
            copied_path = dest_file
            print(f"[MAPS] Map instance copied to: {copied_path}")
        except (PermissionError, OSError) as e:
            # Fallback pro vývoj / testovací prostředí (např. lokální adresář projektu)
            fallback_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "data", "robot", "maps", date_str, time_str
            )
            try:
                os.makedirs(fallback_dir, exist_ok=True)
                dest_file = os.path.join(fallback_dir, "defaut_map.json")
                shutil.copy2(self.map_file_path, dest_file)
                copied_path = dest_file
                print(f"[MAPS] Note: /data inaccessible ({e}), map copied to fallback: {copied_path}")
            except Exception as ex:
                print(f"[MAPS WARNING] Failed to copy map instance: {ex}")

        return copied_path
