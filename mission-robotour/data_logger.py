# mission-robotour/data_logger.py
from __future__ import annotations
import os
import json
from datetime import datetime
from pathlib import Path
from typing import Optional, Any


class MissionDataLogger:
    """
    Zaznamenává průběh mise Robotour do:
      /data/robot/mission-robotour/<yyyy-mm-dd>/mission-<HH-MM-SS>.dat
    Den a čas odpovídá zobrazení úvodní obrazovky (nová mise).
    """

    def __init__(self, base_dir: str = "/data/robot/mission-robotour"):
        self.base_dir = Path(base_dir)
        self.file_handle = None
        self.file_path: Optional[Path] = None

    def start_mission(self) -> str:
        """Uzavře předchozí misi (pokud běžela) a otevře nový .dat soubor pro novou misi."""
        self.close()
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S")

        target_dir = self.base_dir / date_str
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
            self.file_path = target_dir / f"mission-{time_str}.dat"
            self.file_handle = open(self.file_path, "w", encoding="utf-8")
        except (PermissionError, OSError):
            # Fallback na lokální složku projektu
            local_base = Path(os.path.dirname(os.path.abspath(__file__))) / "data" / "robot" / "mission-robotour" / date_str
            local_base.mkdir(parents=True, exist_ok=True)
            self.file_path = local_base / f"mission-{time_str}.dat"
            self.file_handle = open(self.file_path, "w", encoding="utf-8")

        self.log("MISSION_INIT", {"start_time": now.isoformat(), "file": str(self.file_path)})
        print(f"[MissionDataLogger] Nová mise otevřena: {self.file_path}")
        return str(self.file_path)

    def log(self, event: str, details: Any = None):
        """Zapíše řádek události ve formátu: <ISO_TIMESTAMP> [<EVENT>] <JSON_PAYLOAD>."""
        if not self.file_handle or self.file_handle.closed:
            return

        ts = datetime.now().isoformat()
        if details is not None:
            if isinstance(details, (dict, list)):
                payload = json.dumps(details, ensure_ascii=False)
            else:
                payload = str(details)
            line = f"{ts} [{event}] {payload}\n"
        else:
            line = f"{ts} [{event}]\n"

        try:
            self.file_handle.write(line)
            self.file_handle.flush()
        except Exception as e:
            print(f"[MissionDataLogger] Chyba zápisu logu: {e}")

    def close(self):
        """Uzavře aktuální soubor mise."""
        if self.file_handle and not self.file_handle.closed:
            self.log("MISSION_CLOSED", {"close_time": datetime.now().isoformat()})
            try:
                self.file_handle.close()
            except Exception:
                pass
            print(f"[MissionDataLogger] Mise uzavřena: {self.file_path}")
            self.file_handle = None
            self.file_path = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
