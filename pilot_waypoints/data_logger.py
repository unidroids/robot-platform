from pathlib import Path
from datetime import datetime

class DataLogger:
    def __init__(self, base_dir="/data/robot/pilot_waypoints"):
        now = datetime.now()
        base = Path(base_dir)
        day_dir = base / now.strftime("%Y-%m-%d")
        day_dir.mkdir(parents=True, exist_ok=True)

        self.file_path = day_dir / f"navigate-{now.strftime('%H-%M-%S')}.csv"
        self.f = open(self.file_path, "w", encoding="utf-8")

    def print(self, *args, **kwargs):
        """
        Ekvivalent built-in print, ale vždy píše do self.f.
        """
        kwargs.pop("file", None)
        print(*args, file=self.f, **kwargs)
        self.f.flush()

    def close(self):
        if hasattr(self, 'f') and not self.f.closed:
            self.f.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
