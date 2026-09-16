# fusion/core_modules/odometry.py
"""
Sledování odometrie podvozku, škálování rychlosti a ZUPT detekce stání.
"""
import time
from typing import Optional, Dict, Any


class OdometryTracker:
    """
    Sledování odometrie kol:
    - Korekce lineární rychlosti kalibračním faktorem 1.05 (v_korig = v_raw / 1.05).
    - ZUPT (Zero Velocity Update) detekce stání z kroků kol (left_steps, right_steps).
      Pokud se kroky po dobu >= 80 ms nemění a rychlost je < 20 mm/s, je detekován klidový stav.
    """

    SCALE_FACTOR = 1.05

    def __init__(self):
        self.raw_speed: float = 0.0
        self.speed: float = 0.0
        self.sAcc: float = 0.05
        self.last_left_steps: Optional[int] = None
        self.last_right_steps: Optional[int] = None
        self.last_steps_change_time: float = time.monotonic()
        self.is_stationary: bool = True

    def update(
        self,
        speed_left: float,
        speed_right: float,
        left_steps: Optional[int] = None,
        right_steps: Optional[int] = None,
        ts: float = 0.0
    ) -> None:
        """
        Aktualizace odometrie:
        speed_left, speed_right v mm/s.
        """
        self.raw_speed = (float(speed_left) + float(speed_right)) / 2.0
        self.speed = self.raw_speed / self.SCALE_FACTOR

        now = time.monotonic()
        if left_steps is not None and right_steps is not None:
            if self.last_left_steps is not None and self.last_right_steps is not None:
                d_left = left_steps - self.last_left_steps
                d_right = right_steps - self.last_right_steps
                if d_left != 0 or d_right != 0 or abs(self.speed) > 20.0:
                    self.last_steps_change_time = now
                    self.is_stationary = False
                else:
                    # Kroky se nezměnily a rychlost je blízká nule
                    if (now - self.last_steps_change_time >= 0.08) and (abs(self.speed) < 20.0):
                        self.is_stationary = True
            else:
                self.last_steps_change_time = now
                self.is_stationary = (abs(self.speed) < 20.0)

            self.last_left_steps = left_steps
            self.last_right_steps = right_steps
        else:
            if abs(self.speed) < 1.0:
                self.is_stationary = True
            else:
                self.is_stationary = False

    def get_diagnostics(self) -> Dict[str, Any]:
        """Diagnostický výstup pro příkaz STATUS."""
        return {
            "raw_speed": round(self.raw_speed, 2),
            "corrected_speed": round(self.speed, 2),
            "scale_factor": self.SCALE_FACTOR,
            "is_stationary": self.is_stationary,
            "left_steps": self.last_left_steps,
            "right_steps": self.last_right_steps
        }
