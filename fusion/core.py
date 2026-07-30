import time
from typing import Optional
from data.nav_fusion_data import NavFusionData

class FusionCore:
    def __init__(self):
        self.ready = False

        # GPS state
        self._lat: float = 0.0
        self._lon: float = 0.0
        self._hAcc: float = 0.0
        self._have_position: bool = False

        # Heading state (from RTK GPS)
        self._global_heading: float = 0.0
        self._headingAcc: float = 180.0
        self._have_heading: bool = False

        # Compass state
        self._compass_angle: float = 0.0
        self._have_compass: bool = False

        # Fusion state
        self._fused_heading: float = 0.0

    @staticmethod
    def _norm_deg(a: float) -> float:
        """Normalizace do [0, 360)."""
        a = a % 360.0
        if a < 0.0:
            a += 360.0
        return a

    @staticmethod
    def _diff_deg(a_from: float, a_to: float) -> float:
        """Nejkratší rozdíl a_to - a_from v intervalu (-180, 180]."""
        return (a_to - a_from + 180.0) % 360.0 - 180.0

    def _update_ready_flag(self) -> None:
        self.ready = self._have_position and self._have_heading

    def update_position(self, lat: float, lon: float, hAcc: float):
        self._lat = float(lat)
        self._lon = float(lon)
        self._hAcc = float(hAcc)
        self._have_position = True
        self._fuse()

    def update_heading(self, heading: float, headingAcc: float):
        """Heading from dual-antenna GPS (North East)"""
        self._global_heading = self._norm_deg(heading)
        self._headingAcc = float(headingAcc)
        self._have_heading = True
        self._fuse()

    def update_compass(self, yaw: float):
        """Compass yaw (East North)"""
        self._compass_angle = float(yaw)
        self._have_compass = True
        self._fuse()

    def _fuse(self):
        self._update_ready_flag()
        if not self._have_heading:
            return

        # Heading je primární. Pokud je chyba do 5 stupňů, použijeme jej přímo.
        if self._headingAcc <= 5.0:
            self._fused_heading = self._global_heading
        else:
            # Pokud je chyba velká, zkusíme použít kompas, pokud je k dispozici
            if self._have_compass:
                # Kompas je EN, korekce na NE je +90 stupňů
                compass_ne = self._norm_deg(self._compass_angle + 90.0)
                
                # Zjistíme, zda je kompas v rozsahu (heading ± headingAcc)
                diff = abs(self._diff_deg(self._global_heading, compass_ne))
                if diff <= self._headingAcc:
                    self._fused_heading = compass_ne
                else:
                    self._fused_heading = self._global_heading
            else:
                self._fused_heading = self._global_heading

    def get_solution(self) -> NavFusionData:
        return NavFusionData(
            ts_mono=time.monotonic(),
            lat=self._lat,
            lon=self._lon,
            hAcc=self._hAcc,
            heading=self._fused_heading,
            headingAcc=self._headingAcc,
            speed=0.0,
            sAcc=0.0,
            gyroZ=0.0,
            gyroZAcc=0.0,
            gnssFixOK=True if self._hAcc < 10.0 else False,
            drUsed=False,
        )