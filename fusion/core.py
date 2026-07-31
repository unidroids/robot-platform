import time
from data.nav_fusion_data import NavFusionData

class FusionCore:
    def __init__(self):
        self.ready = False

        # GPS state
        self._lat: float = 0.0
        self._lon: float = 0.0
        self._hAcc: float = 0.0
        self._gpsSol: str = "NONE"
        self._have_position: bool = False

        # Heading state (from RTK GPS)
        self._global_heading: float = 0.0
        self._headingAcc: float = 180.0
        self._headingSol: str = "NONE"
        self._have_heading: bool = False

        # Odometry state
        self._speed: float = 0.0
        self._sAcc: float = 0.05

        # Compass state
        self._gyroZ: float = 0.0
        self._gyroZAcc: float = 5.0

        # Fusion state
        self._fused_heading: float = 0.0
        self._fusionSol: str = "COPY"

    @staticmethod
    def _norm_deg(a: float) -> float:
        """Normalizace do [0, 360)."""
        a = a % 360.0
        if a < 0.0:
            a += 360.0
        return a

    def _update_ready_flag(self) -> None:
        self.ready = self._have_position and self._have_heading

    def update_position(self, lat: float, lon: float, hAcc: float, gpsSol: str):
        self._lat = float(lat)
        self._lon = float(lon)
        self._hAcc = float(hAcc)
        self._gpsSol = gpsSol
        self._have_position = True
        self._fuse()

    def update_heading(self, heading: float, headingAcc: float, headingSol: str):
        """Heading from dual-antenna GPS (North East)"""
        self._global_heading = self._norm_deg(heading)
        self._headingAcc = float(headingAcc)
        self._headingSol = headingSol
        self._have_heading = True
        self._fuse()

    def update_odometry(self, speed_left: float, speed_right: float):
        """Odometry speed in mm/s"""
        self._speed = (speed_left + speed_right) / 2.0

    def update_gyro(self, wz: float):
        """Compass gyro in deg/s"""
        self._gyroZ = float(wz)

    def _fuse(self):
        self._update_ready_flag()
        if not self._have_heading:
            return

        # Heading je primární
        self._fused_heading = self._global_heading

    def get_solution(self) -> NavFusionData:
        return NavFusionData(
            ts_mono=time.monotonic(),
            lat=self._lat,
            lon=self._lon,
            hAcc=self._hAcc,
            heading=self._fused_heading,
            headingAcc=self._headingAcc,
            speed=self._speed,
            sAcc=self._sAcc,
            gyroZ=self._gyroZ,
            gyroZAcc=self._gyroZAcc,
            gpsSol=self._gpsSol,
            headingSol=self._headingSol,
            fusionSol=self._fusionSol,
        )