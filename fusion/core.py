import time
from dataclasses import dataclass
from data.nav_fusion_data import NavFusionData

@dataclass
class HeadingData:
    heading: float = 0.0
    acc: float = 180.0
    sol: str = "NONE"

class FusionCore:
    def __init__(self):
        self.ready = False

        # GPS state
        self._lat: float = 0.0
        self._lon: float = 0.0
        self._hAcc: float = 0.0
        self._gpsSol: str = "NONE"
        self._have_position: bool = False

        # Headings
        self.gps_heading = HeadingData()
        self.compass_heading = HeadingData()
        self.default_heading = HeadingData()
        self.fused_heading = HeadingData()

        # Odometry state
        self._speed: float = 0.0
        self._sAcc: float = 0.05

        # Compass state
        self._gyroZ: float = 0.0
        self._gyroZAcc: float = 5.0

        # Fusion state
        self._fusionSol: str = "COPY"

    @staticmethod
    def _norm_deg(a: float) -> float:
        """Normalizace do [0, 360)."""
        a = a % 360.0
        if a < 0.0:
            a += 360.0
        return a

    def _update_ready_flag(self) -> None:
        self.ready = self._have_position and (self.gps_heading.sol != "NONE" or self.compass_heading.sol != "NONE")

    def update_position(self, lat: float, lon: float, hAcc: float, gpsSol: str):
        self._lat = float(lat)
        self._lon = float(lon)
        self._hAcc = float(hAcc)
        self._gpsSol = gpsSol
        self._have_position = True
        self._fuse_heading()

    def update_heading(self, heading: float, headingAcc: float, headingSol: str, length: float = 0.0):
        """Heading from dual-antenna GPS (North East)"""
        self.gps_heading.heading = self._norm_deg(heading)
        self.gps_heading.acc = float(headingAcc)
        self.gps_heading.sol = headingSol
        self._fuse_heading()

    def update_odometry(self, speed_left: float, speed_right: float):
        """Odometry speed in mm/s"""
        self._speed = (speed_left + speed_right) / 2.0

    def update_gyro(self, wz: float):
        """Compass gyro in deg/s"""
        self._gyroZ = float(wz)

    def update_compass_angle(self, yaw: float):
        """Compass angle in deg"""
        self.compass_heading.heading = self._norm_deg(185-float(yaw))
        self.compass_heading.acc = 4.0
        self.compass_heading.sol = "COMPASS"
        self._fuse_heading()

    def _fuse_heading(self):
        self._update_ready_flag()

        if self.gps_heading.sol != "NONE" and self.gps_heading.acc < 5.0:
            self.fused_heading.heading = self.gps_heading.heading
            self.fused_heading.acc = self.gps_heading.acc
            self.fused_heading.sol = self.gps_heading.sol
            self._fusionSol = "UNIHEADING"
        elif self.compass_heading.sol != "NONE":
            self.fused_heading.heading = self.compass_heading.heading
            self.fused_heading.acc = self.compass_heading.acc
            self.fused_heading.sol = self.compass_heading.sol
            self._fusionSol = "COMPASS"
        else:
            # Fallback (nemáme přesnou GPS a nemáme kompas)
            self.fused_heading.heading = self.default_heading.heading
            self.fused_heading.acc = self.default_heading.acc
            self.fused_heading.sol = self.default_heading.sol
            self._fusionSol = "NONE"

    def get_solution(self) -> NavFusionData:
        return NavFusionData(
            ts_mono=time.monotonic(),
            lat=self._lat,
            lon=self._lon,
            hAcc=self._hAcc,
            heading=self.fused_heading.heading,
            headingAcc=self.fused_heading.acc,
            speed=self._speed,
            sAcc=self._sAcc,
            gyroZ=self._gyroZ,
            gyroZAcc=self._gyroZAcc,
            gpsSol=self._gpsSol,
            headingSol=self.fused_heading.sol,
            fusionSol=self._fusionSol,
        )