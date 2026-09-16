# fusion/core.py
"""
Jádro fúze senzorických dat (fusion).
Fasáda a koordinátor propojující doménové komponenty:
- ConstellationManager (geometrie trojúhelníku, FDE, transformace polohy [0,0,0])
- HeadingManager (UNIHEADING, 3-anténní kinematický konsenzus, gyro DR)
- OdometryTracker (škálování rychlosti 1.05, kroky kol, ZUPT detekce stání)
"""
from __future__ import annotations
import time
from typing import Optional, Dict, Any, List
from collections import deque

from data.nav_fusion_data import NavFusionData
from core_modules.geo_math import (
    norm_deg,
    angle_diff,
    circular_mean,
    geo_dist,
    compute_tangent_offset
)
from core_modules.odometry import OdometryTracker
from core_modules.constellation import AntennaState, ConstellationManager
from core_modules.heading import HeadingData, HeadingManager
from core_modules.position import PositionDeadReckoningTracker

__all__ = [
    "FusionCore",
    "HeadingData",
    "AntennaState",
    "PositionDeadReckoningTracker"
]


class FusionCore:
    """
    Koordinátor a fasáda fúze senzorických dat.
    100% zpětně kompatibilní veřejné API i stavové atributy.
    """

    ODOMETRY_SCALE_FACTOR = OdometryTracker.SCALE_FACTOR

    def __init__(self):
        self.ready: bool = False
        self._last_imu_ts: Optional[float] = None
        self._last_msg_mono_ts: Optional[float] = None

        # Doménové manažery
        self.odometry = OdometryTracker()
        self.constellation = ConstellationManager()
        self.heading = HeadingManager()
        self.position_tracker = PositionDeadReckoningTracker()

        # IMU / Náklon a akcelerace
        self._gyroZ: float = 0.0
        self._gyroZAcc: float = 2.0
        self._pitch: float = 0.0
        self._roll: float = 0.0
        self._ax: float = 0.0
        self._ay: float = 0.0
        self._az: float = 9.81

    # --- Delegované statické metody pro zpětnou kompatibilitu ---
    @staticmethod
    def _norm_deg(a: float) -> float:
        return norm_deg(a)

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        return angle_diff(target, current)

    @staticmethod
    def _circular_mean(angles: List[float]) -> float:
        return circular_mean(angles)

    @staticmethod
    def _geo_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        return geo_dist(lat1, lon1, lat2, lon2)

    @staticmethod
    def _compute_tangent_offset(offset_x: float, offset_y: float, v_ms: float, wz_deg_s: float) -> float:
        return compute_tangent_offset(offset_x, offset_y, v_ms, wz_deg_s)

    # --- Vlastnosti pro zpětnou kompatibilitu atributů ---

    @property
    def bestnav_pos_type(self) -> str:
        return self.constellation.bestnav_pos_type

    @property
    def uniheading_pos_type(self) -> str:
        return self.heading.uniheading_pos_type

    @property
    def gps_heading(self) -> HeadingData:
        return self.heading.gps_heading

    @gps_heading.setter
    def gps_heading(self, val: HeadingData) -> None:
        self.heading.gps_heading = val

    @property
    def dual_heading(self) -> HeadingData:
        return self.heading.dual_heading

    @dual_heading.setter
    def dual_heading(self, val: HeadingData) -> None:
        self.heading.dual_heading = val

    @property
    def fused_heading(self) -> HeadingData:
        return self.heading.fused_heading

    @fused_heading.setter
    def fused_heading(self, val: HeadingData) -> None:
        self.heading.fused_heading = val

    @property
    def _heading_initialized(self) -> bool:
        return self.heading.heading_initialized

    @_heading_initialized.setter
    def _heading_initialized(self, val: bool) -> None:
        self.heading.heading_initialized = val

    @property
    def _heading_source(self) -> str:
        return self.heading.heading_source

    @_heading_source.setter
    def _heading_source(self, val: str) -> None:
        self.heading.heading_source = val

    @property
    def _fusionSol(self) -> str:
        return self.heading.fusionSol

    @_fusionSol.setter
    def _fusionSol(self, val: str) -> None:
        self.heading.fusionSol = val

    @property
    def _heading_consensus_status(self) -> str:
        return self.heading.heading_consensus_status

    @_heading_consensus_status.setter
    def _heading_consensus_status(self, val: str) -> None:
        self.heading.heading_consensus_status = val

    @property
    def _bestnav_course_buffer(self) -> deque:
        return self.heading.bestnav_course_buffer

    @property
    def _raw_speed(self) -> float:
        return self.odometry.raw_speed

    @_raw_speed.setter
    def _raw_speed(self, val: float) -> None:
        self.odometry.raw_speed = val

    @property
    def _speed(self) -> float:
        return self.odometry.speed

    @_speed.setter
    def _speed(self, val: float) -> None:
        self.odometry.speed = val

    @property
    def _sAcc(self) -> float:
        return self.odometry.sAcc

    @_sAcc.setter
    def _sAcc(self, val: float) -> None:
        self.odometry.sAcc = val

    @property
    def _is_stationary(self) -> bool:
        return self.odometry.is_stationary

    @_is_stationary.setter
    def _is_stationary(self, val: bool) -> None:
        self.odometry.is_stationary = val

    @property
    def _last_left_steps(self) -> Optional[int]:
        return self.odometry.last_left_steps

    @property
    def _last_right_steps(self) -> Optional[int]:
        return self.odometry.last_right_steps

    @property
    def _last_steps_change_time(self) -> float:
        return self.odometry.last_steps_change_time

    @property
    def _ant_gps(self) -> AntennaState:
        return self.constellation.ant_gps

    @property
    def _ant_master(self) -> AntennaState:
        return self.constellation.ant_master

    @property
    def _ant_slave(self) -> AntennaState:
        return self.constellation.ant_slave

    @property
    def _triangle_status(self) -> str:
        return self.constellation.triangle_status

    @_triangle_status.setter
    def _triangle_status(self, val: str) -> None:
        self.constellation.triangle_status = val

    @property
    def _active_antenna_name(self) -> str:
        return self.constellation.active_antenna_name

    @_active_antenna_name.setter
    def _active_antenna_name(self, val: str) -> None:
        self.constellation.active_antenna_name = val

    @property
    def _dist_dual(self) -> float:
        return self.constellation.dist_dual

    @property
    def _dist_gps_master(self) -> float:
        return self.constellation.dist_gps_master

    @property
    def _dist_gps_slave(self) -> float:
        return self.constellation.dist_gps_slave

    @property
    def _center_lat(self) -> float:
        return self.position_tracker.lat if self.position_tracker.have_position else self.constellation.center_lat

    @property
    def _center_lon(self) -> float:
        return self.position_tracker.lon if self.position_tracker.have_position else self.constellation.center_lon

    @property
    def _center_hAcc(self) -> float:
        return self.position_tracker.hAcc if self.position_tracker.have_position else self.constellation.center_hAcc

    @property
    def _gpsSol(self) -> str:
        return self.position_tracker.pos_type if self.position_tracker.dr_active else self.constellation.gpsSol

    @property
    def _have_position(self) -> bool:
        return self.position_tracker.have_position or self.constellation.have_position

    @property
    def _dr_active(self) -> bool:
        return self.position_tracker.dr_active

    def _update_ready_flag(self) -> None:
        self.ready = self._have_position and self.heading.heading_initialized

    # ------------------ Anténní aktualizace ------------------

    def update_gps_antenna(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0
    ) -> None:
        """Aktualizace primární GNSS antény UM980 (gnss-gps)."""
        self.constellation.update_gps(lat, lon, hAcc, pos_type, trk_gnd, hor_spd)
        self._last_msg_mono_ts = time.monotonic()
        self._evaluate_constellation_and_position()
        self._evaluate_antenna_headings()

    def update_master_antenna(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0
    ) -> None:
        """Aktualizace Master antény UM982 (gnss-dual)."""
        self.constellation.update_master(lat, lon, hAcc, pos_type, trk_gnd, hor_spd)
        self._last_msg_mono_ts = time.monotonic()
        self._evaluate_constellation_and_position()
        self._evaluate_antenna_headings()

    def update_slave_antenna(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0
    ) -> None:
        """Aktualizace Slave antény UM982 (gnss-dual BESTNAVH)."""
        self.constellation.update_slave(lat, lon, hAcc, pos_type, trk_gnd, hor_spd)
        self._last_msg_mono_ts = time.monotonic()
        self._evaluate_constellation_and_position()
        self._evaluate_antenna_headings()

    def update_position(self, lat: float, lon: float, hAcc: float, gpsSol: str) -> None:
        """Zpětná kompatibilita pro staré volání."""
        self.update_gps_antenna(lat, lon, hAcc, gpsSol)

    def _evaluate_constellation_and_position(self) -> None:
        self.constellation.evaluate_constellation_and_position(
            heading_deg=self.heading.fused_heading.heading,
            heading_initialized=self.heading.heading_initialized
        )
        is_rtk = (self.constellation.bestnav_pos_type in ConstellationManager.RTK_SOLUTIONS) and (
            self.constellation.triangle_status in (
                "TRIANGLE_OK", "EXCLUDED_GPS", "EXCLUDED_MASTER", "EXCLUDED_SLAVE",
                "PAIR_DUAL_OK", "PAIR_GPS_MASTER_OK", "PAIR_GPS_SLAVE_OK",
                "SINGLE_GPS", "SINGLE_MASTER", "SINGLE_SLAVE"
            )
        )
        if self.constellation.have_position:
            self.position_tracker.update_gnss(
                lat=self.constellation.center_lat,
                lon=self.constellation.center_lon,
                hAcc=self.constellation.center_hAcc,
                pos_type=self.constellation.gpsSol,
                is_rtk=is_rtk,
                ts_mono=time.monotonic()
            )
        self._update_ready_flag()

    def _evaluate_antenna_headings(self) -> None:
        self.heading.evaluate_consensus(
            ant_gps=self.constellation.ant_gps,
            ant_master=self.constellation.ant_master,
            ant_slave=self.constellation.ant_slave,
            speed=self.odometry.speed,
            gyroZ=self._gyroZ,
            is_stationary=self.odometry.is_stationary
        )
        self._update_ready_flag()

    # ------------------ Heading management ------------------

    def update_dual_heading(self, heading: float, headingAcc: float, headingSol: str, length: float = 0.0) -> None:
        """Heading z duální antény UM982 (UNIHEADING)."""
        self._last_msg_mono_ts = time.monotonic()
        self.heading.update_dual_heading(heading, headingAcc, headingSol, length)
        self._update_ready_flag()

    def update_gps_heading(self, heading: float, headingAcc: float, headingSol: str, hor_spd: float = 0.0) -> None:
        """Kurz z pohybu (BESTNAV trk_gnd)."""
        self.heading.update_single_course(
            heading=heading,
            headingAcc=headingAcc,
            headingSol=headingSol,
            hor_spd=hor_spd,
            gyroZ=self._gyroZ,
            is_stationary=self.odometry.is_stationary
        )
        self._update_ready_flag()

    # ------------------ Odometrie ------------------

    def update_odometry(
        self,
        speed_left: float,
        speed_right: float,
        left_steps: Optional[int] = None,
        right_steps: Optional[int] = None,
        ts: float = 0.0
    ) -> None:
        """Odometrická rychlost a kroky kol z podvozku."""
        self.odometry.update(speed_left, speed_right, left_steps, right_steps, ts)
        self.position_tracker.update_odometry(
            speed_mm_s=self.odometry.speed,
            is_stationary=self.odometry.is_stationary,
            heading_deg=self.heading.fused_heading.heading,
            heading_valid=self.heading.heading_initialized,
            ts_mono=time.monotonic()
        )
        self._update_ready_flag()

    # ------------------ IMU a náklon ------------------

    def update_imu(
        self,
        ts: float,
        delta_yaw: float,
        wz: float,
        pitch: float = 0.0,
        roll: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
        az: float = 9.81
    ) -> None:
        """Zpracování 20Hz zpráv z gnss-imu (topic GYRO)."""
        self._gyroZ = float(wz)
        self._pitch = float(pitch)
        self._roll = float(roll)
        self._ax = float(ax)
        self._ay = float(ay)
        self._az = float(az)
        self._last_msg_mono_ts = time.monotonic()

        self.heading.update_imu(delta_yaw, wz, self.odometry.is_stationary)
        self._last_imu_ts = ts
        self._update_ready_flag()

    def update_gyro(self, ts: float, wz: float) -> None:
        """Zpětná kompatibilita."""
        dt = 0.05
        if self._last_imu_ts is not None:
            calc_dt = ts - self._last_imu_ts
            if 0 < calc_dt < 1.0:
                dt = calc_dt
        delta_yaw = float(wz) * dt
        self.update_imu(ts, delta_yaw, wz)

    # ------------------ Výstupy a diagnostika ------------------

    def get_solution(self) -> NavFusionData:
        """Sestaví aktuální navigační řešení pro publikaci (PILOT)."""
        now = time.monotonic()
        self.heading.check_stale(now, self._last_msg_mono_ts)

        if self.position_tracker.dr_active:
            out_lat = self.position_tracker.lat
            out_lon = self.position_tracker.lon
            out_hAcc = self.position_tracker.hAcc
            out_gpsSol = "DEAD_RECKONING"
            out_fusionSol = "DEAD_RECKONING"
        else:
            out_lat = self.position_tracker.lat if self.position_tracker.have_position else self.constellation.center_lat
            out_lon = self.position_tracker.lon if self.position_tracker.have_position else self.constellation.center_lon
            out_hAcc = self.position_tracker.hAcc if self.position_tracker.have_position else self.constellation.center_hAcc
            out_gpsSol = self.constellation.gpsSol
            out_fusionSol = self.heading.fusionSol

        return NavFusionData(
            ts_mono=now,
            lat=out_lat,
            lon=out_lon,
            hAcc=out_hAcc,
            heading=self.heading.fused_heading.heading,
            headingAcc=self.heading.fused_heading.acc,
            speed=self.odometry.speed,
            sAcc=self.odometry.sAcc,
            gyroZ=self._gyroZ,
            gyroZAcc=self._gyroZAcc,
            gpsSol=out_gpsSol,
            headingSol=self.heading.fused_heading.sol,
            fusionSol=out_fusionSol,
            bestnav_pos_type=self.constellation.bestnav_pos_type,
            uniheading_pos_type=self.heading.uniheading_pos_type,
            pitch=self._pitch,
            roll=self._roll,
            heading_source=self.heading.heading_source,
            antenna_status=self.constellation.triangle_status
        )

    def get_diagnostics(self) -> Dict[str, Any]:
        """Komplexní diagnostická data pro rozšířený příkaz STATUS."""
        now = time.monotonic()
        return {
            "constellation": self.constellation.get_constellation_diagnostics(),
            "antennas": self.constellation.get_antennas_diagnostics(now),
            "heading_hold": self.heading.get_heading_hold_diagnostics(),
            "odometry": self.odometry.get_diagnostics(),
            "position_dr": self.position_tracker.get_diagnostics(),
            "heading_consensus": self.heading.get_consensus_diagnostics(
                self.constellation.ant_gps,
                self.constellation.ant_master,
                self.constellation.ant_slave
            ),
            "attitude": {
                "pitch": round(self._pitch, 2),
                "roll": round(self._roll, 2),
                "gyroZ": round(self._gyroZ, 2)
            }
        }