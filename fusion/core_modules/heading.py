# fusion/core_modules/heading.py
"""
Správa kurzu a držení severu:
- UNIHEADING NARROW_INT z duální antény UM982 s komplementárním vyhlazováním.
- 3-anténní kinematický konsenzus kurzu s kompenzací tečných úhlů v zatáčkách.
- Záložní 3-vzorkový kurzový zámek z BESTNAV při přímé jízdě.
- Integrace úhlu z gyroskopu se ZUPT ochranou proti driftu při stání.
"""
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, Any, List

try:
    from .geo_math import norm_deg, angle_diff, circular_mean, compute_tangent_offset
    from .constellation import AntennaState
except (ImportError, ValueError):
    from core_modules.geo_math import norm_deg, angle_diff, circular_mean, compute_tangent_offset
    from core_modules.constellation import AntennaState


@dataclass
class HeadingData:
    """Stav úhlu natočení a odhadu jeho přesnosti."""
    heading: float = 0.0
    acc: float = 180.0
    sol: str = "NONE"


class HeadingManager:
    """
    Manažer výpočtu a fúze azimutu/severu.
    """

    def __init__(self):
        self.gps_heading = HeadingData()
        self.dual_heading = HeadingData()
        self.fused_heading = HeadingData()

        self.heading_source: str = "NONE"
        self.heading_initialized: bool = False
        self.fusionSol: str = "NONE"
        self.uniheading_pos_type: str = "NONE"

        # Buffer pro 3 po sobě jdoucí vzorky kurzu z BESTNAV (+-3°)
        self.bestnav_course_buffer: deque = deque(maxlen=3)

        # Stav konsenzu ze 3 antén
        self.heading_consensus_status: str = "INIT"

    def update_dual_heading(
        self,
        heading: float,
        headingAcc: float,
        headingSol: str,
        length: float = 0.0
    ) -> None:
        """
        Heading z duální antény UM982 (UNIHEADING).
        Při pos_type == NARROW_INT a acc < 1.5° se provádí prvotní inicializace nebo
        plynulé vyhlazení (alpha=0.1) pro zabránění skoků.
        """
        self.dual_heading.heading = norm_deg(heading)
        self.dual_heading.acc = float(headingAcc)
        self.dual_heading.sol = headingSol
        self.uniheading_pos_type = headingSol

        if headingSol in ("NARROW_INT", "NAROW_INT") and self.dual_heading.acc < 1.5:
            if not self.heading_initialized:
                # První inicializace
                self.heading_initialized = True
                self.fused_heading.heading = self.dual_heading.heading
                self.fused_heading.acc = self.dual_heading.acc
                self.fused_heading.sol = self.dual_heading.sol
                self.heading_source = "UNIHEADING"
                self.fusionSol = "UNIHEADING"
            else:
                # Plynulé navazování (komplementární vyhlazování s alpha = 0.1)
                diff = angle_diff(self.dual_heading.heading, self.fused_heading.heading)
                self.fused_heading.heading = norm_deg(self.fused_heading.heading + 0.1 * diff)
                self.fused_heading.acc = self.dual_heading.acc
                self.fused_heading.sol = self.dual_heading.sol
                self.heading_source = "UNIHEADING"
                self.fusionSol = "UNIHEADING"
        else:
            # UNIHEADING ztratil NARROW_INT
            if self.heading_source == "UNIHEADING":
                self.heading_source = "GYRO"
                self.fused_heading.sol = "GYRO"
                self.fusionSol = "GYRO"
                self.fused_heading.acc = max(self.fused_heading.acc, 4.0)

    def update_single_course(
        self,
        heading: float,
        headingAcc: float,
        headingSol: str,
        hor_spd: float,
        gyroZ: float,
        is_stationary: bool
    ) -> None:
        """
        Kurz z pohybu (BESTNAV trk_gnd).
        Podmínka: v > 0.3 m/s, 3 po sobě jdoucí měření se shodují v rozmezí +-3° a robot nezatáčí (|wz| < 3°/s).
        """
        norm_hdg = norm_deg(heading)
        self.gps_heading.heading = norm_hdg
        self.gps_heading.acc = float(headingAcc)
        self.gps_heading.sol = headingSol

        # Podmínky pro kurzový zámek:
        # 1. Rychlost vůči zemi z BESTNAV (hor_spd) > 0.3 m/s
        # 2. Robot jede rovně (|gyroZ| < 3.0°/s)
        # 3. Robot nestojí (dle odometrických kroků ZUPT)
        is_moving_straight = (hor_spd > 0.3) and (abs(gyroZ) < 3.0) and (not is_stationary)

        if is_moving_straight and headingSol not in ("NONE", "NONE_SOL"):
            self.bestnav_course_buffer.append(norm_hdg)

            if len(self.bestnav_course_buffer) == 3:
                h1, h2, h3 = self.bestnav_course_buffer
                diff12 = abs(angle_diff(h1, h2))
                diff23 = abs(angle_diff(h2, h3))
                diff13 = abs(angle_diff(h1, h3))

                if max(diff12, diff23, diff13) <= 3.0:
                    avg_course = circular_mean(list(self.bestnav_course_buffer))

                    if not self.heading_initialized:
                        self.heading_initialized = True
                        self.fused_heading.heading = avg_course
                        self.fused_heading.acc = 3.0
                        self.fused_heading.sol = "BESTNAV_COURSE"
                        self.heading_source = "BESTNAV_COURSE"
                        self.fusionSol = "BESTNAV_COURSE"
                    elif self.heading_source != "UNIHEADING":
                        # Pokud není k dispozici UNIHEADING, korigujeme drift gyra
                        diff = angle_diff(avg_course, self.fused_heading.heading)
                        self.fused_heading.heading = norm_deg(self.fused_heading.heading + 0.05 * diff)
                        self.fused_heading.acc = 3.0
                        self.fused_heading.sol = "BESTNAV_COURSE"
                        self.heading_source = "BESTNAV_COURSE"
                        self.fusionSol = "BESTNAV_COURSE"
        else:
            if not is_moving_straight:
                self.bestnav_course_buffer.clear()

    def evaluate_consensus(
        self,
        ant_gps: AntennaState,
        ant_master: AntennaState,
        ant_slave: AntennaState,
        speed: float,
        gyroZ: float,
        is_stationary: bool
    ) -> None:
        """
        Kinematická kompenzace tečného natočení antén v zatáčce a 3-anténní konsenzus kurzu.
        Podmínky:
        - v >= 0.3 m/s
        - Mírná rotace: |gyroZ| <= 10.0 deg/s
        - Robot nestojí na místě
        """
        now = time.monotonic()

        v_odo = (speed / 1000.0) if speed > 10.0 else speed
        v_gnss = max(ant_gps.hor_spd, ant_master.hor_spd, ant_slave.hor_spd)
        v_ms = max(v_odo, v_gnss)

        if is_stationary or v_ms < 0.3:
            self.heading_consensus_status = "LOW_SPEED"
            return

        if abs(gyroZ) > 10.0:
            self.heading_consensus_status = "EXCESSIVE_ROTATION"
            return

        fresh_antennas: List[AntennaState] = []
        for ant in [ant_gps, ant_master, ant_slave]:
            if ant.valid and (now - ant.ts_mono < 2.0) and (ant.hor_spd >= 0.25 or v_ms >= 0.3):
                beta = compute_tangent_offset(ant.offset_x, ant.offset_y, v_ms, gyroZ)
                ant.tangent_offset_deg = beta
                ant.compensated_heading = norm_deg(ant.trk_gnd - beta)
                fresh_antennas.append(ant)
            else:
                ant.tangent_offset_deg = 0.0
                ant.compensated_heading = 0.0

        if len(fresh_antennas) < 2:
            self.heading_consensus_status = "INSUFFICIENT_ANTENNAS"
            return

        TOLERANCE_DEG = 3.5

        if len(fresh_antennas) == 3:
            h_gps = ant_gps.compensated_heading
            h_mst = ant_master.compensated_heading
            h_slv = ant_slave.compensated_heading

            d_gm = abs(angle_diff(h_gps, h_mst))
            d_gs = abs(angle_diff(h_gps, h_slv))
            d_ms = abs(angle_diff(h_mst, h_slv))

            if max(d_gm, d_gs, d_ms) <= TOLERANCE_DEG:
                self.heading_consensus_status = "3_ANTENNAS_AGREE"
                consensus_heading = circular_mean([h_gps, h_mst, h_slv])
                self.apply_consensus_heading(consensus_heading, acc=2.0)
            elif d_ms <= TOLERANCE_DEG and d_gm > TOLERANCE_DEG and d_gs > TOLERANCE_DEG:
                # Master a Slave souhlasí, GPS je odlehlá (multipath)
                self.heading_consensus_status = "2_AGREE_GPS_OUTLIER"
                consensus_heading = circular_mean([h_mst, h_slv])
                self.apply_consensus_heading(consensus_heading, acc=2.5)
            elif d_gm <= TOLERANCE_DEG and d_ms > TOLERANCE_DEG:
                # GPS a Master souhlasí, Slave je odlehlá
                self.heading_consensus_status = "2_AGREE_SLAVE_OUTLIER"
                consensus_heading = circular_mean([h_gps, h_mst])
                self.apply_consensus_heading(consensus_heading, acc=2.5)
            elif d_gs <= TOLERANCE_DEG and d_ms > TOLERANCE_DEG:
                # GPS a Slave souhlasí, Master je odlehlá
                self.heading_consensus_status = "2_AGREE_MASTER_OUTLIER"
                consensus_heading = circular_mean([h_gps, h_slv])
                self.apply_consensus_heading(consensus_heading, acc=2.5)
            else:
                self.heading_consensus_status = "NO_CONSENSUS"
        elif len(fresh_antennas) == 2:
            a1, a2 = fresh_antennas
            d = abs(angle_diff(a1.compensated_heading, a2.compensated_heading))
            if d <= TOLERANCE_DEG:
                self.heading_consensus_status = f"PAIR_AGREE_{a1.name}_{a2.name}"
                consensus_heading = circular_mean([a1.compensated_heading, a2.compensated_heading])
                self.apply_consensus_heading(consensus_heading, acc=3.0)
            else:
                self.heading_consensus_status = "PAIR_MISMATCH"

    def apply_consensus_heading(self, heading: float, acc: float) -> None:
        """Aplikuje konsenzuální sever na stav fúze."""
        if not self.heading_initialized:
            self.heading_initialized = True
            self.fused_heading.heading = heading
            self.fused_heading.acc = acc
            self.fused_heading.sol = "BESTNAV_CONSENSUS"
            self.heading_source = "BESTNAV_CONSENSUS"
            self.fusionSol = "BESTNAV_CONSENSUS"
        elif self.heading_source != "UNIHEADING":
            # Pokud neběží NARROW_INT z duální antény, korigujeme drift gyra
            diff = angle_diff(heading, self.fused_heading.heading)
            self.fused_heading.heading = norm_deg(self.fused_heading.heading + 0.05 * diff)
            self.fused_heading.acc = acc
            self.fused_heading.sol = "BESTNAV_CONSENSUS"
            self.heading_source = "BESTNAV_CONSENSUS"
            self.fusionSol = "BESTNAV_CONSENSUS"

    def update_imu(self, delta_yaw: float, wz: float, is_stationary: bool) -> None:
        """
        Zpracování integrace úhlu z 20Hz IMU zprávy se ZUPT ochranou proti driftu.
        """
        # Zero Velocity Update (ZUPT):
        # Pokud robot stojí (kroky kol se nemění) a |wz| < 1.5°/s,
        # robot s diferenciálním podvozkem se po zemi nemůže otáčet.
        # Přírůstek delta_yaw je v klidu čistý šum/drift gyra a ignoruje se.
        if is_stationary and abs(wz) < 1.5:
            effective_delta_yaw = 0.0
        else:
            effective_delta_yaw = delta_yaw

        # Numerická integrace úhlu (dead reckoning)
        if self.heading_initialized:
            self.fused_heading.heading = norm_deg(self.fused_heading.heading + effective_delta_yaw)

            # Pokud neudržujeme UNIHEADING ani BESTNAV_COURSE, zdroj je GYRO
            if self.heading_source not in ("UNIHEADING", "BESTNAV_COURSE"):
                self.heading_source = "GYRO"
                self.fused_heading.sol = "GYRO"
                self.fusionSol = "GYRO"
                # Přesnost mírně degraduje v čase, jen pokud se robot pohybuje
                if not is_stationary:
                    self.fused_heading.acc = min(15.0, self.fused_heading.acc + 0.05)

    def check_stale(self, now: float, last_msg_mono_ts: Optional[float], timeout: float = 2.0) -> None:
        """Při výpadku zpráv degraduje navigační řešení."""
        if last_msg_mono_ts is not None and (now - last_msg_mono_ts) > timeout:
            self.fused_heading.sol = "NONE"
            self.fused_heading.acc = 180.0
            self.fusionSol = "NONE"

    def get_heading_hold_diagnostics(self) -> Dict[str, Any]:
        """Diagnostika držení severu pro příkaz STATUS."""
        return {
            "source": self.heading_source,
            "heading": round(self.fused_heading.heading, 2),
            "acc": round(self.fused_heading.acc, 2),
            "uniheading_sol": self.dual_heading.sol,
            "course_buffer_len": len(self.bestnav_course_buffer),
        }

    def get_consensus_diagnostics(
        self,
        ant_gps: AntennaState,
        ant_master: AntennaState,
        ant_slave: AntennaState
    ) -> Dict[str, Any]:
        """Diagnostika 3-anténního konsenzu a tečných úhlů pro příkaz STATUS."""
        return {
            "status": self.heading_consensus_status,
            "tangent_offsets": {
                "gps": round(ant_gps.tangent_offset_deg, 2),
                "master": round(ant_master.tangent_offset_deg, 2),
                "slave": round(ant_slave.tangent_offset_deg, 2),
            },
            "compensated_headings": {
                "gps": round(ant_gps.compensated_heading, 2),
                "master": round(ant_master.compensated_heading, 2),
                "slave": round(ant_slave.compensated_heading, 2),
            }
        }
