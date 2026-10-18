# fusion/core_modules/constellation.py
"""
Správa trojúhelníkové konstelace 3 antén, validace geometrie a orientace základny,
Fault Detection & Exclusion (FDE) a transformace polohy do středu otáčení robota [0, 0, 0].
"""
import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple, List

try:
    from .geo_math import geo_dist, angle_diff, wrap180
except (ImportError, ValueError):
    from core_modules.geo_math import geo_dist, angle_diff, wrap180


@dataclass
class AntennaState:
    """Stav a parametry jednotlivé GNSS antény."""
    name: str
    lat: float = 0.0
    lon: float = 0.0
    hAcc: float = 0.0
    pos_type: str = "NONE"
    ts_mono: float = 0.0
    offset_x: float = 0.0  # body x (m, dopředu od středu otáčení)
    offset_y: float = 0.0  # body y (m, vlevo od středu otáčení)
    valid: bool = False
    trk_gnd: float = 0.0   # kurz vůči zemi ve stupních
    hor_spd: float = 0.0   # horizontální rychlost v m/s
    compensated_heading: float = 0.0  # kurz po kinematické kompenzaci tečného úhlu
    tangent_offset_deg: float = 0.0   # vypočtený tečný úhel beta ve stupních
    wn: Optional[int] = None          # GPS týden
    ms: Optional[int] = None          # GPS milisekundy v týdnu


class ConstellationManager:
    """
    Správa 3-anténní konstelace (kalibrovaná geometrie Robotour 2026):
    - UM980 (gnss-gps): [+0.3200, -0.0059] m (předsazená anténa na ose)
    - UM982 Master (gnss-dual): [+0.2617, +0.2341] m (vlevo vzadu)
    - UM982 Slave (gnss-dual): [+0.2617, -0.2341] m (vpravo vzadu)
    - Základna Master-Slave: 0.4682 m (fyzické měření a interferometrie UNIHEADING)
    - Ramena GPS-Master: 0.2470 m, GPS-Slave: 0.2356 m
    - Kalibrovaný montážní offset základny: +1.11°
    """

    # Kalibrované nominální rozměry trojúhelníku
    NOMINAL_D_MS = 0.4682
    NOMINAL_D_GM = 0.2470
    NOMINAL_D_GS = 0.2356
    CALIB_SKEW_DEG = 1.11

    # Dvoustupňové toleranční meze
    TOL_3ANT_M = 0.080          # +- 8.0 cm pro plný trojúhelník (3 antény)
    TOL_2ANT_M = 0.050          # +- 5.0 cm pro 2 antény (eliminace driftu bez křížové triangulace)
    TOL_ANG_DEG_STRAIGHT = 8.0  # +- 8.0° při |wz| <= 10°/s
    TOL_ANG_DEG_TURN = 15.0     # +- 15.0° při |wz| > 10°/s

    # Váhové rozdělení pro 2 antény (GPS má 70 % váhu díky geodetické dish anténě)
    WEIGHT_GPS_2ANT = 0.70

    # Celočíselné RTK stavy dle tabulky 7-172 (Position or Velocity Type) Unicore N4 manuálu
    RTK_SOLUTIONS = (
        "NARROW_INT",    # 50: Integer narrow-lane ambiguity solution (plný RTK fix)
        "NAROW_INT",     # tolerance pro překlep
        "WIDE_INT",      # 49: Integer wide-lane ambiguity solution
        "L1_INT",        # 48: Integer L1 ambiguity solution
        "INS_RTKFIXED",  # 56: INS RTK fixed ambiguities solution
        "FIXED",         # generický alias
        "RTK_FIXED",
    )

    def __init__(self):
        # 3 antény s kalibrovanými tělesnými offsety:
        self.ant_gps = AntennaState("gnss-gps", offset_x=0.3200, offset_y=-0.0059)
        self.ant_master = AntennaState("gnss-dual-master", offset_x=0.2617, offset_y=0.2341)
        self.ant_slave = AntennaState("gnss-dual-slave", offset_x=0.2617, offset_y=-0.2341)

        # Stav geometrie a aktivní antény
        self.triangle_status: str = "INIT"
        self.active_antenna_name: str = "NONE"
        self.used_antennas: List[str] = []
        self._target_antenna_name: str = "NONE"
        self._switch_counter: int = 0
        self._curr_offset_x: float = 0.3200
        self._curr_offset_y: float = -0.0059

        # Měřené vzdálenosti ramen a úhlová odchylka
        self.dist_dual: float = 0.0
        self.dist_gps_master: float = 0.0
        self.dist_gps_slave: float = 0.0
        self.baseline_skew_deg: float = 0.0
        self.current_epoch_key: Optional[Tuple[int, int]] = None

        # Výsledná poloha transformovaná na střed otáčení [0,0,0]
        self.center_lat: float = 0.0
        self.center_lon: float = 0.0
        self.center_hAcc: float = 0.0
        self.gpsSol: str = "NONE"
        self.bestnav_pos_type: str = "NONE"
        self.have_position: bool = False

    def update_gps(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0,
        wn: Optional[int] = None,
        ms: Optional[int] = None
    ) -> None:
        """Aktualizace primární antény UM980 (gnss-gps)."""
        self.ant_gps.lat = float(lat)
        self.ant_gps.lon = float(lon)
        self.ant_gps.hAcc = float(hAcc)
        self.ant_gps.pos_type = pos_type
        self.ant_gps.trk_gnd = float(trk_gnd)
        self.ant_gps.hor_spd = float(hor_spd)
        self.ant_gps.wn = wn
        self.ant_gps.ms = ms
        self.ant_gps.ts_mono = time.monotonic()
        self.ant_gps.valid = pos_type not in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS")

    def update_master(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0,
        wn: Optional[int] = None,
        ms: Optional[int] = None
    ) -> None:
        """Aktualizace Master antény UM982 (gnss-dual)."""
        self.ant_master.lat = float(lat)
        self.ant_master.lon = float(lon)
        self.ant_master.hAcc = float(hAcc)
        self.ant_master.pos_type = pos_type
        self.ant_master.trk_gnd = float(trk_gnd)
        self.ant_master.hor_spd = float(hor_spd)
        self.ant_master.wn = wn
        self.ant_master.ms = ms
        self.ant_master.ts_mono = time.monotonic()
        self.ant_master.valid = pos_type not in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS")

    def update_slave(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0,
        wn: Optional[int] = None,
        ms: Optional[int] = None
    ) -> None:
        """Aktualizace Slave antény UM982 (gnss-dual BESTNAVH)."""
        self.ant_slave.lat = float(lat)
        self.ant_slave.lon = float(lon)
        self.ant_slave.hAcc = float(hAcc)
        self.ant_slave.pos_type = pos_type
        self.ant_slave.trk_gnd = float(trk_gnd)
        self.ant_slave.hor_spd = float(hor_spd)
        self.ant_slave.wn = wn
        self.ant_slave.ms = ms
        self.ant_slave.ts_mono = time.monotonic()
        self.ant_slave.valid = pos_type not in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS")

    def update_epoch(
        self,
        epoch_msgs: Dict[str, Any],
        heading_deg: Optional[float] = None,
        heading_initialized: bool = False,
        wz_deg_s: float = 0.0,
        now_mono: Optional[float] = None
    ) -> None:
        """
        Aktualizuje stav antén ze synchronizované epochy (wn, ms) a spustí geometrické vyhodnocení.
        Pokud některá anténa v epoše chybí, je označena jako neplatná pro tuto epochu (žádné míchání).
        """
        if now_mono is None:
            now_mono = time.monotonic()

        ant_configs = [
            ("gps", self.ant_gps, self.update_gps),
            ("master", self.ant_master, self.update_master),
            ("slave", self.ant_slave, self.update_slave)
        ]

        found_wn = None
        found_ms = None

        for key, ant, updater in ant_configs:
            if key in epoch_msgs and epoch_msgs[key]:
                d = epoch_msgs[key]
                wn = d.get("wn")
                ms = d.get("ms")
                if wn is not None:
                    found_wn = wn
                if ms is not None:
                    found_ms = ms

                updater(
                    lat=d["lat"],
                    lon=d["lon"],
                    hAcc=d["hAcc"],
                    pos_type=d["pos_type"],
                    trk_gnd=d.get("trk_gnd", 0.0),
                    hor_spd=d.get("hor_spd", 0.0),
                    wn=wn,
                    ms=ms
                )
                ant.ts_mono = now_mono
            else:
                # Anténa v této epoše nedorazila -> neplatná
                ant.valid = False
                ant.pos_type = "NONE"
                ant.ts_mono = 0.0

        if found_wn is not None and found_ms is not None:
            self.current_epoch_key = (found_wn, found_ms)

        self.evaluate_constellation_and_position(
            heading_deg=heading_deg,
            heading_initialized=heading_initialized,
            wz_deg_s=wz_deg_s,
            now=now_mono
        )

    def evaluate_constellation_and_position(
        self,
        heading_deg: Optional[float] = None,
        heading_initialized: bool = False,
        wz_deg_s: float = 0.0,
        now: Optional[float] = None
    ) -> None:
        """
        Vyhodnotí geometrii trojúhelníku 3 antén a orientaci základny Master->Slave,
        aplikuje Fault Detection & Exclusion (FDE) s dvoustupňovou tolerancí (8cm / 5cm),
        a transformuje výsledný váhovaný centroid na střed otáčení robota [0, 0, 0].
        Striktní podmínka integrity: vyžaduje konsenzus minimálně 2 antén.
        """
        if now is None:
            now = time.monotonic()

        gps_fresh = self.ant_gps.valid and (now - self.ant_gps.ts_mono < 2.0)
        master_fresh = self.ant_master.valid and (now - self.ant_master.ts_mono < 2.0)
        slave_fresh = self.ant_slave.valid and (now - self.ant_slave.ts_mono < 2.0)

        gps_rtk = gps_fresh and (self.ant_gps.pos_type in self.RTK_SOLUTIONS)
        master_rtk = master_fresh and (self.ant_master.pos_type in self.RTK_SOLUTIONS)
        slave_rtk = slave_fresh and (self.ant_slave.pos_type in self.RTK_SOLUTIONS)

        # 1. Výpočet vzájemných vzdáleností v metrech
        if master_rtk and slave_rtk:
            self.dist_dual = geo_dist(
                self.ant_master.lat, self.ant_master.lon,
                self.ant_slave.lat, self.ant_slave.lon
            )
        else:
            self.dist_dual = 0.0

        if gps_rtk and master_rtk:
            self.dist_gps_master = geo_dist(
                self.ant_gps.lat, self.ant_gps.lon,
                self.ant_master.lat, self.ant_master.lon
            )
        else:
            self.dist_gps_master = 0.0

        if gps_rtk and slave_rtk:
            self.dist_gps_slave = geo_dist(
                self.ant_gps.lat, self.ant_gps.lon,
                self.ant_slave.lat, self.ant_slave.lon
            )
        else:
            self.dist_gps_slave = 0.0

        # 2. Kontrola úhlu natočení základny Master -> Slave vůči fúzovanému azimutu robota
        az_ms_ok = True
        self.baseline_skew_deg = 0.0
        if master_rtk and slave_rtk:
            m_lat = 111139.0
            lat_avg = (self.ant_master.lat + self.ant_slave.lat) / 2.0
            m_lon = 111139.0 * math.cos(math.radians(lat_avg))
            de = (self.ant_slave.lon - self.ant_master.lon) * m_lon
            dn = (self.ant_slave.lat - self.ant_master.lat) * m_lat

            az_ms = (math.degrees(math.atan2(de, dn))) % 360.0
            if heading_initialized and heading_deg is not None:
                # Master je vlevo, Slave vpravo -> spojnice směřuje pod úhlem heading + 90° (+ kalibrovaný offset 1.11°)
                expected_az = (heading_deg + 90.0 + self.CALIB_SKEW_DEG) % 360.0
                self.baseline_skew_deg = angle_diff(az_ms, expected_az)
                tol_ang = self.TOL_ANG_DEG_TURN if abs(wz_deg_s) > 10.0 else self.TOL_ANG_DEG_STRAIGHT
                az_ms_ok = abs(self.baseline_skew_deg) <= tol_ang

        # 3. Odchylky délek ramen od kalibrovaných rozměrů
        err_ms = abs(self.dist_dual - self.NOMINAL_D_MS)
        err_gm = abs(self.dist_gps_master - self.NOMINAL_D_GM)
        err_gs = abs(self.dist_gps_slave - self.NOMINAL_D_GS)

        # Kritérium 3 antén (tolerance +- 8.0 cm + úhel základny)
        p3_ms = (err_ms <= self.TOL_3ANT_M) and az_ms_ok
        p3_gm = (err_gm <= self.TOL_3ANT_M)
        p3_gs = (err_gs <= self.TOL_3ANT_M)

        # Kritérium 2 antén (přísnější tolerance +- 5.0 cm + úhel základny pro MS)
        p2_ms = (err_ms <= self.TOL_2ANT_M) and az_ms_ok
        p2_gm = (err_gm <= self.TOL_2ANT_M)
        p2_gs = (err_gs <= self.TOL_2ANT_M)

        used_antennas: List[str] = []

        # 4. FDE Klasifikace konstelace
        if gps_rtk and master_rtk and slave_rtk:
            if p3_ms and p3_gm and p3_gs:
                self.triangle_status = "TRIANGLE_OK"
                used_antennas = ["gnss-gps", "gnss-dual-master", "gnss-dual-slave"]
            elif p2_ms and (not p3_gm or not p3_gs):
                # GPS odskočila (multipath)
                self.triangle_status = "EXCLUDED_GPS"
                used_antennas = ["gnss-dual-master", "gnss-dual-slave"]
            elif p2_gm and not p3_ms:
                # Slave odskočila
                self.triangle_status = "EXCLUDED_SLAVE"
                used_antennas = ["gnss-gps", "gnss-dual-master"]
            elif p2_gs and not p3_ms:
                # Master odskočila
                self.triangle_status = "EXCLUDED_MASTER"
                used_antennas = ["gnss-gps", "gnss-dual-slave"]
            else:
                self.triangle_status = "TRIANGLE_DEGRADED"
        elif master_rtk and slave_rtk and p2_ms:
            self.triangle_status = "EXCLUDED_GPS"
            used_antennas = ["gnss-dual-master", "gnss-dual-slave"]
        elif gps_rtk and master_rtk and p2_gm:
            self.triangle_status = "EXCLUDED_SLAVE"
            used_antennas = ["gnss-gps", "gnss-dual-master"]
        elif gps_rtk and slave_rtk and p2_gs:
            self.triangle_status = "EXCLUDED_MASTER"
            used_antennas = ["gnss-gps", "gnss-dual-slave"]
        elif gps_rtk or master_rtk or slave_rtk:
            # Pouze 1 anténa s RTK fixem nebo geometrie selhala -> DEGRADED (vyžadujeme konsenzus min. 2 antén)
            self.triangle_status = "TRIANGLE_DEGRADED"
            used_antennas = []
        elif gps_fresh or master_fresh or slave_fresh:
            # Máme pozici, ale antény nemají RTK fix (NARROW_INT) -> geometrii nelze ověřit
            self.triangle_status = "NO_NARROW_INT"
            used_antennas = []
        else:
            self.triangle_status = "NO_VALID_FIX"
            used_antennas = []

        self.used_antennas = used_antennas

        # 5. Transformace na střed otáčení robota [0, 0, 0] a váhovaný centroid
        def project_to_center(ant: AntennaState) -> Tuple[float, float]:
            if heading_initialized and heading_deg is not None:
                h_rad = math.radians(heading_deg)
                # Tělesný vektor (x dopředu, y vlevo) do NED (North, East):
                d_north = ant.offset_x * math.cos(h_rad) + ant.offset_y * math.sin(h_rad)
                d_east = ant.offset_x * math.sin(h_rad) - ant.offset_y * math.cos(h_rad)
                c_lat = ant.lat - (d_north / 111139.0)
                c_lon = ant.lon - (d_east / (111139.0 * math.cos(math.radians(ant.lat))))
                return c_lat, c_lon
            else:
                return ant.lat, ant.lon

        if not used_antennas:
            # Žádná anténa nesplňuje geometrický RTK konsenzus
            # Pokud je přítomna jakákoli čerstvá anténa bez RTK, zachováme hrubou polohu pro telemetrii
            fallback_ant = None
            if gps_fresh:
                fallback_ant = self.ant_gps
            elif master_fresh:
                fallback_ant = self.ant_master
            elif slave_fresh:
                fallback_ant = self.ant_slave

            if fallback_ant is not None:
                self.active_antenna_name = fallback_ant.name
                self.center_lat, self.center_lon = project_to_center(fallback_ant)
                self.center_hAcc = fallback_ant.hAcc
                self.gpsSol = fallback_ant.pos_type
                self.bestnav_pos_type = fallback_ant.pos_type
                self.have_position = True
            else:
                self.have_position = False
                self.active_antenna_name = "NONE"
                self.gpsSol = "NONE"
                self.bestnav_pos_type = "NONE"
            return

        if len(used_antennas) == 3:
            # Všechny 3 antény: rovnoměrný centroid (1/3 každá)
            cg_lat, cg_lon = project_to_center(self.ant_gps)
            cm_lat, cm_lon = project_to_center(self.ant_master)
            cs_lat, cs_lon = project_to_center(self.ant_slave)
            self.center_lat = (cg_lat + cm_lat + cs_lat) / 3.0
            self.center_lon = (cg_lon + cm_lon + cs_lon) / 3.0
            self.center_hAcc = (self.ant_gps.hAcc + self.ant_master.hAcc + self.ant_slave.hAcc) / 3.0
            self.active_antenna_name = "gnss-gps"
            self.gpsSol = "NARROW_INT"
            self.bestnav_pos_type = "NARROW_INT"
            self.have_position = True

        elif "gnss-gps" in used_antennas:
            # Stav 2 antén s GPS: 70 % váha pro UM980 geodetickou anténu, 30 % pro duální anténu
            cg_lat, cg_lon = project_to_center(self.ant_gps)
            dual_ant = self.ant_master if "gnss-dual-master" in used_antennas else self.ant_slave
            cd_lat, cd_lon = project_to_center(dual_ant)
            w_gps = self.WEIGHT_GPS_2ANT
            w_dual = 1.0 - w_gps
            self.center_lat = w_gps * cg_lat + w_dual * cd_lat
            self.center_lon = w_gps * cg_lon + w_dual * cd_lon
            self.center_hAcc = w_gps * self.ant_gps.hAcc + w_dual * dual_ant.hAcc
            self.active_antenna_name = "gnss-gps"
            self.gpsSol = "NARROW_INT"
            self.bestnav_pos_type = "NARROW_INT"
            self.have_position = True

        elif "gnss-dual-master" in used_antennas and "gnss-dual-slave" in used_antennas:
            # Stav 2 antén bez GPS (Master + Slave): 50 % Master, 50 % Slave
            cm_lat, cm_lon = project_to_center(self.ant_master)
            cs_lat, cs_lon = project_to_center(self.ant_slave)
            self.center_lat = 0.5 * cm_lat + 0.5 * cs_lat
            self.center_lon = 0.5 * cm_lon + 0.5 * cs_lon
            self.center_hAcc = 0.5 * self.ant_master.hAcc + 0.5 * self.ant_slave.hAcc
            self.active_antenna_name = "gnss-dual-master"
            self.gpsSol = "NARROW_INT"
            self.bestnav_pos_type = "NARROW_INT"
            self.have_position = True

    def get_constellation_diagnostics(self) -> Dict[str, Any]:
        """Diagnostika geometrie konstelace pro příkaz STATUS."""
        return {
            "status": self.triangle_status,
            "active_antenna": self.active_antenna_name,
            "used_antennas": list(self.used_antennas),
            "dist_dual_m": round(self.dist_dual, 3),
            "dist_gps_master_m": round(self.dist_gps_master, 3),
            "dist_gps_slave_m": round(self.dist_gps_slave, 3),
            "baseline_skew_deg": round(self.baseline_skew_deg, 2),
            "sync_epoch": list(self.current_epoch_key) if self.current_epoch_key else None
        }

    def get_antennas_diagnostics(self, now: Optional[float] = None) -> Dict[str, Any]:
        """Diagnostika jednotlivých antén pro příkaz STATUS."""
        if now is None:
            now = time.monotonic()
        return {
            "gps": {
                "valid": self.ant_gps.valid,
                "pos_type": self.ant_gps.pos_type,
                "hAcc": round(self.ant_gps.hAcc, 3),
                "age_s": round(now - self.ant_gps.ts_mono, 2) if self.ant_gps.ts_mono > 0 else None
            },
            "master": {
                "valid": self.ant_master.valid,
                "pos_type": self.ant_master.pos_type,
                "hAcc": round(self.ant_master.hAcc, 3),
                "age_s": round(now - self.ant_master.ts_mono, 2) if self.ant_master.ts_mono > 0 else None
            },
            "slave": {
                "valid": self.ant_slave.valid,
                "pos_type": self.ant_slave.pos_type,
                "hAcc": round(self.ant_slave.hAcc, 3),
                "age_s": round(now - self.ant_slave.ts_mono, 2) if self.ant_slave.ts_mono > 0 else None
            }
        }
