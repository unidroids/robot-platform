# fusion/core_modules/constellation.py
"""
Správa trojúhelníkové konstelace 3 antén, validace geometrie,
Fault Detection & Exclusion (FDE) a transformace polohy do středu otáčení.
"""
import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

try:
    from .geo_math import geo_dist
except (ImportError, ValueError):
    from core_modules.geo_math import geo_dist


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


class ConstellationManager:
    """
    Správa 3-anténní konstelace (Pythagorejský trojúhelník 7-24-25):
    - UM980 (gnss-gps): [+0.32, 0.0] m (předsazená anténa na ose)
    - UM982 Master (gnss-dual): [+0.25, +0.24] m (vlevo vzadu)
    - UM982 Slave (gnss-dual): [+0.25, -0.24] m (vpravo vzadu)
    - Základna Master-Slave: 0.48 m
    - Ramena GPS-Master a GPS-Slave: 0.25 m
    """

    def __init__(self):
        # 3 antény:
        self.ant_gps = AntennaState("gnss-gps", offset_x=0.32, offset_y=0.0)
        self.ant_master = AntennaState("gnss-dual-master", offset_x=0.25, offset_y=0.24)
        self.ant_slave = AntennaState("gnss-dual-slave", offset_x=0.25, offset_y=-0.24)

        # Stav geometrie a aktivní anténa
        self.triangle_status: str = "INIT"
        self.active_antenna_name: str = "NONE"
        self.dist_dual: float = 0.0
        self.dist_gps_master: float = 0.0
        self.dist_gps_slave: float = 0.0

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
        hor_spd: float = 0.0
    ) -> None:
        """Aktualizace primární antény UM980 (gnss-gps)."""
        self.ant_gps.lat = float(lat)
        self.ant_gps.lon = float(lon)
        self.ant_gps.hAcc = float(hAcc)
        self.ant_gps.pos_type = pos_type
        self.ant_gps.trk_gnd = float(trk_gnd)
        self.ant_gps.hor_spd = float(hor_spd)
        self.ant_gps.ts_mono = time.monotonic()
        self.ant_gps.valid = pos_type not in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS")

    def update_master(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0
    ) -> None:
        """Aktualizace Master antény UM982 (gnss-dual)."""
        self.ant_master.lat = float(lat)
        self.ant_master.lon = float(lon)
        self.ant_master.hAcc = float(hAcc)
        self.ant_master.pos_type = pos_type
        self.ant_master.trk_gnd = float(trk_gnd)
        self.ant_master.hor_spd = float(hor_spd)
        self.ant_master.ts_mono = time.monotonic()
        self.ant_master.valid = pos_type not in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS")

    def update_slave(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        trk_gnd: float = 0.0,
        hor_spd: float = 0.0
    ) -> None:
        """Aktualizace Slave antény UM982 (gnss-dual BESTNAVH)."""
        self.ant_slave.lat = float(lat)
        self.ant_slave.lon = float(lon)
        self.ant_slave.hAcc = float(hAcc)
        self.ant_slave.pos_type = pos_type
        self.ant_slave.trk_gnd = float(trk_gnd)
        self.ant_slave.hor_spd = float(hor_spd)
        self.ant_slave.ts_mono = time.monotonic()
        self.ant_slave.valid = pos_type not in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS")

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

    def evaluate_constellation_and_position(
        self,
        heading_deg: Optional[float] = None,
        heading_initialized: bool = False
    ) -> None:
        """
        Vyhodnotí geometrii trojúhelníku 3 antén, detekuje anomálie / odlehlé antény (FDE),
        vybere nejlepší platnou anténu a transformuje polohu na střed otáčení [0,0,0].
        Podmínka pro geometrické vyhodnocení trojúhelníku (48 cm / 25 cm):
        Antény musí mít RTK fix (NARROW_INT u BESTNAV / BESTNAVH).
        """
        now = time.monotonic()
        gps_fresh = self.ant_gps.valid and (now - self.ant_gps.ts_mono < 2.0)
        master_fresh = self.ant_master.valid and (now - self.ant_master.ts_mono < 2.0)
        slave_fresh = self.ant_slave.valid and (now - self.ant_slave.ts_mono < 2.0)

        # Pro geometrické vyhodnocení konstelace na centimetry je vyžadován NARROW_INT
        gps_rtk = gps_fresh and (self.ant_gps.pos_type in self.RTK_SOLUTIONS)
        master_rtk = master_fresh and (self.ant_master.pos_type in self.RTK_SOLUTIONS)
        slave_rtk = slave_fresh and (self.ant_slave.pos_type in self.RTK_SOLUTIONS)

        # Výpočet vzájemných vzdáleností (jen mezi RTK anténami)
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

        # Nominální vzdálenosti s tolerancí +-8 cm:
        # Základna Master-Slave: 0.48 m (rozsah 0.40 - 0.56 m)
        # Ramena GPS-Master a GPS-Slave: 0.25 m (rozsah 0.17 - 0.33 m)
        d_dual_ok = 0.40 <= self.dist_dual <= 0.56
        d_gm_ok = 0.17 <= self.dist_gps_master <= 0.33
        d_gs_ok = 0.17 <= self.dist_gps_slave <= 0.33

        gps_usable = gps_rtk
        master_usable = master_rtk
        slave_usable = slave_rtk

        if gps_rtk and master_rtk and slave_rtk:
            # Všechny 3 antény mají NARROW_INT -> plná geometrická kontrola trojúhelníku
            if d_dual_ok and d_gm_ok and d_gs_ok:
                self.triangle_status = "TRIANGLE_OK"
            elif d_dual_ok and (not d_gm_ok or not d_gs_ok):
                # Duální antény souhlasí mezi sebou, ale GPS nesouhlasí -> GPS je vadná / multipath
                gps_usable = False
                self.triangle_status = "EXCLUDED_GPS"
            elif d_gs_ok and not d_dual_ok:
                # GPS souhlasí se Slave, ale Master nesouhlasí -> Master je vadný
                master_usable = False
                self.triangle_status = "EXCLUDED_MASTER"
            elif d_gm_ok and not d_dual_ok:
                # GPS souhlasí s Master, ale Slave nesouhlasí -> Slave je vadný
                slave_usable = False
                self.triangle_status = "EXCLUDED_SLAVE"
            else:
                self.triangle_status = "TRIANGLE_DEGRADED"
        elif master_rtk and slave_rtk:
            self.triangle_status = "PAIR_DUAL_OK" if d_dual_ok else "PAIR_DUAL_MISMATCH"
        elif gps_rtk and master_rtk:
            self.triangle_status = "PAIR_GPS_MASTER_OK" if d_gm_ok else "PAIR_GPS_MASTER_MISMATCH"
        elif gps_rtk and slave_rtk:
            self.triangle_status = "PAIR_GPS_SLAVE_OK" if d_gs_ok else "PAIR_GPS_SLAVE_MISMATCH"
        elif gps_rtk:
            self.triangle_status = "SINGLE_GPS"
        elif master_rtk:
            self.triangle_status = "SINGLE_MASTER"
        elif slave_rtk:
            self.triangle_status = "SINGLE_SLAVE"
        elif gps_fresh or master_fresh or slave_fresh:
            # Máme pozici, ale antény nemají RTK fix (NARROW_INT) -> geometrii nelze ověřit
            self.triangle_status = "NO_NARROW_INT"
        else:
            self.triangle_status = "NO_VALID_FIX"

        # Výběr nejlepší aktivní antény pro výstupní polohu:
        # Priorita: 1. platná RTK anténa (GPS -> Master -> Slave)
        #           2. fallback na jakoukoli čerstvou anténu bez RTK (SINGLE/FLOAT)
        chosen_ant: Optional[AntennaState] = None
        if gps_usable:
            chosen_ant = self.ant_gps
        elif master_usable:
            chosen_ant = self.ant_master
        elif slave_usable:
            chosen_ant = self.ant_slave
        elif gps_fresh:
            chosen_ant = self.ant_gps
        elif master_fresh:
            chosen_ant = self.ant_master
        elif slave_fresh:
            chosen_ant = self.ant_slave

        if chosen_ant is not None:
            self.active_antenna_name = chosen_ant.name
            self.gpsSol = chosen_ant.pos_type
            self.bestnav_pos_type = chosen_ant.pos_type
            self.center_hAcc = chosen_ant.hAcc
            self.have_position = True

            # Transformace polohy antény na střed osy otáčení [0, 0, 0]
            if heading_initialized and heading_deg is not None:
                heading_rad = math.radians(heading_deg)
                # Převod vektoru z tělesného systému (x dopředu, y vlevo) do NED (North, East):
                d_north = chosen_ant.offset_x * math.cos(heading_rad) - chosen_ant.offset_y * math.sin(heading_rad)
                d_east = chosen_ant.offset_x * math.sin(heading_rad) + chosen_ant.offset_y * math.cos(heading_rad)

                # Odečteme předsazení antény pro získání středu otáčení
                self.center_lat = chosen_ant.lat - (d_north / 111139.0)
                lat_rad = math.radians(chosen_ant.lat)
                self.center_lon = chosen_ant.lon - (d_east / (111139.0 * math.cos(lat_rad)))
            else:
                self.center_lat = chosen_ant.lat
                self.center_lon = chosen_ant.lon
        else:
            self.active_antenna_name = "NONE"

    def get_constellation_diagnostics(self) -> Dict[str, Any]:
        """Diagnostika geometrie konstelace pro příkaz STATUS."""
        return {
            "status": self.triangle_status,
            "active_antenna": self.active_antenna_name,
            "dist_dual_m": round(self.dist_dual, 3),
            "dist_gps_master_m": round(self.dist_gps_master, 3),
            "dist_gps_slave_m": round(self.dist_gps_slave, 3),
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
