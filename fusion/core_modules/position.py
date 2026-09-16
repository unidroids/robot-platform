# fusion/core_modules/position.py
"""
Polohový Dead Reckoning Tracker (odometrická integrace polohy).
Umožňuje kontinuální navigaci i při výpadku GNSS (např. pod mostem, v tunelu či pod stromy).
"""
import math
import time
from typing import Optional, Dict, Any

try:
    from .geo_math import displace_wgs84, geo_dist
except (ImportError, ValueError):
    from core_modules.geo_math import displace_wgs84, geo_dist


class PositionDeadReckoningTracker:
    """
    Sleduje polohu robota a při výpadku GNSS RTK signálu přepíná na odometrický dead reckoning:
    - V GNSS RTK režimu kotví absolutní polohu ze středu otáčení robota.
    - Při ztrátě signálu (např. pod mostem) integruje rychlost odometrie a azimut ze fúzovaného gyra.
    - ZUPT ochrana: Při stání na místě se poloha nemění a drift neroste.
    - Nejistota hAcc roste úměrně ujeté vzdálenosti a času.
    - Plynulý návrat při re-akvizici RTK fixu.
    """

    def __init__(self):
        # Aktuální výstupní poloha
        self.lat: float = 0.0
        self.lon: float = 0.0
        self.hAcc: float = 100.0
        self.pos_type: str = "NONE"
        self.have_position: bool = False

        # Stav Dead Reckoning
        self.dr_active: bool = False
        self.dr_distance_m: float = 0.0
        self.dr_duration_s: float = 0.0
        self.dr_start_ts: Optional[float] = None
        self.fusion_phase: int = 1  # 1: FREE_DR, 2: LEASH, 3: BLEND
        self.leash_active: bool = False

        # Poslední známé GNSS hodnoty
        self.last_valid_gnss_ts: Optional[float] = None
        self.gnss_lat: float = 0.0
        self.gnss_lon: float = 0.0
        self.gnss_hAcc: float = 100.0
        self.gnss_pos_type: str = "NONE"
        self.gnss_is_rtk: bool = False

        # Časová značka odometrie
        self.last_odo_ts: Optional[float] = None

    def update_gnss(
        self,
        lat: float,
        lon: float,
        hAcc: float,
        pos_type: str,
        is_rtk: bool,
        ts_mono: Optional[float] = None
    ) -> None:
        """
        Aktualizace polohy z GNSS konstelace.
        is_rtk: True pokud má konstelace platné RTK řešení (NARROW_INT apod.)
        """
        if ts_mono is None:
            ts_mono = time.monotonic()

        if pos_type in ("NONE", "NONE_SOL", "INSUFFICIENT_OBS"):
            return

        self.gnss_lat = float(lat)
        self.gnss_lon = float(lon)
        self.gnss_hAcc = float(hAcc)
        self.gnss_pos_type = pos_type
        self.gnss_is_rtk = is_rtk
        self.last_valid_gnss_ts = ts_mono

        if is_rtk:
            if self.dr_active:
                # Návrat z Dead Reckoning na RTK
                dist_error = geo_dist(self.lat, self.lon, lat, lon)
                if dist_error < 1.5:
                    # Plynulé dotažení polohy (50 % v prvním kroku)
                    self.lat = self.lat + 0.5 * (lat - self.lat)
                    self.lon = self.lon + 0.5 * (lon - self.lon)
                else:
                    self.lat = lat
                    self.lon = lon
                self.dr_active = False
                self.dr_start_ts = None
                self.fusion_phase = 1
                self.leash_active = False
            else:
                self.lat = lat
                self.lon = lon

            self.hAcc = hAcc
            self.pos_type = pos_type
            self.have_position = True
            self.dr_distance_m = 0.0
            self.dr_duration_s = 0.0
        else:
            # Ne-RTK fix (např. SINGLE/FLOAT)
            if not self.have_position:
                # Prvotní inicializace před naskočením RTK
                self.lat = lat
                self.lon = lon
                self.hAcc = hAcc
                self.pos_type = pos_type
                self.have_position = True

    def update_odometry(
        self,
        speed_mm_s: float,
        is_stationary: bool,
        heading_deg: Optional[float],
        heading_valid: bool,
        ts_mono: Optional[float] = None
    ) -> None:
        """
        Propagace polohy pomocí odometrie a azimutu.
        Aktivní zejména při výpadku nebo degradaci GNSS (např. pod mostem).
        """
        if ts_mono is None:
            ts_mono = time.monotonic()

        dt = 0.0
        if self.last_odo_ts is not None:
            calc_dt = ts_mono - self.last_odo_ts
            if 0 < calc_dt <= 2.0:
                dt = calc_dt
        self.last_odo_ts = ts_mono

        now = ts_mono
        gnss_stale = (self.last_valid_gnss_ts is None) or (now - self.last_valid_gnss_ts > 2.0)
        gnss_lost = gnss_stale or (not self.gnss_is_rtk)

        if self.have_position and gnss_lost:
            # Aktivace / pokračování Dead Reckoning
            if not self.dr_active:
                self.dr_active = True
                self.dr_start_ts = now
                self.dr_distance_m = 0.0
                self.dr_duration_s = 0.0
                self.fusion_phase = 1
                self.leash_active = False

            self.dr_duration_s = (now - self.dr_start_ts) if self.dr_start_ts else 0.0
            self.pos_type = "DEAD_RECKONING"

            # ZUPT ochrana při stání: žádný posun ani drift
            if is_stationary or abs(speed_mm_s) < 20.0:
                return

            if heading_valid and heading_deg is not None and dt > 0:
                v_ms = speed_mm_s / 1000.0
                d_meters = v_ms * dt
                self.dr_distance_m += abs(d_meters)

                heading_rad = math.radians(heading_deg)
                d_north = d_meters * math.cos(heading_rad)
                d_east = d_meters * math.sin(heading_rad)

                self.lat, self.lon = displace_wgs84(self.lat, self.lon, d_north, d_east)

                # Nárůst nejistoty: 2.5 % z ujeté vzdálenosti + 0.005 m/s časového driftu gyra
                self.hAcc += 0.025 * abs(d_meters) + 0.005 * dt

                # Aplikace 3-fázového vodítka (Leash / Bounding) vůči degradované GNSS
                self._apply_gnss_leash(dt, now)

    def _apply_gnss_leash(self, dt: float, now: float) -> None:
        """
        3-fázová fúze Dead Reckoningu s degradovanou GNSS (SINGLE / PSRDIFF):
        - Fáze 1 (0 až 50 m / <60 s a hAcc_DR < hAcc_GPS): 100% volný DR bez zásahu GNSS.
        - Fáze 2 (50 až 120 m / <150 s): Měkké vodítko – oříznutí polohy na okraj r = 1.5 * hAcc_GPS.
        - Fáze 3 (>120 m / >=150 s): Vodítko + jemný tah ke středu GNSS (ochrana před nekonečným driftem).
        """
        self.leash_active = False

        # Ověření dostupnosti a čerstvosti ne-RTK GNSS
        if self.last_valid_gnss_ts is None or (now - self.last_valid_gnss_ts > 3.0):
            self.fusion_phase = 1
            return

        if self.gnss_lat == 0.0 or self.gnss_lon == 0.0 or self.gnss_hAcc <= 0.0:
            self.fusion_phase = 1
            return

        # Rozhodnutí o fázi
        if self.dr_distance_m < 50.0 and self.dr_duration_s < 60.0 and self.hAcc < self.gnss_hAcc:
            self.fusion_phase = 1
            return

        dist_to_gnss = geo_dist(self.lat, self.lon, self.gnss_lat, self.gnss_lon)
        r_max = 1.5 * max(self.gnss_hAcc, 1.0)

        if self.dr_distance_m < 120.0 and self.dr_duration_s < 150.0:
            # Fáze 2: Vodítko (Leash / Bounding) na hranici r = 1.5 * hAcc
            self.fusion_phase = 2
            if dist_to_gnss > r_max and dist_to_gnss > 0.001:
                ratio = r_max / dist_to_gnss
                self.lat = self.gnss_lat + ratio * (self.lat - self.gnss_lat)
                self.lon = self.gnss_lon + ratio * (self.lon - self.gnss_lon)
                self.leash_active = True
        else:
            # Fáze 3: Dlouhodobý výpadek – vodítko + jemný komplementární filtr
            self.fusion_phase = 3
            if dist_to_gnss > r_max and dist_to_gnss > 0.001:
                ratio = r_max / dist_to_gnss
                self.lat = self.gnss_lat + ratio * (self.lat - self.gnss_lat)
                self.lon = self.gnss_lon + ratio * (self.lon - self.gnss_lon)
                self.leash_active = True

            # Jemný posun ke středu GNSS (cca 2 % za sekundu)
            blend_factor = min(0.05, 0.02 * dt)
            self.lat += blend_factor * (self.gnss_lat - self.lat)
            self.lon += blend_factor * (self.gnss_lon - self.lon)

    def get_diagnostics(self) -> Dict[str, Any]:
        """Diagnostika pro rozšířený příkaz STATUS."""
        if self.dr_active:
            mode = "DEAD_RECKONING"
        elif self.gnss_is_rtk:
            mode = "GNSS_RTK"
        elif self.have_position:
            mode = "GNSS_NON_RTK"
        else:
            mode = "NONE"

        return {
            "mode": mode,
            "dr_active": self.dr_active,
            "dr_distance_m": round(self.dr_distance_m, 2),
            "dr_duration_s": round(self.dr_duration_s, 2),
            "hAcc": round(self.hAcc, 3),
            "fusion_phase": self.fusion_phase,
            "leash_active": self.leash_active,
            "have_position": self.have_position
        }

