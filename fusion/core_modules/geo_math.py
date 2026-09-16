# fusion/core_modules/geo_math.py
"""
Matematické, geodetické a kinematické funkce pro fúzi senzorických dat.
Bezstavové čisté funkce bez vedlejších efektů.
"""
import math
from typing import List, Tuple


def norm_deg(a: float) -> float:
    """Normalizace úhlu do rozsahu [0, 360)."""
    a = a % 360.0
    if a < 0.0:
        a += 360.0
    return a


def angle_diff(target: float, current: float) -> float:
    """Rozdíl úhlů target - current v rozsahu [-180, +180]."""
    diff = (target - current + 180.0) % 360.0 - 180.0
    return diff


def circular_mean(angles: List[float]) -> float:
    """Kruhový průměr úhlů ve stupních s korektním ošetřením periody 360°."""
    if not angles:
        return 0.0
    sin_sum = sum(math.sin(math.radians(a)) for a in angles)
    cos_sum = sum(math.cos(math.radians(a)) for a in angles)
    mean_deg = math.degrees(math.atan2(sin_sum, cos_sum))
    return mean_deg % 360.0


def geo_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Vzdálenost v metrech mezi dvěma body na zemském povrchu (aproximace WGS84)."""
    if lat1 == 0.0 or lat2 == 0.0:
        return 0.0
    dx = (lat2 - lat1) * 111139.0
    dy = (lon2 - lon1) * 111139.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
    return math.hypot(dx, dy)


def compute_tangent_offset(offset_x: float, offset_y: float, v_ms: float, wz_deg_s: float) -> float:
    """
    Vypočte úhel odchylky beta (ve stupních) vektoru rychlosti antény od podélné osy robota:
    beta = atan2(v_lat, v_fwd)
    kde:
      v_ms: dopředná rychlost středu robota (m/s)
      wz_deg_s: úhlová rychlost otáčení (stupně/s, CW = +)
      offset_x: předsazení antény dopředu (m)
      offset_y: příčné posunutí antény vlevo (m)
    """
    if v_ms < 0.2:
        return 0.0
    wz_rad = math.radians(wz_deg_s)
    v_fwd = v_ms + wz_rad * offset_y
    v_lat = wz_rad * offset_x
    return math.degrees(math.atan2(v_lat, v_fwd))


def displace_wgs84(lat: float, lon: float, d_north: float, d_east: float) -> Tuple[float, float]:
    """
    Posun bodu na zemském elipsoidu WGS84 o d_north a d_east metrů.
    Vrací (nový_lat, nový_lon).
    """
    if lat == 0.0 and lon == 0.0:
        return 0.0, 0.0
    new_lat = lat + (d_north / 111139.0)
    lat_rad = math.radians(lat)
    cos_lat = math.cos(lat_rad)
    if abs(cos_lat) < 1e-6:
        cos_lat = 1e-6 if cos_lat >= 0 else -1e-6
    new_lon = lon + (d_east / (111139.0 * cos_lat))
    return new_lat, new_lon
