# maps/geo_geometry.py
"""
Geodetické a geometrické funkce pro mapovou službu:
- Převod WGS84 (lat, lon) <-> lokální rovinné ENU souřadnice v metrech.
- Výpočet vzdálenosti a azimutu mezi dvěma body.
- Projekce bodu na úsečku (vzdálenost bod-úsečka, průmět).
- Výpočet pravostranného posunu (offset vpravo) pro jízdu v pravém jízdním pruhu.
"""
from __future__ import annotations
import math
from typing import Tuple, List, Optional, Dict, Any

# Parametry WGS84 elipsoidu
WGS84_A = 6378137.0  # hlavní poloosa (m)
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_E2 = (WGS84_A**2 - WGS84_B**2) / (WGS84_A**2)


def norm_deg(deg: float) -> float:
    """Normalizace úhlu do rozsahu [0, 360)."""
    d = deg % 360.0
    if d < 0.0:
        d += 360.0
    return d


def calc_distance_and_azimuth(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    """
    Vypočte vzdálenost (m) a počáteční azimut (0-360°) z bodu 1 do bodu 2 na elipsoidu WGS84.
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    lam1 = math.radians(lon1)
    lam2 = math.radians(lon2)

    dphi = phi2 - phi1
    dlam = lam2 - lam1

    # Průměrný poloměr zakřivení
    mean_lat = (phi1 + phi2) / 2.0
    sin_lat = math.sin(mean_lat)
    denom = 1.0 - WGS84_E2 * (sin_lat**2)
    radius_m = WGS84_A * (1.0 - WGS84_E2) / (denom**1.5)
    radius_n = WGS84_A / math.sqrt(denom)

    dx = dlam * radius_n * math.cos(mean_lat)
    dy = dphi * radius_m
    dist = math.hypot(dx, dy)

    # Azimut (sever = 0°, východ = 90°)
    azimuth = math.degrees(math.atan2(dx, dy))
    return dist, norm_deg(azimuth)


def wgs84_to_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Převede souřadnice (lat, lon) do lokální tečné roviny ENU (East, North) v metrech
    vzhledem k referenčnímu bodu (ref_lat, ref_lon).
    """
    phi = math.radians(lat)
    lam = math.radians(lon)
    phi0 = math.radians(ref_lat)
    lam0 = math.radians(ref_lon)

    dphi = phi - phi0
    dlam = lam - lam0

    sin_phi0 = math.sin(phi0)
    denom = 1.0 - WGS84_E2 * (sin_phi0**2)
    radius_m = WGS84_A * (1.0 - WGS84_E2) / (denom**1.5)
    radius_n = WGS84_A / math.sqrt(denom)

    x_east = dlam * radius_n * math.cos(phi0)
    y_north = dphi * radius_m
    return x_east, y_north


def enu_to_wgs84(x_east: float, y_north: float, ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """
    Převede lokální rovinné souřadnice ENU (East, North) v metrech zpět na WGS84 (lat, lon).
    """
    phi0 = math.radians(ref_lat)

    sin_phi0 = math.sin(phi0)
    denom = 1.0 - WGS84_E2 * (sin_phi0**2)
    radius_m = WGS84_A * (1.0 - WGS84_E2) / (denom**1.5)
    radius_n = WGS84_A / math.sqrt(denom)

    dphi = y_north / radius_m
    dlam = x_east / (radius_n * math.cos(phi0))

    lat = ref_lat + math.degrees(dphi)
    lon = ref_lon + math.degrees(dlam)
    return lat, lon


def project_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float
) -> Tuple[float, float, float, float]:
    """
    Projekce bodu P(px, py) na úsečku AB.
    Vrátí:
    - qx, qy: souřadnice nejbližšího bodu Q na úsečce
    - distance: vzdálenost |P - Q| v metrech
    - t: parametr projekce na úsečce (0.0 = bod A, 1.0 = bod B, v rozmezí [0, 1])
    """
    vx = bx - ax
    vy = by - ay
    v_len_sq = vx * vx + vy * vy

    if v_len_sq < 1e-12:
        # Body A a B splývají
        dist = math.hypot(px - ax, py - ay)
        return ax, ay, dist, 0.0

    # Skalární projekce
    wx = px - ax
    wy = py - ay
    t = (wx * vx + wy * vy) / v_len_sq
    t_clamped = max(0.0, min(1.0, t))

    qx = ax + t_clamped * vx
    qy = ay + t_clamped * vy
    dist = math.hypot(px - qx, py - qy)

    return qx, qy, dist, t_clamped


def calc_right_offset_for_width(width_m: float) -> float:
    """
    Vypočte odsazení vpravo od středu cesty podle šířky komunikace:
    - Šířka < 2.0 m: offset = 0.0 m (střed cesty)
    - Šířka 2.0 až 3.0 m: offset = 0.5 m
    - Šířka 3.0 až 4.0 m: offset = 0.75 m
    - Šířka >= 4.0 m: offset = 1.0 m
    """
    if width_m < 2.0:
        return 0.0
    elif width_m < 3.0:
        return 0.5
    elif width_m < 4.0:
        return 0.75
    else:
        return 1.0


def compute_segment_right_normal(ax: float, ay: float, bx: float, by: float) -> Tuple[float, float]:
    """
    Vypočte jednotkový normálový vektor směřující VPRAVO od směru pohybu A -> B.
    V souřadnicovém systému ENU (x=East, y=North):
    Vektor pohybu: (vx, vy).
    Pravostranná normála: (vy, -vx).
    """
    vx = bx - ax
    vy = by - ay
    length = math.hypot(vx, vy)
    if length < 1e-9:
        return 0.0, 0.0
    ux = vx / length
    uy = vy / length
    # Vpravo od (ux, uy)
    return uy, -ux


def offset_polyline_right(
    points_enu: List[Tuple[float, float]],
    widths: List[float]
) -> List[Tuple[float, float]]:
    """
    Posune lomenou čáru (posloupnost bodů v ENU) doprava vzhledem ke směru jízdy.
    points_enu: [(x0, y0), (x1, y1), ..., (xn, yn)]
    widths: šířky jednotlivých úseků (délka N-1).
    Vrací posunuté body [(x0', y0'), (x1', y1'), ..., (xn', yn')].
    """
    n = len(points_enu)
    if n == 0:
        return []
    if n == 1:
        return [points_enu[0]]

    # 1. Spočteme normálové vektory a offsety pro každý úsek
    seg_normals: List[Tuple[float, float]] = []
    seg_offsets: List[float] = []

    for i in range(n - 1):
        ax, ay = points_enu[i]
        bx, by = points_enu[i + 1]
        nx, ny = compute_segment_right_normal(ax, ay, bx, by)
        seg_normals.append((nx, ny))
        w = widths[i] if i < len(widths) else 4.0
        seg_offsets.append(calc_right_offset_for_width(w))

    # 2. Posuneme jednotlivé body
    result: List[Tuple[float, float]] = []

    # První bod posuneme podle prvního úseku
    p0_x = points_enu[0][0] + seg_normals[0][0] * seg_offsets[0]
    p0_y = points_enu[0][1] + seg_normals[0][1] * seg_offsets[0]
    result.append((p0_x, p0_y))

    # Vnitřní uzly - průměr normál s miter join omezením
    for i in range(1, n - 1):
        n1x, n1y = seg_normals[i - 1]
        n2x, n2y = seg_normals[i]
        off1 = seg_offsets[i - 1]
        off2 = seg_offsets[i]
        off_avg = (off1 + off2) / 2.0

        # Součet normál
        sum_nx = n1x + n2x
        sum_ny = n1y + n2y
        sum_len = math.hypot(sum_nx, sum_ny)

        if sum_len < 1e-4:
            # Úseky jdou téměř v protisměru (otočka o 180°)
            nx = n1x
            ny = n1y
            miter_scale = 1.0
        else:
            nx = sum_nx / sum_len
            ny = sum_ny / sum_len
            # Miter délka = off / cos(theta / 2) = off / (n1 . n_avg)
            cos_half = n1x * nx + n1y * ny
            if cos_half > 0.1:
                miter_scale = min(1.5, 1.0 / cos_half)  # omezíme špičaté zuby
            else:
                miter_scale = 1.0

        px = points_enu[i][0] + nx * off_avg * miter_scale
        py = points_enu[i][1] + ny * off_avg * miter_scale
        result.append((px, py))

    # Poslední bod posuneme podle posledního úseku
    p_last_x = points_enu[-1][0] + seg_normals[-1][0] * seg_offsets[-1]
    p_last_y = points_enu[-1][1] + seg_normals[-1][1] * seg_offsets[-1]
    result.append((p_last_x, p_last_y))

    return result
