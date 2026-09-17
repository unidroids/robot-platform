from __future__ import annotations
from typing import Tuple
import math

__all__ = [
    "deg2rad", "rad2deg",
    "lla_to_ecef", "ecef_to_lla",
    "ecef_to_enu", "enu_to_ecef",
    "heading_gnss_to_enu", "heading_enu_to_gnss",
    "yawrate_cw_to_ccw", "yawrate_ccw_to_cw",
]

_WGS84_A = 6378137.0                  
_WGS84_F = 1.0 / 298.257223563        
_WGS84_B = _WGS84_A * (1.0 - _WGS84_F)
_WGS84_E2 = (_WGS84_A**2 - _WGS84_B**2) / (_WGS84_A**2)   
_WGS84_EP2 = (_WGS84_A**2 - _WGS84_B**2) / (_WGS84_B**2)  

def deg2rad(d: float) -> float:
    return d * math.pi / 180.0

def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi

def lla_to_ecef(lat_deg: float, lon_deg: float, h_m: float = 0.0) -> Tuple[float, float, float]:
    lat = deg2rad(lat_deg)
    lon = deg2rad(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    x = (N + h_m) * cos_lat * cos_lon
    y = (N + h_m) * cos_lat * sin_lon
    z = (N * (1.0 - _WGS84_E2) + h_m) * sin_lat
    return x, y, z

def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    r = math.hypot(x, y)
    if r < 1e-12:
        lat = math.copysign(math.pi / 2.0, z)
        lon = 0.0
        h = abs(z) - _WGS84_B
        return rad2deg(lat), rad2deg(lon), h

    F = 54.0 * _WGS84_B**2 * z * z
    G = r * r + (1.0 - _WGS84_E2) * z * z - _WGS84_E2 * (_WGS84_A**2 - _WGS84_B**2)
    c = (_WGS84_E2**2) * F * r * r / (G**3)
    s = (1.0 + c + math.sqrt(c * c + 2.0 * c)) ** (1.0 / 3.0)
    P = F / (3.0 * (s + 1.0 / s + 1.0) ** 2 * G * G)
    Q = math.sqrt(1.0 + 2.0 * _WGS84_E2 * _WGS84_E2 * P)
    r0 = -(P * _WGS84_E2 * r) / (1.0 + Q) + math.sqrt(
        0.5 * _WGS84_A * _WGS84_A * (1.0 + 1.0 / Q)
        - P * (1.0 - _WGS84_E2) * z * z / (Q * (1.0 + Q))
        - 0.5 * P * r * r
    )
    U = math.sqrt((r - _WGS84_E2 * r0) ** 2 + z * z)
    V = math.sqrt((r - _WGS84_E2 * r0) ** 2 + (1.0 - _WGS84_E2) * z * z)
    z0 = (_WGS84_B**2) * z / (_WGS84_A * V)
    h = U * (1.0 - (_WGS84_B**2) / (_WGS84_A * V))
    lat = math.atan2(z + _WGS84_EP2 * z0, r)
    lon = math.atan2(y, x)
    return rad2deg(lat), rad2deg(lon), h

def ecef_to_enu(x: float, y: float, z: float,
                lat0_deg: float, lon0_deg: float, h0_m: float = 0.0) -> Tuple[float, float, float]:
    x0, y0, z0 = lla_to_ecef(lat0_deg, lon0_deg, h0_m)
    dx = x - x0
    dy = y - y0
    dz = z - z0
    lat0 = deg2rad(lat0_deg)
    lon0 = deg2rad(lon0_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    e = -sin_lon * dx + cos_lon * dy
    n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    u =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return e, n, u

def enu_to_ecef(e: float, n: float, u: float,
                lat0_deg: float, lon0_deg: float, h0_m: float = 0.0) -> Tuple[float, float, float]:
    lat0 = deg2rad(lat0_deg)
    lon0 = deg2rad(lon0_deg)
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u
    dy =  cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u
    dz =               cos_lat * n           + sin_lat * u

    x0, y0, z0 = lla_to_ecef(lat0_deg, lon0_deg, h0_m)
    return x0 + dx, y0 + dy, z0 + dz

def heading_gnss_to_enu(h_gnss_deg: float) -> float:
    return (90.0 - h_gnss_deg) % 360.0

def heading_enu_to_gnss(psi_enu_deg: float) -> float:
    return (90.0 - psi_enu_deg) % 360.0

def yawrate_cw_to_ccw(r_cw_deg_s: float) -> float:
    return -r_cw_deg_s

def yawrate_ccw_to_cw(r_ccw_deg_s: float) -> float:
    return -r_ccw_deg_s
