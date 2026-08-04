from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Literal
import math

from geo_utils import (
    lla_to_ecef, ecef_to_lla,
    ecef_to_enu, enu_to_ecef,
    heading_enu_to_gnss,
)

NearCase = Literal["TWO_INTERSECTIONS", "TANGENT"]

@dataclass
class NearState:
    distance_to_goal_m: float                    
    abs_distance_to_goal_m: float                
    heading_to_near_gnss_deg: Optional[float]    
    case: Optional[NearCase]                     
    near_lat: Optional[float] = None
    near_lon: Optional[float] = None
    near_x_m: Optional[float] = None             
    near_y_m: Optional[float] = None             
    d_perp_m: Optional[float] = None             
    closest_lat: Optional[float] = None          
    closest_lon: Optional[float] = None          
    end_rel_azimuth_deg: Optional[float] = 0.0


class NearWaypoint:
    def __init__(
        self,
        S_lat: float, S_lon: float,
        E_lat: float, E_lon: float,
        L_near_m: Optional[float] = 1.0,
        eps_m: float = 2e-3,
        end_rel_azimuth_deg: float = 0.0,
    ) -> None:
        self.S_lat = float(S_lat)
        self.S_lon = float(S_lon)
        self.E_lat = float(E_lat)
        self.E_lon = float(E_lon)
        self.L_near_m = float(L_near_m) if L_near_m is not None else None
        self.eps_m = float(eps_m)

        self._S_ecef = lla_to_ecef(self.S_lat, self.S_lon, 0.0)
        self._E_ecef = lla_to_ecef(self.E_lat, self.E_lon, 0.0)
        self.end_rel_azimuth_deg = end_rel_azimuth_deg
        self.state: Optional[NearState] = None

    def _compute(self, R_lat: float, R_lon: float) -> NearState:
        Sx, Sy, _ = ecef_to_enu(*self._S_ecef, R_lat, R_lon, 0.0)
        Ex, Ey, _ = ecef_to_enu(*self._E_ecef, R_lat, R_lon, 0.0)

        abs_dist_goal = math.hypot(Ex, Ey)

        vx, vy = Ex - Sx, Ey - Sy
        L_seg = math.hypot(vx, vy)
        if L_seg < 1e-12:
            dist_goal = math.hypot(Ex, Ey)
            return NearState(
                distance_to_goal_m=dist_goal,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=None,
                case=None,
                near_lat=None, near_lon=None,
                near_x_m=None, near_y_m=None,
                d_perp_m=None,
                closest_lat=self.S_lat,
                closest_lon=self.S_lon,
                end_rel_azimuth_deg=self.end_rel_azimuth_deg,
            )

        vx /= L_seg
        vy /= L_seg

        t_q = (-(Sx * vx + Sy * vy))
        Qx = Sx + t_q * vx
        Qy = Sy + t_q * vy
        d_perp = math.hypot(Qx, Qy)

        distance_to_goal_m = (L_seg - t_q)

        # Calculate closest point strictly on the segment [0, L_seg]
        t_closest = max(0.0, min(L_seg, t_q))
        Cx = Sx + t_closest * vx
        Cy = Sy + t_closest * vy
        Cx_ecef, Cy_ecef, Cz_ecef = enu_to_ecef(Cx, Cy, 0.0, R_lat, R_lon, 0.0)
        Clat, Clon, _ = ecef_to_lla(Cx_ecef, Cy_ecef, Cz_ecef)

        if self.L_near_m is None:
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=None,
                case=None,
                near_lat=None, near_lon=None,
                near_x_m=None, near_y_m=None,
                d_perp_m=d_perp,
                closest_lat=Clat,
                closest_lon=Clon,
                end_rel_azimuth_deg=self.end_rel_azimuth_deg,
            )

        Lr = self.L_near_m
        eps = self.eps_m

        if d_perp > Lr + eps:
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=None,
                case=None,
                near_lat=None, near_lon=None,
                near_x_m=None, near_y_m=None,
                d_perp_m=d_perp,
                closest_lat=Clat,
                closest_lon=Clon,
                end_rel_azimuth_deg=self.end_rel_azimuth_deg,
            )
        elif abs(d_perp - Lr) <= eps:
            nx, ny = Qx, Qy
            nx_ecef, ny_ecef, nz_ecef = enu_to_ecef(nx, ny, 0.0, R_lat, R_lon, 0.0)
            nlat, nlon, _ = ecef_to_lla(nx_ecef, ny_ecef, nz_ecef)
            heading_enu = math.degrees(math.atan2(ny, nx)) % 360.0
            heading_gnss = heading_enu_to_gnss(heading_enu)
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=heading_gnss,
                case="TANGENT",
                near_lat=nlat, near_lon=nlon,
                near_x_m=nx, near_y_m=ny,
                d_perp_m=d_perp,
                closest_lat=Clat,
                closest_lon=Clon,
                end_rel_azimuth_deg=self.end_rel_azimuth_deg,
            )
        else:
            delta = math.sqrt(max(0.0, Lr * Lr - d_perp * d_perp))
            n1x, n1y = Qx + delta * vx, Qy + delta * vy
            n2x, n2y = Qx - delta * vx, Qy - delta * vy
            t1 = ((n1x - Sx) * vx + (n1y - Sy) * vy)
            t2 = ((n2x - Sx) * vx + (n2y - Sy) * vy)
            if t1 >= t2:
                nx, ny = n1x, n1y
            else:
                nx, ny = n2x, n2y

            nx_ecef, ny_ecef, nz_ecef = enu_to_ecef(nx, ny, 0.0, R_lat, R_lon, 0.0)
            nlat, nlon, _ = ecef_to_lla(nx_ecef, ny_ecef, nz_ecef)
            heading_enu = math.degrees(math.atan2(ny, nx)) % 360.0
            heading_gnss = heading_enu_to_gnss(heading_enu)
            return NearState(
                distance_to_goal_m=distance_to_goal_m,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=heading_gnss,
                case="TWO_INTERSECTIONS",
                near_lat=nlat, near_lon=nlon,
                near_x_m=nx, near_y_m=ny,
                d_perp_m=d_perp,
                closest_lat=Clat,
                closest_lon=Clon,
                end_rel_azimuth_deg=self.end_rel_azimuth_deg,
            )

    def update(self, R_lat: float, R_lon: float) -> tuple[float, float, Optional[float]]:
        s = self._compute(R_lat, R_lon)
        self.state = s
        return (s.distance_to_goal_m, s.abs_distance_to_goal_m, s.heading_to_near_gnss_deg)
