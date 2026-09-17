from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Literal
import math

try:
    from .geo_utils import (
        lla_to_ecef, ecef_to_lla,
        ecef_to_enu, enu_to_ecef,
        heading_enu_to_gnss,
    )
except (ImportError, ValueError):
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
            psi_gnss = heading_enu_to_gnss(math.degrees(math.atan2(Ey, Ex))) if dist_goal > 1e-12 else None
            return NearState(
                distance_to_goal_m=dist_goal,
                abs_distance_to_goal_m=abs_dist_goal,
                heading_to_near_gnss_deg=psi_gnss,
                case=None,
                near_lat=self.E_lat,
                near_lon=self.E_lon,
                near_x_m=Ex,
                near_y_m=Ey,
                d_perp_m=dist_goal,
                closest_lat=self.E_lat,
                closest_lon=self.E_lon,
                end_rel_azimuth_deg=self.end_rel_azimuth_deg,
            )

        ux, uy = vx / L_seg, vy / L_seg

        t_close = -(Sx * ux + Sy * uy)
        d_signed = Sx * uy - Sy * ux
        d_perp = abs(d_signed)

        Px, Py = Sx + t_close * ux, Sy + t_close * uy

        t_goal = L_seg - t_close
        dist_goal = t_goal

        L = self.L_near_m
        if (L is None) or (d_perp >= L - self.eps_m):
            N_case: NearCase = "TANGENT"
            xN, yN = Px, Py
        else:
            N_case = "TWO_INTERSECTIONS"
            delta = math.sqrt(max(0.0, L * L - d_perp * d_perp))
            xN, yN = Px + delta * ux, Py + delta * uy

        near_ecef = enu_to_ecef(xN, yN, 0.0, R_lat, R_lon, 0.0)
        near_lat, near_lon, _ = ecef_to_lla(*near_ecef)

        closest_ecef = enu_to_ecef(Px, Py, 0.0, R_lat, R_lon, 0.0)
        closest_lat, closest_lon, _ = ecef_to_lla(*closest_ecef)

        heading_gnss = heading_enu_to_gnss(math.degrees(math.atan2(yN, xN)))

        return NearState(
            distance_to_goal_m=dist_goal,
            abs_distance_to_goal_m=abs_dist_goal,
            heading_to_near_gnss_deg=heading_gnss,
            case=N_case,
            near_lat=near_lat,
            near_lon=near_lon,
            near_x_m=xN,
            near_y_m=yN,
            d_perp_m=d_perp,
            closest_lat=closest_lat,
            closest_lon=closest_lon,
            end_rel_azimuth_deg=self.end_rel_azimuth_deg,
        )

    def update(self, R_lat: float, R_lon: float) -> NearState:
        st = self._compute(R_lat, R_lon)
        self.state = st
        return st

    @property
    def distance_to_goal_m(self) -> Optional[float]:
        return None if self.state is None else self.state.distance_to_goal_m

    @property
    def abs_distance_to_goal_m(self) -> Optional[float]:
        return None if self.state is None else self.state.abs_distance_to_goal_m

    @property
    def heading_to_near_gnss_deg(self) -> Optional[float]:
        return None if self.state is None else self.state.heading_to_near_gnss_deg

    @property
    def case(self) -> Optional[NearCase]:
        return None if self.state is None else self.state.case

    @property
    def near_lat(self) -> Optional[float]:
        return None if self.state is None else self.state.near_lat

    @property
    def near_lon(self) -> Optional[float]:
        return None if self.state is None else self.state.near_lon

    @property
    def near_x_m(self) -> Optional[float]:
        return None if self.state is None else self.state.near_x_m

    @property
    def near_y_m(self) -> Optional[float]:
        return None if self.state is None else self.state.near_y_m

    @property
    def d_perp_m(self) -> Optional[float]:
        return None if self.state is None else self.state.d_perp_m

    @property
    def closest_lat(self) -> Optional[float]:
        return None if self.state is None else self.state.closest_lat

    @property
    def closest_lon(self) -> Optional[float]:
        return None if self.state is None else self.state.closest_lon
