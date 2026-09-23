import json
import math
import os
from dataclasses import dataclass

try:
    from .near_waypoint import NearWaypoint, NearState
except (ImportError, ValueError):
    from near_waypoint import NearWaypoint, NearState

@dataclass
class Waypoint:
    lat: float
    lon: float
    rel_azimuth_deg: float = 0.0

class PathTracker:
    def _get_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        lat_mid = math.radians((lat1 + lat2) / 2.0)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_mid) + 1.175 * math.cos(4 * lat_mid)
        m_per_deg_lon = 111412.84 * math.cos(lat_mid) - 93.5 * math.cos(3 * lat_mid)
        dx = (lon2 - lon1) * m_per_deg_lon
        dy = (lat2 - lat1) * m_per_deg_lat
        return math.degrees(math.atan2(dx, dy)) % 360.0

    def __init__(self, route_input, L_near_m: float = 2.0):
        self.waypoints = []
        self.L_near_m = L_near_m
        
        data = None
        if isinstance(route_input, (dict, list)):
            data = route_input
        elif isinstance(route_input, str):
            try:
                data = json.loads(route_input)
            except Exception as e:
                raise ValueError(f"Chyba parsování JSON trasy: {e}")
        else:
            raise ValueError(f"Neplatný formát trasy (očekáván dict, list nebo JSON string, předáno: {type(route_input).__name__})")

        if data is None:
            raise ValueError("Vstupní data trasy jsou prázdná")

        # 1. Formát MAPS (obsahuje pole 'nodes' s 'lat' a 'lon')
        if isinstance(data, dict) and "nodes" in data:
            for node in data["nodes"]:
                if isinstance(node, dict) and "lat" in node and "lon" in node:
                    self.waypoints.append(Waypoint(lat=float(node["lat"]), lon=float(node["lon"])))
        # 2. Formát WAYPOINTS (obsahuje pole 'waypoints')
        elif isinstance(data, dict) and "waypoints" in data:
            for wp in data["waypoints"]:
                rel_az = float(wp.get("rel_azimuth_deg", 0.0))
                self.waypoints.append(Waypoint(lat=float(wp["lat"]), lon=float(wp["lon"]), rel_azimuth_deg=rel_az))
        # 3. Přímý seznam bodů
        elif isinstance(data, list):
            for pt in data:
                if isinstance(pt, dict) and "lat" in pt and "lon" in pt:
                    self.waypoints.append(Waypoint(lat=float(pt["lat"]), lon=float(pt["lon"])))
        else:
            raise ValueError("Neznámá struktura trasy (očekává se 'nodes', 'waypoints' nebo pole bodů)")

        if len(self.waypoints) < 2:
            raise ValueError(f"Nedostatečný počet bodů v trase: {len(self.waypoints)} (vyžadováno min. 2)")

        print(f"[PathTracker] Načteno {len(self.waypoints)} waypointů")

        # Přepočet rel_azimuth_deg na základě geometrie trasy
        for i in range(1, len(self.waypoints) - 1):
            wp_prev = self.waypoints[i-1]
            wp_curr = self.waypoints[i]
            wp_next = self.waypoints[i+1]
            b1 = self._get_bearing(wp_prev.lat, wp_prev.lon, wp_curr.lat, wp_curr.lon)
            b2 = self._get_bearing(wp_curr.lat, wp_curr.lon, wp_next.lat, wp_next.lon)
            diff = (b2 - b1 + 180) % 360 - 180
            wp_curr.rel_azimuth_deg = diff
            
        self.current_wp_index = 0
        self.active_near_wp = None
        self.artificial_segment = False
        self._update_active_wp(0)
        print(f"[PathTracker] Inicializace: Start z Waypointu 0 -> 1.")

    def _update_active_wp(self, index: int):
        if index < len(self.waypoints) - 1:
            curr_wp = self.waypoints[index]
            next_wp = self.waypoints[index + 1]
            self.active_near_wp = NearWaypoint(
                curr_wp.lat, curr_wp.lon,
                next_wp.lat, next_wp.lon,
                L_near_m=self.L_near_m,
                end_rel_azimuth_deg=next_wp.rel_azimuth_deg
            )
        else:
            self.active_near_wp = None

    def update(self, R_lat: float, R_lon: float) -> NearState:
        if not self.waypoints or self.active_near_wp is None:
            return None

        state = self.active_near_wp.update(R_lat, R_lon)

        # Přepnutí na další waypoint
        if state.distance_to_goal_m <= 0.0:
            self.current_wp_index += 1
            if self.current_wp_index < len(self.waypoints) - 1:
                print(f"[PathTracker] Dosažen Waypoint {self.current_wp_index}. Přepínám na další segment.")
                self._update_active_wp(self.current_wp_index)
                state = self.active_near_wp.update(R_lat, R_lon)
            else:
                print(f"[PathTracker] Dosažen cíl celé trasy!")
                self.active_near_wp = None
                return None

        return state

