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

    def __init__(self, route_json_path: str, L_near_m: float = 2.0):
        self.waypoints = []
        self.L_near_m = L_near_m
        
        # Ověření cesty k souboru s fallbackem na lokální složku
        resolved_path = route_json_path
        if not os.path.exists(resolved_path):
            local_alt = os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints", os.path.basename(route_json_path))
            if os.path.exists(local_alt):
                resolved_path = local_alt

        try:
            with open(resolved_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                for wp in data.get("waypoints", []):
                    rel_az = wp.get("rel_azimuth_deg", 0.0)
                    self.waypoints.append(Waypoint(lat=wp["lat"], lon=wp["lon"], rel_azimuth_deg=rel_az))
            print(f"[PathTracker] Načteno {len(self.waypoints)} waypointů ze souboru: {resolved_path}")
            
            # Přepočet rel_azimuth_deg na základě geometrie trasy
            for i in range(1, len(self.waypoints) - 1):
                wp_prev = self.waypoints[i-1]
                wp_curr = self.waypoints[i]
                wp_next = self.waypoints[i+1]
                b1 = self._get_bearing(wp_prev.lat, wp_prev.lon, wp_curr.lat, wp_curr.lon)
                b2 = self._get_bearing(wp_curr.lat, wp_curr.lon, wp_next.lat, wp_next.lon)
                diff = (b2 - b1 + 180) % 360 - 180
                wp_curr.rel_azimuth_deg = diff

        except Exception as e:
            print(f"[PathTracker] Chyba při načítání cesty ({resolved_path}): {e}")
            
        self.current_wp_index = 0
        self.active_near_wp = None
        self.artificial_segment = False
        self.initialized_position = False

    def _update_active_wp(self, index: int, S_lat=None, S_lon=None, E_lat=None, E_lon=None):
        if S_lat is not None and E_lat is not None:
            b1 = self._get_bearing(S_lat, S_lon, E_lat, E_lon)
            if self.current_wp_index < len(self.waypoints) - 1:
                tgt_S = self.waypoints[self.current_wp_index]
                tgt_E = self.waypoints[self.current_wp_index + 1]
                b2 = self._get_bearing(tgt_S.lat, tgt_S.lon, tgt_E.lat, tgt_E.lon)
                diff = (b2 - b1 + 180) % 360 - 180
            else:
                diff = 0.0
            self.active_near_wp = NearWaypoint(
                S_lat, S_lon, E_lat, E_lon,
                L_near_m=self.L_near_m,
                end_rel_azimuth_deg=diff
            )
            return

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
        if not self.waypoints:
            return None

        # 1. První inicializace
        if not self.initialized_position:
            self.initialized_position = True
            
            # Najdeme nejbližší segment k aktuální pozici
            best_idx = 0
            min_dist = float('inf')
            
            for i in range(len(self.waypoints) - 1):
                wp1 = self.waypoints[i]
                wp2 = self.waypoints[i+1]
                nw = NearWaypoint(wp1.lat, wp1.lon, wp2.lat, wp2.lon, L_near_m=self.L_near_m)
                st = nw.update(R_lat, R_lon)
                if st.d_perp_m < min_dist:
                    min_dist = st.d_perp_m
                    best_idx = i

            print(f"[PathTracker] Inicializace: Nejbližší segment je {best_idx} (vzdálenost k čáře: {min_dist:.2f} m).")
            
            if min_dist > 1.0:
                self.artificial_segment = True
                self.current_wp_index = best_idx
                
                wp_target = self.waypoints[best_idx]
                print(f"[PathTracker] Vytvářím umělý segment: [Aktuální pozice] -> Waypoint {best_idx}.")
                self._update_active_wp(best_idx, S_lat=R_lat, S_lon=R_lon, E_lat=wp_target.lat, E_lon=wp_target.lon)
            else:
                self.artificial_segment = False
                self.current_wp_index = best_idx
                self._update_active_wp(self.current_wp_index)

        # 2. Běžná aktualizace aktivního segmentu
        if self.active_near_wp is None:
            return None
            
        state = self.active_near_wp.update(R_lat, R_lon)
        
        # Přepnutí na další waypoint
        if state.distance_to_goal_m <= 0.0:
            if self.artificial_segment:
                print(f"[PathTracker] Dosažen cíl umělého segmentu. Napojuji se na standardní trasu od indexu {self.current_wp_index}.")
                self.artificial_segment = False
                self._update_active_wp(self.current_wp_index)
                if self.active_near_wp:
                    state = self.active_near_wp.update(R_lat, R_lon)
            else:
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
