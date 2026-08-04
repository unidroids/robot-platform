import json
import math
from dataclasses import dataclass
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
        
        try:
            with open(route_json_path, 'r') as f:
                data = json.load(f)
                for wp in data.get("waypoints", []):
                    rel_az = wp.get("rel_azimuth_deg", 0.0)
                    self.waypoints.append(Waypoint(lat=wp["lat"], lon=wp["lon"], rel_azimuth_deg=rel_az))
            print(f"[PathTracker] Načteno {len(self.waypoints)} waypointů.")
            
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
            print(f"[PathTracker] Chyba při načítání cesty: {e}")
            
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
            self.active_near_wp = NearWaypoint(S_lat, S_lon, E_lat, E_lon, L_near_m=self.L_near_m, end_rel_azimuth_deg=diff)
            self.artificial_segment = True
            print(f"[PathTracker] Vytvořena UMĚLÁ úsečka: S=({S_lat:.6f}, {S_lon:.6f}) -> E=({E_lat:.6f}, {E_lon:.6f}) s úhlem napojení {diff:.1f}°")
        else:
            self.current_wp_index = index
            S = self.waypoints[index]
            E = self.waypoints[index + 1]
            diff = E.rel_azimuth_deg
            self.active_near_wp = NearWaypoint(S.lat, S.lon, E.lat, E.lon, L_near_m=self.L_near_m, end_rel_azimuth_deg=diff)
            self.artificial_segment = False
            print(f"[PathTracker] Změna na segment {index}: S=({S.lat:.6f}, {S.lon:.6f}) -> E=({E.lat:.6f}, {E.lon:.6f}) s azimutem konce {diff:.1f}°")

    def _do_global_search(self, R_lat: float, R_lon: float):
        best_idx = -1
        min_dist = float('inf')
        
        for i in range(len(self.waypoints) - 1):
            nw = NearWaypoint(self.waypoints[i].lat, self.waypoints[i].lon, 
                              self.waypoints[i+1].lat, self.waypoints[i+1].lon, 
                              L_near_m=self.L_near_m,
                              end_rel_azimuth_deg=self.waypoints[i+1].rel_azimuth_deg)
            nw.update(R_lat, R_lon)
            st = nw.state
            
            if st.d_perp_m is not None and st.d_perp_m < min_dist:
                min_dist = st.d_perp_m
                best_idx = i
                
        if best_idx != -1:
            print(f"[PathTracker] Startovní vyhledání: Nejbližší segment je {best_idx} (vzdálenost {min_dist:.2f}m).")
            self.current_wp_index = best_idx
            
            nw_best = NearWaypoint(self.waypoints[best_idx].lat, self.waypoints[best_idx].lon, 
                              self.waypoints[best_idx+1].lat, self.waypoints[best_idx+1].lon, 
                              L_near_m=self.L_near_m,
                              end_rel_azimuth_deg=self.waypoints[best_idx+1].rel_azimuth_deg)
            nw_best.update(R_lat, R_lon)
            
            if nw_best.state.case is not None:
                self._update_active_wp(best_idx)
            else:
                target_lat = nw_best.state.closest_lat
                target_lon = nw_best.state.closest_lon
                print(f"[PathTracker] Startovní bod je příliš daleko od trasy. Vytvářím umělou úsečku k nejbližšímu bodu na segmentu {best_idx}.")
                self._update_active_wp(-1, S_lat=R_lat, S_lon=R_lon, E_lat=target_lat, E_lon=target_lon)

    def update(self, R_lat: float, R_lon: float) -> NearState:
        if not self.initialized_position and len(self.waypoints) >= 2:
            self._do_global_search(R_lat, R_lon)
            self.initialized_position = True

        if not self.active_near_wp:
            return None

        # --- FÁZE 1: Zkusíme aktuální segment ---
        if not self.artificial_segment:
            self.active_near_wp.update(R_lat, R_lon)
            state = self.active_near_wp.state
            
            # 1. Přejetí segmentu (dist < 0)
            if state.distance_to_goal_m is not None and state.distance_to_goal_m < 0:
                if self.current_wp_index + 2 < len(self.waypoints):
                    print(f"[PathTracker] Přejíždím na další segment: {self.current_wp_index + 1}")
                    self._update_active_wp(self.current_wp_index + 1)
                    return self.update(R_lat, R_lon)
                else:
                    print(f"[PathTracker] Dosažen cíl trasy (GOAL_REACHED).")
                    return state

            # 2. Úspěšný průnik (máme near point)
            if state.case is not None:
                return state
                
            print(f"[PathTracker] Ztráta průniku na aktuálním segmentu {self.current_wp_index} (d_perp={state.d_perp_m:.2f}m). Vracím se k němu nejkratší cestou.")
            target_lat = state.closest_lat
            target_lon = state.closest_lon
            self._update_active_wp(-1, S_lat=R_lat, S_lon=R_lon, E_lat=target_lat, E_lon=target_lon)
            self.active_near_wp.update(R_lat, R_lon)
            return self.active_near_wp.state
        
        else:
            # --- Řešení pro umělý segment ---
            self.active_near_wp.update(R_lat, R_lon)
            state = self.active_near_wp.state
            
            # Pokud dojedeme na konec umělého segmentu (např. do 0.5m)
            if state.distance_to_goal_m is not None and state.distance_to_goal_m < 0.5:
                print(f"[PathTracker] Umělý segment projet. Vracím se k normálnímu hledání na segmentu {self.current_wp_index}.")
                self.artificial_segment = False
                self._update_active_wp(self.current_wp_index)
                return self.update(R_lat, R_lon)
            elif state.case is not None:
                # Dokud se robot umí chytit na umělý segment, jezdí po něm
                return state
            else:
                print(f"[PathTracker] Ztracen i umělý segment. Přepočítávám novou umělou úsečku k cíli.")
                self.artificial_segment = False
                return self.update(R_lat, R_lon)
