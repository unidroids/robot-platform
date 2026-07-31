import json
import math
from dataclasses import dataclass
from near_waypoint import NearWaypoint, NearState

@dataclass
class Waypoint:
    lat: float
    lon: float

class PathTracker:
    def __init__(self, route_json_path: str, L_near_m: float = 1.0):
        self.waypoints = []
        self.L_near_m = L_near_m
        
        try:
            with open(route_json_path, 'r') as f:
                data = json.load(f)
                for wp in data.get("waypoints", []):
                    self.waypoints.append(Waypoint(lat=wp["lat"], lon=wp["lon"]))
            print(f"[PathTracker] Načteno {len(self.waypoints)} waypointů.")
        except Exception as e:
            print(f"[PathTracker] Chyba při načítání cesty: {e}")
            
        self.current_wp_index = 0
        self.active_near_wp = None
        self.artificial_segment = False
        
        if len(self.waypoints) >= 2:
            self._update_active_wp(0)

    def _update_active_wp(self, index: int, S_lat=None, S_lon=None, E_lat=None, E_lon=None):
        if S_lat is not None and E_lat is not None:
            self.active_near_wp = NearWaypoint(S_lat, S_lon, E_lat, E_lon, L_near_m=self.L_near_m)
            self.artificial_segment = True
            print(f"[PathTracker] Vytvořena UMĚLÁ úsečka: S=({S_lat:.6f}, {S_lon:.6f}) -> E=({E_lat:.6f}, {E_lon:.6f})")
        else:
            self.current_wp_index = index
            S = self.waypoints[index]
            E = self.waypoints[index + 1]
            self.active_near_wp = NearWaypoint(S.lat, S.lon, E.lat, E.lon, L_near_m=self.L_near_m)
            self.artificial_segment = False
            print(f"[PathTracker] Změna na segment {index}: S=({S.lat:.6f}, {S.lon:.6f}) -> E=({E.lat:.6f}, {E.lon:.6f})")

    def update(self, R_lat: float, R_lon: float) -> NearState:
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
                
            print(f"[PathTracker] Ztráta průniku na aktuálním segmentu {self.current_wp_index} (d_perp={state.d_perp_m:.2f}m). Spouštím FÁZI 2 (hledání).")
        
        else:
            # --- Řešení pro umělý segment ---
            self.active_near_wp.update(R_lat, R_lon)
            state = self.active_near_wp.state
            
            # Pokud dojedeme na konec umělého segmentu (např. do 0.5m)
            if state.distance_to_goal_m is not None and state.distance_to_goal_m < 0.5:
                print(f"[PathTracker] Umělý segment projet. Vracím se k normálnímu hledání.")
                self.artificial_segment = False
            elif state.case is not None:
                # Dokud se robot umí chytit na umělý segment, jezdí po něm
                return state
            else:
                print(f"[PathTracker] Ztracen i umělý segment. Přepočítávám.")
                self.artificial_segment = False

        # --- FÁZE 2: Iterace přes všechny segmenty ---
        best_idx = -1
        min_dist = float('inf')
        
        for i in range(len(self.waypoints) - 1):
            nw = NearWaypoint(self.waypoints[i].lat, self.waypoints[i].lon, 
                              self.waypoints[i+1].lat, self.waypoints[i+1].lon, 
                              L_near_m=self.L_near_m)
            nw.update(R_lat, R_lon)
            st = nw.state
            
            if st.d_perp_m is not None and st.d_perp_m < min_dist:
                min_dist = st.d_perp_m
                best_idx = i
                
        if best_idx != -1:
            print(f"[PathTracker] Nejbližší nalezený segment je {best_idx} s kolmou vzdáleností {min_dist:.2f}m.")
            
            # Zkusíme jestli na něm máme průsečík
            nw_best = NearWaypoint(self.waypoints[best_idx].lat, self.waypoints[best_idx].lon, 
                              self.waypoints[best_idx+1].lat, self.waypoints[best_idx+1].lon, 
                              L_near_m=self.L_near_m)
            nw_best.update(R_lat, R_lon)
            
            if nw_best.state.case is not None:
                print(f"[PathTracker] Průsečík existuje, obnovuji trasu na segmentu {best_idx}.")
                self._update_active_wp(best_idx)
                return nw_best.state
            else:
                print(f"[PathTracker] Průsečík na segmentu {best_idx} neexistuje (příliš daleko). Vytvářím umělou úsečku.")
                # Najdeme nejbližší waypoint z tohoto segmentu
                S = self.waypoints[best_idx]
                E = self.waypoints[best_idx+1]
                
                # Zjednodušená vzdálenost (zde by šlo použít haversine, ale hypot stačí na malé vzdálenosti)
                dist_S = math.hypot(S.lat - R_lat, S.lon - R_lon)
                dist_E = math.hypot(E.lat - R_lat, E.lon - R_lon)
                target = S if dist_S < dist_E else E
                
                self._update_active_wp(-1, S_lat=R_lat, S_lon=R_lon, E_lat=target.lat, E_lon=target.lon)
                self.active_near_wp.update(R_lat, R_lon)
                return self.active_near_wp.state
                
        return None
