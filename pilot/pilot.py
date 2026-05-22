# pilot.py
from __future__ import annotations
import threading
import time
import math 
import traceback
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

from drive_client import DriveClient
from fusion_client import FusionClient
from data.nav_fusion_data import NavFusionData

from geo_utils import heading_gnss_to_enu, lla_to_ecef, ecef_to_enu
from near_waypoint import NearWaypoint
from data.waypoints_data import WayPointsData, Waypoint

from data_loger import DataLogger

# ---------------------- util -----------------------------

@staticmethod
def _wrap_angle_deg(a: float) -> float:
    """wrap to [-180,180]"""
    a = (a + 180.0) % 360.0 - 180.0
    return a

@staticmethod
def _sign(x: float) -> int:
    return (x > 0) - (x < 0)  # -1, 0, +1      

@dataclass
class PilotState:
    mode: str = "IDLE"                 # IDLE | NAVIGATE | GOAL_REACHED | GOAL_NOT_REACHED
    near_case: str = "N/A"             # TWO_INTERSECTIONS | TANGENT | NO_INTERSECTION | N/A
    #dist_to_goal_m: float = 0.0
    #cross_track_m: float = 0.0         # kolmá vzdálenost k přímce S–E (telemetrie)
    #left_pwm: int = 0
    #right_pwm: int = 0
    #heading_enu_deg: float = 0.0
    last_note: str = ""
    ts_mono: float = 0.0               # monotonic timestamp poslední aktualizace

class Pilot:

    VERSION = "1.2.0"

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.fusion_client: Optional[FusionClient] = None
        self.drive_client: Optional[DriveClient] = None

        self._state_lock = threading.Lock()
        self._state = PilotState()

    # ---------------------- stavové API ----------------------

    def _set_state(self, **updates) -> None:
        with self._state_lock:
            for k, v in updates.items():
                setattr(self._state, k, v)
            self._state.ts_mono = time.monotonic()

    def get_state(self) -> dict:
        with self._state_lock:
            return asdict(self._state)

    # ---------------------- lifecycle ------------------------

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if not self._initialized:
                self.drive_client = DriveClient()
                self.fusion_client = FusionClient()
                self._initialized = True

            self.drive_client.connect()
            self.fusion_client.connect()

            self.running = True
            self._set_state(mode="IDLE", near_case="N/A", last_note="SERVICE STARTED")
            print("[SERVICE] STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "OK WAS NOT RUNNING"
            self._stop_event.set()
            if self._thread:
                self._thread.join(timeout=2.0)
            try:
                if self.fusion_client:
                    self.fusion_client.disconnect()
                if self.drive_client:
                    self.drive_client.send_motors_off()
                    self.drive_client.disconnect()
            finally:
                self.drive_client = None
                self.fusion_client = None
                self._initialized = False
                self.running = False
                self._set_state(mode="IDLE", near_case="N/A", last_note="SERVICE STOPPED")
                print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

  

    # ---------------------- navigační vlákno -----------------

    def _navigate_thread(self, fusion: FusionClient, drive: DriveClient, route: WayPointsData, goal_radius: float):
        """
        Navigační smyčka:
        - čte GNSS NavFusionData (10 Hz)
        - vybere near point na aktuálním úseku (segmentu) routy s L_near
        - vyhodnotí waypoint/cíl a případně přepne na další segment
        - vypočte korekci rychlostí kol
        - odešle rychlosti kol do služby drive
        """
        GOAL_RADIUS = float(goal_radius)

        if not route or len(route.waypoints) < 2:
            print("[PILOT] Error: Route must contain at least 2 waypoints.")
            return

        L_NEAR = 2  # lookahead pro near point (m)
        B = 0.58     # rozchod kol (m)

        total_waypoints = len(route.waypoints)
        current_idx = 0

        def init_segment(idx):
            S = route.waypoints[idx]
            E = route.waypoints[idx+1]
            print(f"[PILOT] Navigating segment {idx+1}/{total_waypoints-1}: "
                  f"from (lat={S.lat:.6f}, lon={S.lon:.6f}) to (lat={E.lat:.6f}, lon={E.lon:.6f})")
            return NearWaypoint(S_lat=S.lat, S_lon=S.lon, E_lat=E.lat, E_lon=E.lon, L_near_m=L_NEAR)

        nearwaypoint = init_segment(current_idx)


        max_erros = 5
        error_count = 0

        self._set_state(mode="NAVIGATE", near_case="N/A", last_note="Navigation started")
        resp = drive.send_motors_on()
        #resp = drive.send_break()
        left_speed, right_speed = 0.0, 0.0
        
        #heading_one_wheeel_comp_deg = math.atan2(0.3, (B/2)) * (180.0 / math.pi)  # small angle approx
        #heading_comp_deg = 0.0
        #smooth_heading_comp_deg = 0.0

        distance_to_goal_m, abs_distance_to_goal_m, heading_to_near_gnss_deg = 0.0, 0.0, 0.0
        heading_error = 0.0
        kappa = 0.0
        drive_mode = "N/A"

        log = DataLogger()

        # print cvs header 
        log.print(
              "ts_mono," # timestamp
              "lat,lon,hAcc," # position
              "raw_heading,smoot_heading,heading_acc,cumulated_angleZ," # heading
              "raw_speed,smooth_speed,speed_acc," # speed
              "last_gyroZ,smooth_gyroZ,gyroZ_acc," # gyroZ
              "gnssFixOK,drUsed," # fix types
              "segment_idx,distance_to_goal_m,abs_distance_to_goal_m,heading_to_near_gnss_deg," # near point
              "heading_error_deg," # heading error
              "left_speed,right_speed,kappa,drive_mode," # drive commands
              "heading_comp_deg,smooth_heading_comp_deg" # heading compensation
        )

        last_loop = time.monotonic()
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                loop_dt_ms = (loop_start - last_loop) * 1000.0
                dt_s = max((loop_start - last_loop), 1e-3)
                last_loop = loop_start

                # 1) Načti Nav Fusion Data
                nav: Optional[NavFusionData] = fusion.read_nav_fusion_data() # blokuj max 1s
                if not nav:
                    drive.send_break()
                    print("[PILOT] No nav data -> sending BREAK")
                    continue
                #print(f"[PILOT] Nav data: lat={nav.lat}, lon={nav.lon}, heading={nav.heading}, speed_m={nav.speed}, gnssFixOK={nav.gnssFixOK}, drUsed={nav.drUsed} ")
                #print(f"[PILOT] Nav data: lat={nav.lat:12.8f}, lon={nav.lon:12.8f}, heading={nav.heading:6.2f}, speed_m={nav.speed*100:6.2f}, gnssFixOK={int(nav.gnssFixOK)}, drUsed={int(nav.drUsed)}")
                
                #continue
                #print("recieved",nav.to_json())
                
                # print telemetry csv line
                log.print(
                    # timestamp "ts_mono,"
                    f"{nav.ts_mono:.3f}," 
                    # position  "lat,lon,hAcc" 
                    f"{nav.lat:.8f},{nav.lon:.8f},{nav.hAcc:.2f}," # position
                    # heading "raw_heading,smoot_heading(odo_angle),heading_acc,cumulated_angleZ," 
                    f"{nav.heading:.2f},{nav.heading:.2f},{nav.headingAcc:.2f},{nav.heading:.2f}," # heading
                    # speed "raw_speed,smooth_speed,speed_acc," 
                    f"{nav.speed:.2f},{nav.speed:.2f},{nav.sAcc:.2f}," # speed
                    # gyroZ "last_gyroZ,smooth_gyroZ(odo_gyro),gyroZ_acc," 
                    f"{nav.gyroZ:.2f},{nav.gyroZ:.2f},{nav.gyroZAcc:.2f}," # gyroZ
                    # fix types "gnssFixOK,drUsed," 
                    f"{int(nav.gnssFixOK)},{int(nav.drUsed)}," # fix types
                    # near point "segment_idx,distance_to_goal_m,abs_distance_to_goal_m,heading_to_near_gnss_deg," 
                    f"{current_idx},{distance_to_goal_m:.2f},{abs_distance_to_goal_m:.2f},{heading_to_near_gnss_deg:.2f},"
                    # heading error "heading_error_deg,"
                    f"{heading_error:.2f},"
                    # drive commands "left_speed,right_speed,kappa,drive_mode," 
                    f"{left_speed:.2f},{right_speed:.2f},{kappa:.4f},{drive_mode},"
                    # heading compensation "heading_comp_deg,smooth_heading_comp_deg," 
                    #f"{heading_comp_deg:.2f},{smooth_heading_comp_deg:.2f}"
                )

                # 2) Zjisti near point
                (distance_to_goal_m, abs_distance_to_goal_m, heading_to_near_gnss_deg) = nearwaypoint.update(R_lat=nav.lat, R_lon=nav.lon)

                # 3) Ověř zda jsi v cíli aktuálního segmentu (i pokud jsme jej lehce přejeli)
                if abs_distance_to_goal_m <= GOAL_RADIUS or distance_to_goal_m < 0.0:
                    if current_idx + 1 < total_waypoints - 1:
                        current_idx += 1
                        nearwaypoint = init_segment(current_idx)
                        print(f"[PILOT] Waypoint {current_idx} reached, switching to next segment. (dist: {abs_distance_to_goal_m:.2f} m)")
                        continue
                    else:
                        print(f"[PILOT] Final goal reached -> stop. Distance to goal: {distance_to_goal_m:.2f} m")
                        drive.send_break()
                        self._set_state(mode="GOAL_REACHED", last_note="Goal reached")
                        break
                
                # 4) Ověř zda existje near point
                if not heading_to_near_gnss_deg:
                    print(f"[PILOT] No near point found -> sending BREAK. Distance to goal: {abs_distance_to_goal_m:.2f} m")
                    drive.send_break()
                    self._set_state(mode="GOAL_NOT_REACHED", near_case="NO_INTERSECTION", last_note="No near point found")
                    break

                # 5) Spočti chybu heading robota vůči near point
                heading_error = _wrap_angle_deg(heading_to_near_gnss_deg - nav.heading) 

                # 6) Plynulý výpočet rychlostí kol
                max_fwd_speed = 100.0  # cm/s (maximální dopředná rychlost)
                max_spin_speed = 40.0 # cm/s (maximální rychlost kola při točení na místě)
                a_y_max = 1.0         # m/s^2 (limit bočního zrychlení)
                
                err_abs = abs(heading_error)
                
                # a) Zpomalování u cíle (začínáme brzdit 3 metry od cíle)
                dist_factor = 1.0
                if abs_distance_to_goal_m < 3.0:
                    dist_factor = max(0.2, abs_distance_to_goal_m / 3.0)
                
                # b) Dopředná rychlost (v_center) klesá s chybou natočení
                if err_abs < 15.0:
                    v_center = max_fwd_speed
                elif err_abs < 60.0:
                    v_center = max_fwd_speed * (1.0 - (err_abs - 15.0) / 45.0)
                else:
                    v_center = 0.0
                    
                v_center *= dist_factor

                # Pure Pursuit geometrie (kappa je požadované zakřivení cesty)
                alpha_rad = math.radians(-heading_error)
                kappa = 2.0 * math.sin(alpha_rad) / L_NEAR
                
                # Limit dopředné rychlosti vůči bočnímu zrychlení
                if abs(kappa) > 0.001:
                    v_ay_limit = math.sqrt(a_y_max / abs(kappa)) * 100.0 # převod m/s -> cm/s
                    v_center = min(v_center, v_ay_limit)

                # c) Rotační rychlost (v_turn) jako plynulý blend mezi Pure Pursuit a otáčením na místě
                # 1. Pure Pursuit (skvělé pro sledování plynulé křivky v pohybu)
                v_turn_pp = v_center * kappa * B / 2.0
                
                # 2. Spin / točení na místě (P-regulátor pro velké úhly)
                spin_mag = max_spin_speed * min(1.0, err_abs / 60.0)
                v_turn_spin = math.copysign(spin_mag, -heading_error)
                
                # 3. Smíchání (blend_factor je 0 pro úhel 0°, 1 pro úhel >= 60°)
                blend_factor = min(1.0, err_abs / 60.0)
                v_turn = (1.0 - blend_factor) * v_turn_pp + blend_factor * v_turn_spin
                
                drive_mode = "BLENDED"
                
                # d) Mix do rychlostí kol
                left_speed = v_center - v_turn
                right_speed = v_center + v_turn
                
                # e) Absolutní omezení rychlostí, abychom nepřekročili limity motorů
                abs_limit = 100.0 # cm/s
                max_wheel = max(abs(left_speed), abs(right_speed))
                if max_wheel > abs_limit:
                    left_speed = (left_speed / max_wheel) * abs_limit
                    right_speed = (right_speed / max_wheel) * abs_limit

                left_speed = round(left_speed)
                right_speed = round(right_speed)

                # smooth heading compensation
                #smooth_heading_comp_deg = 0.8 * smooth_heading_comp_deg + 0.2 * heading_comp_deg
                #print(f"[PILOT] Heading error: {heading_error:.2f} deg, heading_comp: {heading_comp_deg:.2f} deg, smooth_comp: {smooth_heading_comp_deg:.2f} deg")

                # 7) Odešli rychlosti kol do drive služby
                pwm = 100 # pevné PWM pro nyní  (left_speed + right_speed)
                result = drive.send_drive(pwm, left_speed, right_speed)
                #result = drive.send_drive(pwm, 40, -40) # testovací pevná rychlost
                #result = drive.send_drive(pwm, -30, 30) # testovací pevná rychlost
                #result = drive.send_pwm(1,1) # testovací duty cycle
                print("[Pilot]", pwm, left_speed, right_speed, result, heading_error, loop_dt_ms)
                #print(f"[PILOT] Drive command sent: PWM={pwm}, left_speed={left_speed} cm/s, right_speed={right_speed} cm/s")
                
                # 8) Handle chybu v komunikaci
                #if result != "OK":
                #    if result.startswith("ERROR RuntimeError: Command error CE=3"):
                #        pass
                #    elif result.startswith("ERROR RuntimeError: Command error CE=4"):
                #        pass
                #    else:
                #        raise RuntimeError(f"Drive service failed with result: {result}")

            except Exception as e:
                drive.send_break()
                print(f"[PILOT ERROR] {e}")
                traceback.print_exc()
                time.sleep(1.0)
                error_count += 1
                if error_count >= max_erros:
                    print("[PILOT] Maximum error count reached, stopping navigation.")
                    self._set_state(mode="GOAL_NOT_REACHED", last_note="Max errors reached")
                    try:
                        drive.send_break()
                    except:
                        pass    
                    break

        drive.send_break()
        time.sleep(0.2)
        drive.send_motors_off()
        print("[PILOT] Navigation ended.")
        

    # ---------------------- API pro řízení -------------------

    def navigate_way_points(self, route: WayPointsData, goal_radius: float = 1.0):
        fusion = self.fusion_client
        drive = self.drive_client
        with self._lock:
            self._ensure_running()
            if self._thread and self._thread.is_alive():
                self._stop_event.set()
                self._thread.join()
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._navigate_thread,
                args=(fusion, drive, route, goal_radius),
                daemon=True
            )
            self._thread.start()


    def navigate_to_point(self, start_lat, start_lon, goal_lat, goal_lon, goal_radius:float = 1.0):
        # Zabalené pro zpětnou kompatibilitu do struktury route o 2 bodech
        route = WayPointsData(
            waypoints=[
                Waypoint(lat=float(start_lat), lon=float(start_lon), curvature=0.0, path_width_m=0.0, rel_azimuth_deg=0.0, corridors=[]),
                Waypoint(lat=float(goal_lat),  lon=float(goal_lon),  curvature=0.0, path_width_m=0.0, rel_azimuth_deg=0.0, corridors=[])
            ]
        )
        self.navigate_way_points(route, float(goal_radius))


if __name__ == "__main__":
    print(f"Sign test {_sign(-30)} {_sign(0)} {_sign(30)}") 
    try:
        pilot = Pilot()
        pilot.start()
        pilot.navigate(
            start_lat=50.0615486,
            start_lon=14.5996717,
            goal_lat=50.0615486,
            goal_lon=14.5996717+0.00002,
            goal_radius=2.0
        )
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        pilot.stop()