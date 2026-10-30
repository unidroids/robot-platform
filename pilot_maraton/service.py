import os
import threading
import time
import math
import json
import base64
from datetime import datetime
from pathlib import Path
import zmq
import asyncio

try:
    from .path_tracker import PathTracker
    from .drive_client import DriveClient
    from .data_logger import DataLogger
    from .geo_utils import rrp_to_nose
    from .corridor_planner import RouteCorridor, ObstacleAvoidanceController
except (ImportError, ValueError):
    from path_tracker import PathTracker
    from drive_client import DriveClient
    from data_logger import DataLogger
    from geo_utils import rrp_to_nose
    from corridor_planner import RouteCorridor, ObstacleAvoidanceController


FRONT_OFFSET_M = 0.40      # Předsazení čumáku robota před středem otáčení (RRP) v metrech
ROBOT_RADIUS_CM = 37.5     # Poloměr válce těla robota (šířka 60 cm + 2x 7.5 cm rezerva = 75 cm)
STOP_DISTANCE_CM = 70.0    # Minimální vzdálenost zastavení před překážkou (0.7 m)
FREE_DISTANCE_CM = 600.0   # Vzdálenost volného prostoru pro plnou rychlost 2.0 m/s (6.0 m)
ABS_MAX_SPEED_CMS = 200.0  # Absolutní maximální rychlost robota (2.0 m/s = 200 cm/s)


class MaratonPilotService:
    """
    Služba PILOT-MARATON:
    Autonomní řízení robota podle waypointů pro soutěžní misi Maraton,
    s budoucím zapojením antikolize z LiDARu (kolizní kurzy), detekce krajnic,
    bezpečnostního dohledu OOW a fúzovaných pozičních dat.
    """

    def __init__(self):
        self.state = "IDLE"  # IDLE, RUNNING, PAUSED, STOPPED, FINISHED
        self.status_info = ""
        self.source = ""
        
        self.max_speed = 170
        self.max_pwm = 240
        
        # Limity zrychlení (v jednotkách rychlosti cm/s za iteraci, tedy per 0.1s)
        self.max_fwd_accel_step = 20.0       # 2.0 m/s^2 vpřed
        self.nominal_brk_accel_step = 20.0   # 2.0 m/s^2 predikovatelné brzdění
        self.max_brk_accel_step = 40.0       # 4.0 m/s^2 (0.4g) nouzové brzdění
        self.active_brk_accel_step = 20.0
        self.max_ang_accel_step = 30.0
        
        # Odstředivé (boční) zrychlení v m/s^2 a rozchod kol v mm
        self.max_centrifugal_accel = 0.5 
        self.wheelbase_mm = 530.0
        
        self.last_v = 0.0
        self.last_w = 0.0
        self.current_speed = 0.0
        
        self.path_tracker = None
        self.drive = DriveClient()
        self.drive.connect()
        
        self.logger = None
        
        self.running = False
        self.control_thread = None
        self.zmq_thread = None
        
        self.fusion_data = None
        self.lidar_distance = -1.0
        self.last_lidar_time = 0.0
        self.collision_data = None
        self.last_collision_time = 0.0
        self.min_ttc_s = 999.0
        self.min_ttc_angle_deg = -1.0
        self.traversability_data = None
        self.traversability_grid = None
        self.last_traversability_time = 0.0
        self.traversability_corridor_cm = -1.0

        # Navigační podstav pro vyhýbání překážkám a synchronizaci s mission_maraton
        self.nav_state = "IDLE"
        self.nav_info = ""
        self.lateral_offset_m = 0.0
        self.corridor_planner: ObstacleAvoidanceController | None = None

        # ZMQ PUB pro okamžité hlášení změn stavu (ipc:///tmp/robot-pilot-maraton, topic STATE)
        self._pub_lock = threading.Lock()
        self._zmq_pub_ctx = None
        self._zmq_pub = None
        self._zmq_pub_failed = False
        self._last_published_sig = None
        self._last_publish_ts = 0.0
        
        # OOW state
        self.oow_tcp_ok = False
        self.oow_zmq_ok = True  # Default true dokud nepřijde OFF
        
        self.last_distance_to_goal = 0.0
        self.receiver = None
        self.oow_task = None

    def _ensure_state_publisher(self):
        if self._zmq_pub is not None or self._zmq_pub_failed:
            return
        pub = None
        try:
            self._zmq_pub_ctx = zmq.Context.instance()
            pub = self._zmq_pub_ctx.socket(zmq.PUB)
            pub.setsockopt(zmq.LINGER, 0)
            pub.bind("ipc:///tmp/robot-pilot-maraton")
            self._zmq_pub = pub
        except Exception as e:
            if pub is not None:
                try:
                    pub.close(linger=0)
                except Exception:
                    pass
            self._zmq_pub_failed = True
            print(f"[PilotMaraton] ZMQ PUB bind info (ipc:///tmp/robot-pilot-maraton): {e}")

    def _publish_state_event(self, force: bool = False):
        now = time.time()
        sig = (self.state, self.nav_state, self.source)
        if not force and sig == self._last_published_sig and (now - self._last_publish_ts) < 1.0:
            return
        self._last_published_sig = sig
        self._last_publish_ts = now
        with self._pub_lock:
            self._ensure_state_publisher()
            if self._zmq_pub is not None:
                try:
                    payload = self.get_status().encode("utf-8")
                    self._zmq_pub.send_multipart([b"STATE", payload], flags=zmq.NOBLOCK)
                except Exception:
                    pass

    def _set_nav_state(self, nav_state: str, nav_info: str, lateral_offset_m: float = 0.0):
        prev_nav = self.nav_state
        self.nav_state = nav_state
        self.nav_info = nav_info
        self.lateral_offset_m = float(lateral_offset_m)
        if prev_nav != nav_state:
            print(f"[PilotMaraton] Změna nav_state: {prev_nav} -> {nav_state} ({nav_info})")
            self._publish_state_event(force=True)
        else:
            self._publish_state_event(force=False)

    def start_service(self, max_speed=100, max_pwm=150, route_input=None) -> tuple[bool, str]:
        if self.state in ["RUNNING"]:
            return False, "ERR: ALREADY RUNNING"
            
        if route_input is None:
            return False, "ERR: START vyžaduje JSON payload s trasou"

        # Inicializace a validace trasy přes PathTracker + RouteCorridor
        try:
            tracker = PathTracker(route_input, L_near_m=2.0)
            self.path_tracker = tracker
            corridor = RouteCorridor(tracker.waypoints)
            self.corridor_planner = ObstacleAvoidanceController(corridor)
        except Exception as e:
            print(f"[PilotMaraton] Chyba inicializace trasy: {e}")
            return False, f"ERR: {e}"

        print(f"[PilotMaraton] START: max_speed={max_speed}, max_pwm={max_pwm}, waypoints={len(self.path_tracker.waypoints)}")
        self.max_speed = min(float(max_speed), ABS_MAX_SPEED_CMS)
        self.max_pwm = max_pwm
        
        # Reset speeds
        self.last_v = 0.0
        self.last_w = 0.0
        self.current_speed = 0.0
        self.last_distance_to_goal = 0.0
        
        # Init logger
        self.logger = DataLogger(base_dir="/data/robot/pilot_maraton")
        self.logger.print("time,lat,lon,nose_lat,nose_lon,heading,target_heading,heading_error,distance_to_goal_m,d_perp_m,wp_index,target_left,target_right,actual_left,actual_right,obstacle_distance_cm,h_acc_mm,state,source,reason,target_wp_index,target_wp_lat,target_wp_lon,near_lat,near_lon,closest_lat,closest_lon,lookahead_dist_m,geom_curvature,geom_radius_m,geom_turn_direction")
        
        # Uložení do /data/robot/pilot_maraton/<yyyy-mm-dd>/<HH-MM-SS>/route.json
        now = datetime.now()
        base_dir = Path("/data/robot/pilot_maraton") / now.strftime("%Y-%m-%d") / now.strftime("%H-%M-%S")
        data_to_dump = route_input if isinstance(route_input, (dict, list)) else json.loads(route_input)
        try:
            base_dir.mkdir(parents=True, exist_ok=True)
            route_file = base_dir / "route.json"
            with open(route_file, "w", encoding="utf-8") as f:
                json.dump(data_to_dump, f, indent=2, ensure_ascii=False)
            print(f"[PilotMaraton] Uložena kopie trasy do: {route_file}")
        except (PermissionError, OSError):
            local_dir = Path(os.path.dirname(os.path.abspath(__file__))) / "data" / "robot" / "pilot_maraton" / now.strftime("%Y-%m-%d") / now.strftime("%H-%M-%S")
            try:
                local_dir.mkdir(parents=True, exist_ok=True)
                route_file = local_dir / "route.json"
                with open(route_file, "w", encoding="utf-8") as f:
                    json.dump(data_to_dump, f, indent=2, ensure_ascii=False)
                print(f"[PilotMaraton] Uložena kopie trasy (fallback) do: {route_file}")
            except Exception as e:
                print(f"[PilotMaraton] Chyba při ukládání route.json: {e}")

        self.state = "RUNNING"
        self.source = "USER"
        self.status_info = "Starting"
        self.nav_state = "CRUISING"
        self.nav_info = "Jízda podle trasy"
        self.lateral_offset_m = 0.0
        self._publish_state_event(force=True)
        
        # WORKAROUND: Chyba ve firmware hoverboardu
        # Firmware vyžaduje po příkazu START probuzení regulace motorů nenulovou rychlostí
        # (speed=1) s nízkým PWM (pwm=1) a následné vynulování rychlosti (speed=0).
        try:
            self.drive.send_start()
            self.drive.send_drive(1, 1, 1)
            time.sleep(0.05)
            self.drive.send_drive(1, 0, 0)
        except Exception as e:
            print(f"[PilotMaraton] Varování: Inicializace DRIVE (workaround firmware) selhala: {e}")
        
        if not self.running:
            self.running = True
            
            # ZMQ Receiver
            self.zmq_thread = threading.Thread(target=self._zmq_loop, daemon=True)
            self.zmq_thread.start()
            
            # Control Loop
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
        
        return True, "OK"

    def stop_service(self):
        print("[PilotMaraton] Zastavuji službu (zpomaluji na 0)...")
        if self.state != "STOPPED":
            self.state = "STOPPED"
            self.source = "USER"
            self.status_info = "Zastaveno příkazem (STOP)"
            self._publish_state_event(force=True)
        return "OK"

    def pause_service(self, source="USER", info=""):
        if self.state != "PAUSED" and self.state != "STOPPED":
            if not info:
                info = "Pozastaveno uživatelem (PAUSE)"
            print(f"[PilotMaraton] PAUSE od {source}: {info}")
            self.state = "PAUSED"
            self.source = source
            self.status_info = info
            self._publish_state_event(force=True)
        return "OK"
        
    def resume_service(self, source="USER", info=""):
        if self.state == "PAUSED":
            if not info:
                info = "Obnoveno uživatelem (RESUME)"
            print(f"[PilotMaraton] RESUME od {source}: {info}")
            self.state = "RUNNING"
            self.source = source
            self.status_info = info
            self._publish_state_event(force=True)
        return "OK"
        
    def get_status(self):
        wp_index = self.path_tracker.current_wp_index if self.path_tracker else 0
        wp_total = len(self.path_tracker.waypoints) if self.path_tracker else 0
        dist = round(float(self.last_distance_to_goal), 2) if self.last_distance_to_goal is not None else 0.0

        # Cílová / požadovaná rychlost pilota v m/s:
        # self.current_speed je interně v cm/s (např. 100 cm/s = 1.0 m/s nebo 120 cm/s = 1.2 m/s)
        raw_cmd_speed = float(self.current_speed) if hasattr(self, 'current_speed') and self.current_speed is not None else 0.0
        if raw_cmd_speed > 10.0:
            target_speed_ms = round(raw_cmd_speed / 100.0, 2)
        else:
            target_speed_ms = round(raw_cmd_speed, 2)

        # Aktuální reálná naměřená rychlost z fúze (odometrie podvozku / GNSS) v m/s:
        # fusion_data['speed'] je v mm/s (např. 850 mm/s = 0.85 m/s)
        actual_speed_ms = 0.0
        if self.fusion_data:
            raw_fusion_spd = float(self.fusion_data.get('speed', 0.0))
            if abs(raw_fusion_spd) > 20.0:
                actual_speed_ms = round(raw_fusion_spd / 1000.0, 2)
            else:
                actual_speed_ms = round(raw_fusion_spd, 2)

        # LiDAR obstacle distance & TTC (výhradně LIDAR-COLLISIONS)
        collisions_active = (time.time() - self.last_collision_time) < 2.0 and self.collision_data is not None
        lidar_active = collisions_active
        obstacle_distance_cm = round(float(self._get_effective_obstacle_distance_cm()), 1) if lidar_active else -1.0
        min_ttc_s = round(float(self.min_ttc_s), 2) if collisions_active else 999.0
        min_ttc_angle_deg = round(float(self.min_ttc_angle_deg), 1) if collisions_active else -1.0

        status_dict = {
            "state": self.state,
            "nav_state": self.nav_state,
            "nav_info": self.nav_info,
            "lateral_offset_m": round(float(self.lateral_offset_m), 2),
            "source": self.source,
            "info": self.status_info,
            "wp_index": wp_index,
            "wp_total": wp_total,
            "distance_to_goal_m": dist,
            "obstacle_distance_cm": obstacle_distance_cm,
            "min_ttc_s": min_ttc_s,
            "min_ttc_angle_deg": min_ttc_angle_deg,
            "lat": 0.0,
            "lon": 0.0,
            "heading": 0.0,
            "speed": actual_speed_ms,              # Zpětná kompatibilita (skutečná rychlost v m/s)
            "speed_actual": actual_speed_ms,       # Aktuální naměřená rychlost v m/s
            "speed_target": target_speed_ms,       # Cílová požadovaná rychlost v m/s
            "gps_sol": "NONE",
            "h_acc_mm": 9999
        }

        if self.fusion_data:
            status_dict["lat"] = self.fusion_data.get('lat', 0.0)
            status_dict["lon"] = self.fusion_data.get('lon', 0.0)
            status_dict["heading"] = self.fusion_data.get('heading', 0.0)
            status_dict["gps_sol"] = self.fusion_data.get('gpsSol', 'NONE')
            try:
                status_dict["h_acc_mm"] = int(round(float(self.fusion_data.get('hAcc', 9999))))
            except (ValueError, TypeError):
                status_dict["h_acc_mm"] = 9999
            if self.source == "GPS" and self.state == "PAUSED":
                status_dict["info"] = f"Nízká přesnost GPS (hAcc: {status_dict['h_acc_mm']} mm, sol: {status_dict['gps_sol']})"

        return json.dumps(status_dict, ensure_ascii=False)

    def shutdown(self):
        print(f"[PilotMaraton] SHUTDOWN")
        self.running = False
        self.drive.disconnect()
        with self._pub_lock:
            if self._zmq_pub is not None:
                try:
                    self._zmq_pub.close(linger=0)
                except Exception:
                    pass
                self._zmq_pub = None
        if self.logger:
            self.logger.close()
            self.logger = None

    def update_fusion(self, data):
        self.fusion_data = data
        
    def update_lidar(self, data):
        self.lidar_distance = data.get("distance", -1.0)
        self.last_lidar_time = time.time()

    def update_collisions(self, data):
        self.collision_data = data
        self.last_collision_time = time.time()
        self.min_ttc_s = float(data.get("min_ttc_s", 999.0))
        self.min_ttc_angle_deg = float(data.get("min_ttc_angle_deg", -1.0))

    def update_traversability(self, data):
        self.traversability_data = data
        self.last_traversability_time = time.time()
        self.traversability_corridor_cm = float(data.get("min_corridor_clearance_cm", -1.0))
        grid_b64 = data.get("grid_b64")
        if grid_b64:
            try:
                raw = base64.b64decode(grid_b64)
                if len(raw) == 6400:
                    self.traversability_grid = raw
            except Exception:
                pass

    def get_traversability_cost_at(self, x_fwd_m: float, y_cw_m: float, now_ts: float | None = None) -> int:
        """
        Vrátí potenciálovou cenu sjízdnosti (0..100, kde 100 = nesjízdné/neznámé)
        v lokálních souřadnicích robota (x_fwd_m in [-2.0, +6.0), y_cw_m in [-4.0, +4.0), +y = vpravo CW).
        Slouží jako podklad pro plánovač / Elastic Band.
        """
        now = time.time() if now_ts is None else now_ts
        if not self.traversability_grid or (now - self.last_traversability_time) >= 2.0:
            return 100
        ix = int(math.floor((x_fwd_m + 2.0) / 0.10))
        iy = int(math.floor((y_cw_m + 4.0) / 0.10))
        if ix < 0 or ix >= 80 or iy < 0 or iy >= 80:
            return 100
        return int(self.traversability_grid[ix * 80 + iy])

    def _get_forward_corridor_distance_cm(self) -> float:
        """
        Z 72 sektorů (po 5°, 0 = vpřed CW) v collision_data vrátí nejbližší
        vzdálenost překážky zasahující do přímého jízdního koridoru robota (šířka 75 cm).
        """
        if not self.collision_data:
            return -1.0
        distances = self.collision_data.get("distances_cm")
        if not distances or len(distances) != 72:
            return -1.0

        min_d = 2000.0
        found = False
        for b in list(range(0, 18)) + list(range(54, 72)):
            d = float(distances[b])
            if d < 0:
                continue
            found = True
            angle_deg = (b + 0.5) * 5.0 if b < 18 else (72 - b - 0.5) * 5.0
            lateral_cm = d * math.sin(math.radians(angle_deg))
            if b in (0, 71) or lateral_cm <= ROBOT_RADIUS_CM:
                if d < min_d:
                    min_d = d
        return min_d if found else -1.0

    def _get_effective_obstacle_distance_cm(self, now_ts: float | None = None) -> float:
        """Vrátí efektivní vzdálenost překážky před robotem (výhradně z LIDAR-COLLISIONS)."""
        now = time.time() if now_ts is None else now_ts
        collisions_active = (now - self.last_collision_time) < 2.0 and self.collision_data is not None

        if collisions_active:
            corr_d = self._get_forward_corridor_distance_cm()
            if corr_d >= 0.0:
                return corr_d

        return -1.0

    def _get_directional_obstacle_distance_cm(self, steer_rel_deg: float, now_ts: float | None = None) -> float:
        """
        Vrátí volnou vzdálenost podél plánovaného směru jízdy steer_rel_deg (ve stupních CW, -60..+60),
        se zachováním tvrdého předního nárazníku <= STOP_DISTANCE_CM (70 cm) v přímém směru
        (výhradně z LIDAR-COLLISIONS).
        Umožňuje plynulé vybočení do volného pruhu kolem překážky stojící na 1.45 m.
        """
        now = time.time() if now_ts is None else now_ts
        collisions_active = (now - self.last_collision_time) < 2.0 and self.collision_data is not None

        if not collisions_active:
            return -1.0

        steer_clamped = max(-60.0, min(60.0, float(steer_rel_deg)))
        candidates = []

        # 1. Tvrdý přední nárazník (pokud je cokoliv přímo před šasi <= 70 cm, okamžitý stop)
        fwd_bumper = self._get_forward_corridor_distance_cm()
        if 0.0 <= fwd_bumper <= STOP_DISTANCE_CM:
            candidates.append(fwd_bumper)

        # 2. Volná dráha ve směru zatáčení steer_clamped
        distances = self.collision_data.get("distances_cm")
        if distances and len(distances) == 72:
            min_steered = 2000.0
            found_s = False
            for b in list(range(0, 18)) + list(range(54, 72)):
                d = float(distances[b])
                if d < 0:
                    continue
                found_s = True
                sec_deg = (b + 0.5) * 5.0 if b < 18 else -((72 - b - 0.5) * 5.0)
                diff_deg = abs(sec_deg - steer_clamped)
                if diff_deg < 85.0:
                    lat_cm = d * math.sin(math.radians(diff_deg))
                    if diff_deg <= 3.0 or lat_cm <= ROBOT_RADIUS_CM:
                        if d < min_steered:
                            min_steered = d
            if found_s:
                candidates.append(min_steered)

        return min(candidates) if candidates else -1.0

    def _evaluate_collision_limits(self, base_target_v: float, obstacle_dist_cm: float, now_ts: float | None = None) -> tuple[float, float, str]:
        """
        Vypočítá omezení rychlosti a brzdné zrychlení podle vzdálenosti překážky a pravidla 2/5 TTC:
        - Zastavení na minimální vzdálenosti 0.7 m (70 cm).
        - Plná rychlost (max 2.0 m/s = 200 cm/s) při volném prostoru >= 6.0 m (600 cm).
        - Robot musí být schopen zastavit za 2/5 * min_ttc_s (predikovatelné brzdění 2 m/s^2, nouzové až 4 m/s^2).
        Vrací: (target_v_cms, brk_accel_step, status_reason_if_stopped)
        """
        now = time.time() if now_ts is None else now_ts
        target_v = min(float(base_target_v), ABS_MAX_SPEED_CMS)
        brk_step = self.nominal_brk_accel_step
        stop_reason = ""

        # 1. Omezení podle vzdálenosti v předním koridoru (0.7 m stop, 6.0 m plná rychlost 2 m/s)
        if 0.0 < obstacle_dist_cm <= STOP_DISTANCE_CM:
            target_v = 0.0
            brk_step = self.max_brk_accel_step
            stop_reason = f"Překážka před robotem ({obstacle_dist_cm:.0f} cm <= {STOP_DISTANCE_CM:.0f} cm)"
        elif STOP_DISTANCE_CM < obstacle_dist_cm < FREE_DISTANCE_CM:
            v_dist_limit = ABS_MAX_SPEED_CMS * ((obstacle_dist_cm - STOP_DISTANCE_CM) / (FREE_DISTANCE_CM - STOP_DISTANCE_CM))
            target_v = min(target_v, v_dist_limit)

        # 2. Omezení podle kolizního kurzu (pravidlo zastavení za 2/5 TTC)
        collisions_active = (now - self.last_collision_time) < 2.0 and self.collision_data is not None
        if collisions_active and 0.0 <= self.min_ttc_s < 900.0:
            t_stop_allowed = 0.4 * self.min_ttc_s  # 2/5 TTC [s]
            # Max rychlost při nominálním brzdění 2.0 m/s^2 (200 cm/s^2): v = a_nom * (2/5 * TTC) = 80 * TTC [cm/s]
            v_ttc_limit = 200.0 * t_stop_allowed
            if self.min_ttc_s <= 0.30 or v_ttc_limit < 10.0:
                v_ttc_limit = 0.0
                if not stop_reason:
                    stop_reason = f"Kolizní kurz (TTC {self.min_ttc_s:.2f} s, úhel {self.min_ttc_angle_deg:.0f}°)"
            target_v = min(target_v, v_ttc_limit)

            # Dynamické přizpůsobení brzdné decelerace (mezi 2.0 m/s^2 a nouzovými 4.0 m/s^2 = 0.4g)
            ref_speed = max(float(self.current_speed), float(self.last_v))
            if ref_speed > target_v:
                if t_stop_allowed <= 0.05:
                    brk_step = self.max_brk_accel_step
                else:
                    # Požadovaný krok za 0.1 s pro zastavení ze stávající rychlosti za 2/5 TTC
                    req_step = ref_speed / (t_stop_allowed * 10.0)
                    brk_step = min(self.max_brk_accel_step, max(self.nominal_brk_accel_step, req_step))

        return target_v, brk_step, stop_reason
        
    def update_oow_zmq(self, msg):
        print(f"[PilotMaraton] OOW ZMQ Event: {msg}")
        msg_upper = msg.strip().upper()
        if "OFF" in msg_upper or "PAUSE" in msg_upper:
            self.oow_zmq_ok = False
            self.pause_service(source="OOW_ZMQ", info="OOW dohled odpojen (timeout/ztráta BLE)")
        elif "ON" in msg_upper or "RESUME" in msg_upper:
            self.oow_zmq_ok = True
            if self.oow_tcp_ok and self.state == "PAUSED" and self.source == "OOW_ZMQ":
                self.resume_service(source="OOW_ZMQ", info="OOW dohled obnoven")
        elif "STOP" in msg_upper:
            self.oow_zmq_ok = False
            self.stop_service()

    def set_oow_tcp_ok(self, is_ok):
        if self.oow_tcp_ok != is_ok:
            print(f"[PilotMaraton] OOW TCP stav se změnil na: {'OK' if is_ok else 'FAIL'}")
        self.oow_tcp_ok = is_ok
        if not is_ok and self.state == "RUNNING":
            self.pause_service(source="OOW_TCP", info="OOW spojení přerušeno")
        elif is_ok and self.state == "PAUSED" and self.source == "OOW_TCP":
            if self.oow_zmq_ok:
                self.resume_service(source="OOW_TCP", info="OOW spojení obnoveno")

    def _zmq_loop(self):
        context = zmq.Context()
        sub = context.socket(zmq.SUB)
        connected = False
        try:
            sub.connect("ipc:///tmp/robot-fusion")
            sub.connect("ipc:///tmp/robot-lidar")
            sub.connect("ipc:///tmp/robot-lidar-collisions")
            sub.connect("ipc:///tmp/robot-lidar-traversability")
            sub.connect("ipc:///tmp/robot-oow")
            sub.setsockopt_string(zmq.SUBSCRIBE, "")
            connected = True
            print("[PilotMaraton] ZMQ Subscriber started.")
        except Exception as e:
            print(f"[PilotMaraton] ZMQ connect warning: {e}")
        
        while self.running and connected:
            try:
                parts = sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) == 2:
                    topic = parts[0].decode('utf-8', errors='ignore')
                    payload = parts[1].decode('utf-8', errors='ignore')
                    if topic == "SOLUTION":
                        self.update_fusion(json.loads(payload))
                    elif topic == "DISTANCE":
                        self.update_lidar(json.loads(payload))
                    elif topic == "COLLISIONS":
                        self.update_collisions(json.loads(payload))
                    elif topic == "TRAVERSABILITY":
                        self.update_traversability(json.loads(payload))
                    elif topic in ["STATUS", "CMD"]:
                        self.update_oow_zmq(payload)
                    else:
                        print(f"[PilotMaraton] Neznámý topic: {topic}")
                else:
                    first_frame = parts[0].decode('utf-8', errors='ignore') if len(parts) > 0 else "EMPTY"
                    print(f"[PilotMaraton] Neplatný formát zprávy (očekáváno len=2), přijato parts: {len(parts)}, první frame: '{first_frame}'")
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                print(f"[PilotMaraton] ZMQ chyba: {e}")
                time.sleep(0.1)
                
    def _calculate_steering(self, heading, target_heading, lidar_dist, current_speed):
        heading_error = target_heading - heading
        heading_error = (heading_error + 180) % 360 - 180
        
        kappa_v = current_speed * 1.5
        v_center = current_speed * math.exp(- (abs(heading_error)/45.0)**2)
        
        # Zpomalení dle LiDARu (mezi 50 a 180 cm)
        if 50.0 <= lidar_dist < 180.0:
            lidar_factor = max(0.1, (lidar_dist - 70.0) / 100.0)
            v_center *= lidar_factor
            
        v_turn_pp = heading_error * (kappa_v / 90.0)
        
        spin_mix = math.exp(- ((180 - abs(heading_error))/60.0)**4)
        v_spin = heading_error * (current_speed / 90.0)
        
        v_turn = (1.0 - spin_mix) * v_turn_pp + spin_mix * v_spin
        
        left = v_center + v_turn
        right = v_center - v_turn
        
        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))
        
        return left, right, heading_error

    def _apply_acceleration_limits(self, target_left, target_right):
        # 1. Kontrola odstředivého zrychlení (Centrifugal acceleration)
        v_target_m = ((target_left + target_right) / 2.0) / 1000.0  # m/s
        w_rads = (target_right - target_left) / self.wheelbase_mm   # rad/s
        a_c = abs(v_target_m * w_rads)                              # m/s^2
        
        if a_c > self.max_centrifugal_accel:
            k = math.sqrt(self.max_centrifugal_accel / a_c)
            target_left *= k
            target_right *= k

        # 2. Rozklad na V a W složky
        v_target = (target_left + target_right) / 2.0
        w_target = (target_left - target_right) / 2.0

        # Dopředné/brzdné zrychlení (změna V)
        brk_limit = getattr(self, "active_brk_accel_step", self.max_brk_accel_step)
        v_diff = v_target - self.last_v
        if v_diff > self.max_fwd_accel_step:
            v_out = self.last_v + self.max_fwd_accel_step
        elif v_diff < -brk_limit:
            v_out = self.last_v - brk_limit
        else:
            v_out = v_target

        # Úhlové zrychlení (změna W)
        w_diff = w_target - self.last_w
        if w_diff > self.max_ang_accel_step:
            w_out = self.last_w + self.max_ang_accel_step
        elif w_diff < -self.max_ang_accel_step:
            w_out = self.last_w - self.max_ang_accel_step
        else:
            w_out = w_target

        self.last_v = v_out
        self.last_w = w_out

        out_left = v_out + w_out
        out_right = v_out - w_out
        return int(out_left), int(out_right)

    def _run_control_step(self, now_ts: float | None = None) -> bool:
        """
        Provede jeden 10Hz krok řízení (navigace, koridorový plánovač, antikolize, regulace motorů a logování).
        Vrací False, pokud má řídicí smyčka skončit (STOPPED/FINISHED po zastavení kol).
        """
        now = time.time() if now_ts is None else float(now_ts)

        target_left = 0
        target_right = 0
        actual_left = 0
        actual_right = 0

        # Variables for logging
        lat = 0.0
        lon = 0.0
        nose_lat = 0.0
        nose_lon = 0.0
        heading = 0.0
        target_heading = 0.0
        heading_error = 0.0
        distance_to_goal = 0.0
        d_perp = 0.0
        near_state = None

        if self.fusion_data:
            lat = float(self.fusion_data.get("lat", 0.0))
            lon = float(self.fusion_data.get("lon", 0.0))
            heading = float(self.fusion_data.get("heading", 0.0))
            nose_lat, nose_lon = rrp_to_nose(lat, lon, heading, FRONT_OFFSET_M)

        collisions_active = (now - self.last_collision_time) < 2.0 and self.collision_data is not None
        lidar_active = collisions_active
        eff_dist = self._get_effective_obstacle_distance_cm(now_ts=now)
        current_lidar = round(float(eff_dist), 1) if (lidar_active and eff_dist >= 0.0) else -1.0

        if self.state in ["RUNNING", "PAUSED", "STOPPED", "FINISHED"]:
            target_v = self.max_speed if self.state == "RUNNING" else 0.0
            self.active_brk_accel_step = self.nominal_brk_accel_step

            # 1. Kontrola LiDARu - bez platných dat z LIDAR-COLLISIONS robot nesmí jet
            if not lidar_active or current_lidar < 0.0:
                target_v = 0.0
                self.active_brk_accel_step = self.max_brk_accel_step
                target_left, target_right = 0, 0
                if self.state == "RUNNING":
                    self.source = "LIDAR"
                    if self.last_collision_time == 0.0:
                        self.status_info = "Čekání na data z LIDAR-COLLISIONS"
                    else:
                        self.status_info = "Výpadek dat z LIDAR-COLLISIONS (timeout > 2s)"
                    self._set_nav_state("OBSTACLE", self.status_info, self.lateral_offset_m)
            # 2. Kontrola fúzních dat (GPS)
            elif not self.fusion_data:
                target_v = 0.0
                target_left, target_right = 0, 0
                if self.state == "RUNNING":
                    self.source = "GPS"
                    self.status_info = "Čekání na data z fúze (GPS)"
            else:
                hAcc = int(round(float(self.fusion_data.get("hAcc", 9999))))  # v mm
                heading_sol = self.fusion_data.get("headingSol", "NONE")
                heading_acc = float(self.fusion_data.get("headingAcc", 9999.0))

                if hAcc > 700 or heading_sol == "NONE" or heading_acc > 15.0:
                    info_msg = f"Nízká přesnost GPS (hAcc: {hAcc} mm, sol: {heading_sol})"
                    if self.state == "RUNNING":
                        self.pause_service(source="GPS", info=info_msg)
                    target_left, target_right = 0, 0
                else:
                    # Pokud byla chyba GPS odstraněna, obnovíme běh
                    if self.state == "PAUSED" and self.source == "GPS" and hAcc < 500 and heading_sol != "NONE" and heading_acc <= 12.0:
                        self.resume_service(source="GPS", info=f"Přesnost GPS obnovena (hAcc: {hAcc} mm, sol: {heading_sol})")

                    near_state = self.path_tracker.update(nose_lat, nose_lon)

                    if near_state is None:
                        if self.state != "FINISHED":
                            reached_end = self.path_tracker and self.path_tracker.current_wp_index >= len(self.path_tracker.waypoints) - 1
                            print("[PilotMaraton] Konec trasy dosažen. Přepínám na stav FINISHED (zpomaluji na 0).")
                            self.state = "FINISHED"
                            self.source = "PILOT"
                            self.status_info = "Cíl dosažen" if reached_end else "Trasa nenalezena"
                            self._set_nav_state("IDLE", self.status_info, 0.0)
                        target_left, target_right = 0, 0
                    else:
                        if near_state.distance_to_goal_m is not None and near_state.distance_to_goal_m < 0 and self.path_tracker.current_wp_index >= len(self.path_tracker.waypoints) - 2:
                            if self.state != "FINISHED":
                                print("[PilotMaraton] Konec trasy dosažen. Přepínám na stav FINISHED (zpomaluji na 0).")
                                self.state = "FINISHED"
                                self.source = "PILOT"
                                self.status_info = "Cíl dosažen"
                                self._set_nav_state("IDLE", self.status_info, 0.0)

                        if near_state.heading_to_near_gnss_deg is not None:
                            target_heading = near_state.heading_to_near_gnss_deg
                        distance_to_goal = near_state.distance_to_goal_m
                        self.last_distance_to_goal = distance_to_goal if distance_to_goal is not None else 0.0
                        d_perp = near_state.d_perp_m

                        # Korekce target_v na základě relativního azimutu (zpomalení do zatáčky)
                        if self.state == "RUNNING" and near_state.end_rel_azimuth_deg is not None and near_state.distance_to_goal_m is not None:
                            rel_az = near_state.end_rel_azimuth_deg
                            v_waypoint = self.max_speed * math.exp(- (abs(rel_az) / 45.0) ** 2)
                            v_waypoint = max(min(self.max_speed, 100.0), v_waypoint)
                            dist = max(0.0, near_state.distance_to_goal_m)
                            slowdown_dist = self.path_tracker.L_near_m
                            if dist < slowdown_dist:
                                target_v = v_waypoint + (self.max_speed - v_waypoint) * (dist / slowdown_dist)

                        # Koridorový plánovač a vyhodnocení statických/dynamických překážek
                        decision = None
                        if self.state == "RUNNING" and self.corridor_planner is not None:
                            decision = self.corridor_planner.step(
                                lat_rrp=lat,
                                lon_rrp=lon,
                                heading_deg=heading,
                                current_speed_cms=self.current_speed,
                                max_speed_cms=self.max_speed,
                                current_wp_index=self.path_tracker.current_wp_index,
                                default_target_heading_deg=target_heading,
                                collision_data=self.collision_data,
                                collisions_active=collisions_active,
                                traversability_active=False,
                                get_traversability_cost_fn=lambda x, y: 0,
                                now=now,
                            )

                            if decision.goal_reached_near_obstacle:
                                print(f"[PilotMaraton] {decision.nav_info}. Přepínám na FINISHED.")
                                self.state = "FINISHED"
                                self.source = "PILOT"
                                self.status_info = decision.nav_info
                                self._set_nav_state("IDLE", decision.nav_info, 0.0)
                                target_v = 0.0
                                target_left, target_right = 0, 0
                            else:
                                target_heading = decision.target_heading_deg
                                target_v = min(target_v, decision.speed_cap_cms)

                        # Antikolizní omezení rychlosti a zastavení (podél směru jízdy + 0.7 m nárazník + 2/5 TTC)
                        steer_rel_deg = (target_heading - heading + 180.0) % 360.0 - 180.0
                        ctrl_lidar_cm = self._get_directional_obstacle_distance_cm(steer_rel_deg, now_ts=now)
                        if ctrl_lidar_cm < 0.0:
                            ctrl_lidar_cm = current_lidar

                        if self.state == "RUNNING":
                            target_v, brk_step, stop_reason = self._evaluate_collision_limits(target_v, ctrl_lidar_cm, now_ts=now)
                            self.active_brk_accel_step = brk_step
                        else:
                            stop_reason = ""

                        if self.state == "RUNNING":
                            if target_v <= 0.0:
                                target_left, target_right = 0, 0
                                if decision is not None and decision.nav_state in ("WAITING_CLEARANCE", "NO_PATH", "OBSTACLE"):
                                    self.source = "NAV"
                                    self.status_info = decision.nav_info
                                    self._set_nav_state(decision.nav_state, decision.nav_info, decision.lateral_offset_m)
                                else:
                                    self.source = "LIDAR"
                                    info_txt = stop_reason if stop_reason else f"Překážka před robotem ({ctrl_lidar_cm:.0f} cm)"
                                    self.status_info = info_txt
                                    self._set_nav_state("OBSTACLE", info_txt, decision.lateral_offset_m if decision else 0.0)
                            else:
                                self.source = "NAV"
                                if decision is not None:
                                    self.status_info = decision.nav_info
                                    self._set_nav_state(decision.nav_state, decision.nav_info, decision.lateral_offset_m)
                                else:
                                    self.status_info = "Jízda podle trasy"
                                    self._set_nav_state("CRUISING", self.status_info, 0.0)
                                eff_steer_speed = max(self.current_speed, min(target_v, self.max_fwd_accel_step))
                                target_left, target_right, heading_error = self._calculate_steering(
                                    heading, target_heading, ctrl_lidar_cm, eff_steer_speed
                                )

            # Řízení požadované rychlosti (current_speed)
            if self.current_speed < target_v:
                self.current_speed = min(target_v, self.current_speed + self.max_fwd_accel_step)
            elif self.current_speed > target_v:
                self.current_speed = max(target_v, self.current_speed - self.active_brk_accel_step)

            # Fyzické limity zrychlení a odeslání do motorů
            actual_left, actual_right = self._apply_acceleration_limits(target_left, target_right)
            self.drive.send_drive(self.max_pwm, actual_left, actual_right)

        if self.state in ["RUNNING", "PAUSED", "STOPPED", "FINISHED"]:
            wp_idx = self.path_tracker.current_wp_index if self.path_tracker else 0
            h_acc_val = int(round(float(self.fusion_data.get("hAcc", 9999)))) if self.fusion_data else 9999
            reason_escaped = f'"{self.status_info}"'

            # Navigační sloupce
            target_wp_idx_str = ""
            target_wp_lat_str = ""
            target_wp_lon_str = ""
            near_lat_str = ""
            near_lon_str = ""
            closest_lat_str = ""
            closest_lon_str = ""
            lookahead_dist_str = ""
            geom_curv_str = ""
            geom_radius_str = ""
            geom_turn_dir_str = ""

            if self.source == "NAV" and self.state == "RUNNING" and near_state is not None and self.path_tracker:
                tgt_idx = min(self.path_tracker.current_wp_index + 1, len(self.path_tracker.waypoints) - 1)
                tgt_wp = self.path_tracker.waypoints[tgt_idx]
                target_wp_idx_str = str(tgt_idx)
                target_wp_lat_str = f"{tgt_wp.lat:.10f}"
                target_wp_lon_str = f"{tgt_wp.lon:.10f}"
                if near_state.near_lat is not None:
                    near_lat_str = f"{near_state.near_lat:.10f}"
                if near_state.near_lon is not None:
                    near_lon_str = f"{near_state.near_lon:.10f}"
                if near_state.closest_lat is not None:
                    closest_lat_str = f"{near_state.closest_lat:.10f}"
                if near_state.closest_lon is not None:
                    closest_lon_str = f"{near_state.closest_lon:.10f}"

                if near_state.near_x_m is not None and near_state.near_y_m is not None:
                    dist_to_near = math.hypot(near_state.near_x_m, near_state.near_y_m)
                else:
                    dist_to_near = self.path_tracker.L_near_m
                lookahead_dist_str = f"{dist_to_near:.3f}"

                alpha_rad = math.radians(heading_error)
                sin_alpha = math.sin(alpha_rad)
                if abs(sin_alpha) < 1e-6 or abs(heading_error) < 0.01:
                    geom_curv_str = "0.000000"
                    geom_radius_str = "inf"
                    geom_turn_dir_str = "STRAIGHT"
                else:
                    curv = (2.0 * sin_alpha) / dist_to_near
                    radius = dist_to_near / (2.0 * abs(sin_alpha))
                    geom_curv_str = f"{curv:.6f}"
                    geom_radius_str = f"{radius:.2f}"
                    geom_turn_dir_str = "RIGHT" if heading_error > 0 else "LEFT"

            if self.logger:
                self.logger.print(f"{now},{lat},{lon},{nose_lat},{nose_lon},{heading},{target_heading},{heading_error},{distance_to_goal},{d_perp},{wp_idx},{target_left},{target_right},{actual_left},{actual_right},{current_lidar},{h_acc_val},{self.state},{self.source},{reason_escaped},{target_wp_idx_str},{target_wp_lat_str},{target_wp_lon_str},{near_lat_str},{near_lon_str},{closest_lat_str},{closest_lon_str},{lookahead_dist_str},{geom_curv_str},{geom_radius_str},{geom_turn_dir_str}")

            if self.state in ["STOPPED", "FINISHED"] and actual_left == 0 and actual_right == 0:
                print(f"[PilotMaraton] Robot plynule zastavil ({self.state}). Ukončuji řídicí smyčku.")
                self.running = False
                return False

        return True

    def _control_loop(self):
        print("[PilotMaraton] Control loop started (10 Hz).")
        while self.running:
            start_time = time.time()
            if not self._run_control_step(now_ts=start_time):
                break
            elapsed = time.time() - start_time
            sleep_time = max(0, 0.1 - elapsed)
            time.sleep(sleep_time)


# Alias pro zpětnou kompatibilitu
WaypointsPilotService = MaratonPilotService
