import os
import threading
import time
import math
import json
from datetime import datetime
from pathlib import Path
import zmq
import asyncio

try:
    from .path_tracker import PathTracker
    from .drive_client import DriveClient
    from .data_logger import DataLogger
    from .geo_utils import rrp_to_nose
except (ImportError, ValueError):
    from path_tracker import PathTracker
    from drive_client import DriveClient
    from data_logger import DataLogger
    from geo_utils import rrp_to_nose


FRONT_OFFSET_M = 0.40  # Předsazení čumáku robota před středem otáčení (RRP) v metrech


class RobotourPilotService:
    """
    Služba PILOT-ROBOTOUR:
    Autonomní řízení robota podle waypointů s podporou antikolize z LiDARu,
    bezpečnostního dohledu OOW a fúzovaných pozičních dat.
    """

    def __init__(self):
        self.state = "IDLE"  # IDLE, RUNNING, PAUSED, STOPPED, FINISHED
        self.status_info = ""
        self.source = ""
        
        self.max_speed = 100
        self.max_pwm = 150
        
        # Limity zrychlení (v jednotkách rychlosti za iteraci, tedy per 0.1s)
        self.max_fwd_accel_step = 20.0
        self.max_brk_accel_step = 40.0
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
        
        # OOW state
        self.oow_tcp_ok = False
        self.oow_zmq_ok = True  # Default true dokud nepřijde OFF
        
        self.last_distance_to_goal = 0.0
        self.receiver = None
        self.oow_task = None

    def start_service(self, max_speed=100, max_pwm=150, route_input=None) -> tuple[bool, str]:
        if self.state in ["RUNNING"]:
            return False, "ERR: ALREADY RUNNING"
            
        if route_input is None:
            return False, "ERR: START vyžaduje JSON payload s trasou"

        # Inicializace a validace trasy přes PathTracker
        try:
            tracker = PathTracker(route_input, L_near_m=2.0)
            self.path_tracker = tracker
        except Exception as e:
            print(f"[PilotRobotour] Chyba inicializace trasy: {e}")
            return False, f"ERR: {e}"

        print(f"[PilotRobotour] START: max_speed={max_speed}, max_pwm={max_pwm}, waypoints={len(self.path_tracker.waypoints)}")
        self.max_speed = max_speed
        self.max_pwm = max_pwm
        
        # Reset speeds
        self.last_v = 0.0
        self.last_w = 0.0
        self.current_speed = 0.0
        self.last_distance_to_goal = 0.0
        
        # Init logger
        self.logger = DataLogger(base_dir="/data/robot/pilot_robotour")
        self.logger.print("time,lat,lon,nose_lat,nose_lon,heading,target_heading,heading_error,distance_to_goal_m,d_perp_m,wp_index,target_left,target_right,actual_left,actual_right,obstacle_distance_cm,h_acc_mm,state,source,reason")
        
        # Uložení do /data/robot/pilot_robotour/<yyyy-mm-dd>/<HH-MM-SS>/route.json
        now = datetime.now()
        base_dir = Path("/data/robot/pilot_robotour") / now.strftime("%Y-%m-%d") / now.strftime("%H-%M-%S")
        data_to_dump = route_input if isinstance(route_input, (dict, list)) else json.loads(route_input)
        try:
            base_dir.mkdir(parents=True, exist_ok=True)
            route_file = base_dir / "route.json"
            with open(route_file, "w", encoding="utf-8") as f:
                json.dump(data_to_dump, f, indent=2, ensure_ascii=False)
            print(f"[PilotRobotour] Uložena kopie trasy do: {route_file}")
        except (PermissionError, OSError):
            local_dir = Path(os.path.dirname(os.path.abspath(__file__))) / "data" / "robot" / "pilot_robotour" / now.strftime("%Y-%m-%d") / now.strftime("%H-%M-%S")
            try:
                local_dir.mkdir(parents=True, exist_ok=True)
                route_file = local_dir / "route.json"
                with open(route_file, "w", encoding="utf-8") as f:
                    json.dump(data_to_dump, f, indent=2, ensure_ascii=False)
                print(f"[PilotRobotour] Uložena kopie trasy (fallback) do: {route_file}")
            except Exception as e:
                print(f"[PilotRobotour] Chyba při ukládání route.json: {e}")

        self.state = "RUNNING"
        self.source = "USER"
        self.status_info = "Starting"
        
        # WORKAROUND: Chyba ve firmware hoverboardu
        # Firmware vyžaduje po příkazu START probuzení regulace motorů nenulovou rychlostí
        # (speed=1) s nízkým PWM (pwm=1) a následné vynulování rychlosti (speed=0).
        try:
            self.drive.send_start()
            self.drive.send_drive(1, 1, 1)
            time.sleep(0.05)
            self.drive.send_drive(1, 0, 0)
        except Exception as e:
            print(f"[PilotRobotour] Varování: Inicializace DRIVE (workaround firmware) selhala: {e}")
        
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
        print("[PilotRobotour] Zastavuji službu (zpomaluji na 0)...")
        if self.state != "STOPPED":
            self.state = "STOPPED"
            self.source = "USER"
            self.status_info = "Zastaveno příkazem (STOP)"
        return "OK"

    def pause_service(self, source="USER", info=""):
        if self.state != "PAUSED" and self.state != "STOPPED":
            if not info:
                info = "Pozastaveno uživatelem (PAUSE)"
            print(f"[PilotRobotour] PAUSE od {source}: {info}")
            self.state = "PAUSED"
            self.source = source
            self.status_info = info
        return "OK"
        
    def resume_service(self, source="USER", info=""):
        if self.state == "PAUSED":
            if not info:
                info = "Obnoveno uživatelem (RESUME)"
            print(f"[PilotRobotour] RESUME od {source}: {info}")
            self.state = "RUNNING"
            self.source = source
            self.status_info = info
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

        # LiDAR obstacle distance
        lidar_active = (time.time() - self.last_lidar_time) < 2.0 and self.lidar_distance >= 0.0
        obstacle_distance_cm = round(float(self.lidar_distance), 1) if lidar_active else -1.0

        status_dict = {
            "state": self.state,
            "source": self.source,
            "info": self.status_info,
            "wp_index": wp_index,
            "wp_total": wp_total,
            "distance_to_goal_m": dist,
            "obstacle_distance_cm": obstacle_distance_cm,
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
        print(f"[PilotRobotour] SHUTDOWN")
        self.running = False
        self.drive.disconnect()
        if self.logger:
            self.logger.close()
            self.logger = None

    def update_fusion(self, data):
        self.fusion_data = data
        
    def update_lidar(self, data):
        self.lidar_distance = data.get("distance", -1.0)
        self.last_lidar_time = time.time()
        
    def update_oow_zmq(self, msg):
        print(f"[PilotRobotour] OOW ZMQ Event: {msg}")
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
            print(f"[PilotRobotour] OOW TCP stav se změnil na: {'OK' if is_ok else 'FAIL'}")
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
            sub.connect("ipc:///tmp/robot-oow")
            sub.setsockopt_string(zmq.SUBSCRIBE, "")
            connected = True
            print("[PilotRobotour] ZMQ Subscriber started.")
        except Exception as e:
            print(f"[PilotRobotour] ZMQ connect warning: {e}")
        
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
                    elif topic in ["STATUS", "CMD"]:
                        self.update_oow_zmq(payload)
                    else:
                        print(f"[PilotRobotour] Neznámý topic: {topic}")
                else:
                    first_frame = parts[0].decode('utf-8', errors='ignore') if len(parts) > 0 else "EMPTY"
                    print(f"[PilotRobotour] Neplatný formát zprávy (očekáváno len=2), přijato parts: {len(parts)}, první frame: '{first_frame}'")
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                print(f"[PilotRobotour] ZMQ chyba: {e}")
                time.sleep(0.1)
                
    def _calculate_steering(self, heading, target_heading, lidar_dist, current_speed):
        heading_error = target_heading - heading
        heading_error = (heading_error + 180) % 360 - 180
        
        kappa_v = current_speed * 1.5
        v_center = current_speed * math.exp(- (abs(heading_error)/45.0)**2)
        
        # Zpomalení dle LiDARu (mezi 50 a 150 cm)
        if 50.0 <= lidar_dist < 150.0:
            lidar_factor = max(0.2, (lidar_dist - 50.0) / 100.0)
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
        v_diff = v_target - self.last_v
        if v_diff > self.max_fwd_accel_step:
            v_out = self.last_v + self.max_fwd_accel_step
        elif v_diff < -self.max_brk_accel_step:
            v_out = self.last_v - self.max_brk_accel_step
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

    def _control_loop(self):
        print("[PilotRobotour] Control loop started (10 Hz).")
        while self.running:
            start_time = time.time()
            
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
            
            if self.fusion_data:
                lat = float(self.fusion_data.get("lat", 0.0))
                lon = float(self.fusion_data.get("lon", 0.0))
                heading = float(self.fusion_data.get("heading", 0.0))
                nose_lat, nose_lon = rrp_to_nose(lat, lon, heading, FRONT_OFFSET_M)

            lidar_active = (time.time() - self.last_lidar_time) < 2.0 and self.lidar_distance >= 0.0
            current_lidar = round(float(self.lidar_distance), 1) if lidar_active else -1.0

            if self.state in ["RUNNING", "PAUSED", "STOPPED", "FINISHED"]:
                target_v = self.max_speed if self.state == "RUNNING" else 0.0

                # 1. Kontrola LiDARu - bez platných dat robot nesmí jet
                if not lidar_active:
                    target_v = 0.0
                    target_left, target_right = 0, 0
                    if self.state == "RUNNING":
                        self.source = "LIDAR"
                        if self.last_lidar_time == 0.0:
                            self.status_info = "Čekání na data z LiDARu"
                        else:
                            self.status_info = "Výpadek dat z LiDARu (timeout > 2s)"
                # 2. Kontrola fúzních dat (GPS)
                elif not self.fusion_data:
                    target_v = 0.0
                    target_left, target_right = 0, 0
                    if self.state == "RUNNING":
                        self.source = "GPS"
                        self.status_info = "Čekání na data z fúze (GPS)"
                else:
                    hAcc = int(round(float(self.fusion_data.get("hAcc", 9999)))) # v mm
                    heading_sol = self.fusion_data.get("headingSol", "NONE")
                    heading_acc = float(self.fusion_data.get("headingAcc", 9999.0))
                    
                    if hAcc > 700 or heading_sol == "NONE" or heading_acc > 6.0:
                        info_msg = f"Nízká přesnost GPS (hAcc: {hAcc} mm, sol: {heading_sol})"
                        if self.state == "RUNNING":
                            self.pause_service(source="GPS", info=info_msg)
                        target_left, target_right = 0, 0
                    else:
                        # Pokud byla chyba GPS odstraněna, obnovíme běh
                        if self.state == "PAUSED" and self.source == "GPS" and hAcc < 500 and heading_sol != "NONE" and heading_acc <= 6.0:
                            self.resume_service(source="GPS", info=f"Přesnost GPS obnovena (hAcc: {hAcc} mm, sol: {heading_sol})")
                            
                        near_state = self.path_tracker.update(nose_lat, nose_lon)
                        
                        if near_state is None:
                            if self.state != "FINISHED":
                                print("[PilotRobotour] Path not found. Přepínám na stav FINISHED (zpomaluji na 0).")
                                self.state = "FINISHED"
                                self.source = "PILOT"
                                self.status_info = "Trasa nenalezena"
                            target_left, target_right = 0, 0
                        else:
                            if near_state.distance_to_goal_m is not None and near_state.distance_to_goal_m < 0 and self.path_tracker.current_wp_index >= len(self.path_tracker.waypoints)-2:
                                if self.state != "FINISHED":
                                    print("[PilotRobotour] Konec trasy dosažen. Přepínám na stav FINISHED (zpomaluji na 0).")
                                    self.state = "FINISHED"
                                    self.source = "PILOT"
                                    self.status_info = "Cíl dosažen"
                                
                            if near_state.heading_to_near_gnss_deg is not None:
                                target_heading = near_state.heading_to_near_gnss_deg
                            distance_to_goal = near_state.distance_to_goal_m
                            self.last_distance_to_goal = distance_to_goal if distance_to_goal is not None else 0.0
                            d_perp = near_state.d_perp_m
                            
                            # Korekce target_v na základě relativního azimutu (zpomalení do zatáčky)
                            if self.state == "RUNNING" and near_state.end_rel_azimuth_deg is not None and near_state.distance_to_goal_m is not None:
                                rel_az = near_state.end_rel_azimuth_deg
                                v_waypoint = self.max_speed * math.exp(- (abs(rel_az)/45.0)**2)
                                v_waypoint = max(min(self.max_speed, 100.0), v_waypoint)
                                dist = max(0.0, near_state.distance_to_goal_m)
                                slowdown_dist = self.path_tracker.L_near_m
                                if dist < slowdown_dist:
                                    target_v = v_waypoint + (self.max_speed - v_waypoint) * (dist / slowdown_dist)

                            # Kontrola antikolize LiDARu (< 70 cm)
                            if 0 < current_lidar < 70.0:
                                target_v = 0.0
                                target_left, target_right = 0, 0
                                if self.state == "RUNNING":
                                    self.source = "LIDAR"
                                    self.status_info = f"Překážka před robotem ({current_lidar:.0f} cm)"
                            else:
                                if self.state == "RUNNING" and self.source in ["LIDAR", "GPS", "USER"]:
                                    self.source = "NAV"
                                    self.status_info = "Jízda podle trasy"
                                target_left, target_right, heading_error = self._calculate_steering(heading, target_heading, current_lidar, self.current_speed)

                # Řízení požadované rychlosti (current_speed)
                if self.current_speed < target_v:
                    self.current_speed = min(target_v, self.current_speed + self.max_fwd_accel_step)
                elif self.current_speed > target_v:
                    self.current_speed = max(target_v, self.current_speed - self.max_brk_accel_step)

                # Fyzické limity zrychlení a odeslání do motorů
                actual_left, actual_right = self._apply_acceleration_limits(target_left, target_right)
                self.drive.send_drive(self.max_pwm, actual_left, actual_right)
                
            if self.state in ["RUNNING", "PAUSED", "STOPPED", "FINISHED"]:
                wp_idx = self.path_tracker.current_wp_index if self.path_tracker else 0
                h_acc_val = int(round(float(self.fusion_data.get("hAcc", 9999)))) if self.fusion_data else 9999
                reason_escaped = f'"{self.status_info}"'
                if self.logger:
                    self.logger.print(f"{time.time()},{lat},{lon},{nose_lat},{nose_lon},{heading},{target_heading},{heading_error},{distance_to_goal},{d_perp},{wp_idx},{target_left},{target_right},{actual_left},{actual_right},{current_lidar},{h_acc_val},{self.state},{self.source},{reason_escaped}")
                
                if self.state in ["STOPPED", "FINISHED"] and actual_left == 0 and actual_right == 0:
                    print(f"[PilotRobotour] Robot plynule zastavil ({self.state}). Ukončuji řídicí smyčku.")
                    self.running = False
                    break
            
            elapsed = time.time() - start_time
            sleep_time = max(0, 0.1 - elapsed)
            time.sleep(sleep_time)


# Alias pro zpětnou kompatibilitu
WaypointsPilotService = RobotourPilotService
