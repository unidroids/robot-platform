import threading
import time
import math
import json
import zmq
import asyncio
from path_tracker import PathTracker
from drive_client import DriveClient
from data_logger import DataLogger

class WaypointsPilotService:
    def __init__(self):
        self.state = "IDLE" # IDLE, RUNNING, PAUSED, STOPPED, FINISHED
        self.status_info = ""
        self.source = ""
        
        self.max_speed = 100
        self.max_pwm = 150
        
        # Limity zrychlení (v jednotkách rychlosti za iteraci, tedy per 0.1s)
        self.max_fwd_accel_step = 20.0
        self.max_brk_accel_step = 40.0
        self.max_ang_accel_step = 30.0  # dříve max_lat_accel_step (omezuje jak rychle se roztočíme)
        
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
        
        self.fusion_data = None
        self.lidar_distance = -1.0
        self.last_lidar_time = 0.0
        
        # OOW state
        self.oow_tcp_ok = False
        self.oow_zmq_ok = True  # Default true dokud nepřijde OFF
        
        self.receiver = None
        
        self.oow_task = None
        
        self.control_thread = None
        self.zmq_thread = None

    def start_service(self, max_speed=100, max_pwm=150):
        if self.state in ["RUNNING"]:
            return "ALREADY RUNNING"
            
        print(f"[PilotService] START: max_speed={max_speed}, max_pwm={max_pwm}")
        self.max_speed = max_speed
        self.max_pwm = max_pwm
        
        # Reset speeds
        self.last_v = 0.0
        self.last_w = 0.0
        self.current_speed = 0.0
        
        # Init logger
        self.logger = DataLogger()
        self.logger.print("time,lat,lon,heading,target_heading,heading_error,distance_to_goal_m,d_perp_m,target_left,target_right,actual_left,actual_right,lidar_dist,state")
        
        self.path_tracker = PathTracker("/opt/projects/robotour/pilot_waypoints/waypoints/_route.json", L_near_m=2.0)
        
        self.state = "RUNNING"
        self.source = "USER"
        self.status_info = "Starting"
        
        if not self.running:
            self.running = True
            
            # ZMQ Receiver
            self.zmq_thread = threading.Thread(target=self._zmq_loop, daemon=True)
            self.zmq_thread.start()
            
            # Control Loop
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
        
        return "OK"

    def stop_service(self):
        print("[PilotService] Zastavuji službu (zpomaluji na 0)...")
        if self.state != "STOPPED":
            self.state = "STOPPED"
            self.source = "USER"
            self.status_info = "Stopped by command"
        return "OK"
        
    def pause_service(self, source="USER", info=""):
        if self.state != "PAUSED" and self.state != "STOPPED":
            print(f"[PilotService] PAUSE od {source}: {info}")
            self.state = "PAUSED"
            self.source = source
            self.status_info = info
        return "OK"
        
    def resume_service(self, source="USER", info=""):
        if self.state == "PAUSED":
            print(f"[PilotService] RESUME od {source}: {info}")
            self.state = "RUNNING"
            self.source = source
            self.status_info = info
        return "OK"
        
    def get_status(self):
        if self.state == "IDLE":
            return "IDLE"
        elif self.state == "RUNNING":
            idx = self.path_tracker.current_wp_index if self.path_tracker else 0
            if self.fusion_data:
                lat = self.fusion_data.get('lat', 0)
                lon = self.fusion_data.get('lon', 0)
                heading = self.fusion_data.get('heading', 0)
                gpsSol = self.fusion_data.get('gpsSol', 'NONE')
                hAcc = self.fusion_data.get('hAcc', 9999)
                headingSol = self.fusion_data.get('headingSol', 'NONE')
                headingAcc = self.fusion_data.get('headingAcc', 9999)
                return f"RUNNING idx:{idx} lat:{lat} lon:{lon} heading:{heading} gpsSol:{gpsSol} hAcc:{hAcc} headingSol:{headingSol} hedingAcc:{headingAcc}"
            return f"RUNNING {idx}"
        elif self.state == "PAUSED":
            if self.source == "GPS" and self.fusion_data:
                hAcc = self.fusion_data.get("hAcc", 9999)
                headingAcc = self.fusion_data.get('headingAcc', 9999)
                return f"PAUSED {self.source} Current hAcc: {hAcc} mm headingAcc: {headingAcc}"
            return f"PAUSED {self.source} {self.status_info}"
        elif self.state == "STOPPED":
            return f"STOPPED {self.source} {self.status_info}"
        elif self.state == "FINISHED":
            return f"FINISHED {self.status_info}"
        return self.state

    def shutdown(self):
        print(f"[PilotService] SHUTDOWN")
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
        print(f"[PilotService] OOW ZMQ Event: {msg}")
        msg_upper = msg.strip().upper()
        if "OFF" in msg_upper or "PAUSE" in msg_upper:
            self.oow_zmq_ok = False
            self.pause_service(source="OOW_ZMQ", info=msg)
        elif "ON" in msg_upper or "RESUME" in msg_upper:
            self.oow_zmq_ok = True
            if self.oow_tcp_ok and self.state == "PAUSED" and self.source == "OOW_ZMQ":
                self.resume_service(source="OOW_ZMQ", info=msg)
        elif "STOP" in msg_upper:
            self.oow_zmq_ok = False
            self.stop_service()

    def set_oow_tcp_ok(self, is_ok):
        if self.oow_tcp_ok != is_ok:
            print(f"[PilotService] OOW TCP stav se změnil na: {'OK' if is_ok else 'FAIL'}")
        self.oow_tcp_ok = is_ok
        if not is_ok and self.state == "RUNNING":
            self.pause_service(source="OOW_TCP", info="Lost OOW connection")
        elif is_ok and self.state == "PAUSED" and self.source == "OOW_TCP":
            if self.oow_zmq_ok:
                self.resume_service(source="OOW_TCP", info="OOW connection restored")

    def _zmq_loop(self):
        context = zmq.Context()
        sub = context.socket(zmq.SUB)
        sub.connect("ipc:///tmp/robot-fusion")
        sub.connect("ipc:///tmp/robot-lidar")
        sub.connect("ipc:///tmp/robot-oow")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        print("[PilotService] ZMQ Subscriber started.")
        
        while self.running:
            try:
                msg = sub.recv_string(flags=zmq.NOBLOCK)
                if msg.startswith("SOLUTION/"):
                    self.update_fusion(json.loads(msg[9:]))
                elif msg.startswith("distance/"):
                    self.update_lidar(json.loads(msg[9:]))
                else:
                    self.update_oow_zmq(msg)
            except zmq.Again:
                time.sleep(0.01)
            except Exception as e:
                print(f"[PilotService] ZMQ chyba: {e}")
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
        w_rads = (target_right - target_left) / self.wheelbase_mm   # rad/s (rozdíl v mm/s / rozchod v mm)
        a_c = abs(v_target_m * w_rads)                              # m/s^2
        
        if a_c > self.max_centrifugal_accel:
            # Zachováme poloměr zatáčení, snížíme rychlost i úhlovou rychlost (obě škálujeme k)
            # a_c = v * w -> po naškálování: a_c_new = (k * v) * (k * w) = k^2 * a_c
            k = math.sqrt(self.max_centrifugal_accel / a_c)
            target_left *= k
            target_right *= k

        # 2. Rozklad na V a W složky (v jednotkách rychlosti motorů, např. mm/s)
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
        print("[PilotService] Control loop started (10 Hz).")
        while self.running:
            start_time = time.time()
            
            target_left = 0
            target_right = 0
            actual_left = 0
            actual_right = 0
            
            # Variables for logging
            lat = 0
            lon = 0
            heading = 0
            target_heading = 0
            heading_error = 0
            distance_to_goal = 0
            d_perp = 0
            
            if self.state in ["RUNNING", "PAUSED", "STOPPED", "FINISHED"]:
                # Defaultní target_v pro případy, kdy nemáme data z GPS atd.
                target_v = self.max_speed if self.state == "RUNNING" else 0.0

                # Sledování trasy a logika GPS
                if not self.fusion_data:
                    pass
                else:
                    hAcc = self.fusion_data.get("hAcc", 9999) # v mm
                    heading_sol = self.fusion_data.get("headingSol", "NONE")
                    heading_acc = self.fusion_data.get("headingAcc", 9999.0)
                    
                    if hAcc > 700 or heading_sol == "NONE" or heading_acc > 6.0:
                        heading_val = self.fusion_data.get("heading", 0.0)
                        info_msg = f"Bad GPS/Hdg. hAcc:{hAcc}mm hdg:{heading_val:.1f} hdgAcc:{heading_acc} hdgSol:{heading_sol}"
                        if self.state == "RUNNING":
                            self.pause_service(source="GPS", info=info_msg)
                        target_left, target_right = 0, 0
                    else:
                        # Pokud byla chyba GPS odstraněna, obnovíme běh (pokud jsme byli pauznutí kvůli ní)
                        if self.state == "PAUSED" and self.source == "GPS" and hAcc < 500 and heading_sol != "NONE" and heading_acc <= 6.0:
                            self.resume_service(source="GPS", info=f"Accuracy improved: hAcc={hAcc} mm, hdgSol={heading_sol}, hdgAcc={heading_acc}")
                            
                        lat = self.fusion_data.get("lat")
                        lon = self.fusion_data.get("lon")
                        heading = self.fusion_data.get("heading")
                        
                        near_state = self.path_tracker.update(lat, lon)
                        
                        if near_state is None:
                            if self.state != "FINISHED":
                                print("[PilotService] Path not found. Přepínám na stav FINISHED (zpomaluji na 0).")
                                self.state = "FINISHED"
                                self.status_info = "Path not found"
                            target_left, target_right = 0, 0
                        else:
                            if near_state.distance_to_goal_m is not None and near_state.distance_to_goal_m < 0 and self.path_tracker.current_wp_index >= len(self.path_tracker.waypoints)-2:
                                if self.state != "FINISHED":
                                    print("[PilotService] Konec trasy dosažen. Přepínám na stav FINISHED (zpomaluji na 0).")
                                    self.state = "FINISHED"
                                    self.status_info = "Goal reached"
                                
                            if near_state.heading_to_near_gnss_deg is not None:
                                target_heading = near_state.heading_to_near_gnss_deg
                            distance_to_goal = near_state.distance_to_goal_m
                            d_perp = near_state.d_perp_m
                            
                            # Korekce target_v na základě relativního azimutu (zpomalení do zatáčky)
                            if self.state == "RUNNING" and near_state.end_rel_azimuth_deg is not None and near_state.distance_to_goal_m is not None:
                                rel_az = near_state.end_rel_azimuth_deg
                                v_waypoint = self.max_speed * math.exp(- (abs(rel_az)/45.0)**2)
                                v_waypoint = max(min(self.max_speed, 200.0), v_waypoint)
                                dist = max(0.0, near_state.distance_to_goal_m)
                                slowdown_dist = self.path_tracker.L_near_m
                                if dist < slowdown_dist:
                                    target_v = v_waypoint + (self.max_speed - v_waypoint) * (dist / slowdown_dist)

                            # Řízení požadované rychlosti (current_speed)
                            if self.current_speed < target_v:
                                self.current_speed = min(target_v, self.current_speed + self.max_fwd_accel_step)
                            elif self.current_speed > target_v:
                                self.current_speed = max(target_v, self.current_speed - self.max_brk_accel_step)
                            
                            # Validace aktuálnosti lidaru (timeout 2s)
                            lidar_active = (time.time() - self.last_lidar_time) < 2.0
                            current_lidar = self.lidar_distance if lidar_active else -1.0
                            
                            if 0 < current_lidar < 70.0:
                                print(f"[PilotService] Lidar antikolize ({current_lidar}cm) - zastavuji.")
                                target_left, target_right = 0, 0
                            else:
                                target_left, target_right, heading_error = self._calculate_steering(heading, target_heading, current_lidar, self.current_speed)

                # Fallback aktualizace rychlosti, pokud nebyla upravena (např. chyba GPS)
                if not self.fusion_data or (self.fusion_data and (self.fusion_data.get("hAcc", 9999) > 700 or self.fusion_data.get("headingSol", "NONE") == "NONE")):
                    if self.current_speed < target_v:
                        self.current_speed = min(target_v, self.current_speed + self.max_fwd_accel_step)
                    elif self.current_speed > target_v:
                        self.current_speed = max(target_v, self.current_speed - self.max_brk_accel_step)

                # Spočítáme fyzické limity zrychlení a pošleme do motorů
                actual_left, actual_right = self._apply_acceleration_limits(target_left, target_right)
                self.drive.send_drive(self.max_pwm, actual_left, actual_right)
                
            if self.state in ["RUNNING", "PAUSED", "STOPPED", "FINISHED"]:
                if self.logger:
                    self.logger.print(f"{time.time()},{lat},{lon},{heading},{target_heading},{heading_error},{distance_to_goal},{d_perp},{target_left},{target_right},{actual_left},{actual_right},{self.lidar_distance},{self.state}")
                
                # Pokud jsme ve stavu STOPPED nebo FINISHED a skutečně jsme zastavili, ukončíme smyčku (motory drží pozici)
                if self.state in ["STOPPED", "FINISHED"] and actual_left == 0 and actual_right == 0:
                    print(f"[PilotService] Robot plynule zastavil ({self.state}). Ukončuji řídicí smyčku.")
                    self.running = False
                    break
            
            # 10 Hz
            elapsed = time.time() - start_time
            sleep_time = max(0, 0.1 - elapsed)
            time.sleep(sleep_time)
