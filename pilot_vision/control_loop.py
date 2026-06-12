import threading
import time
from drive_client import DriveClient

class ControlLoop:
    """
    Řídící smyčka s frekvencí 10 Hz.
    Vyhodnocuje stav senzorů a posílá řídící povely.
    """
    def __init__(self, state):
        self.state = state
        self.is_running = False
        self.thread = None
        self.shutdown_event = threading.Event()
        self.active_token = None
        self.drive = DriveClient()
        self.is_paused = False
        
        self.loop_counter = 0

    def start(self, token: str):
        self.active_token = token
        if not self.is_running:
            self.shutdown_event.clear()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            self.is_running = True

    def stop(self):
        if self.is_running:
            self.shutdown_event.set()
            if self.thread:
                self.thread.join(timeout=3.0)
            self.is_running = False
            self.active_token = None
            try:
                self.drive.send_break()
            except:
                pass

    def update_token(self, token: str):
        self.active_token = token

    def pause(self):
        self.is_paused = True
        try:
            self.drive.send_break()
        except:
            pass

    def resume(self):
        self.is_paused = False

    def _calculate_steering(self, pose_lines, lidar_dist=5000.0):
        if not pose_lines:
            return None

        best_line = max(pose_lines, key=lambda l: l.get("line_conf", 0.0))
        pts = best_line.get("points", [])
        if not pts:
            return None

        pts.sort(key=lambda pt: pt.get('x', 0))
        # Vezmeme bod kousek dále před robotem pro lepší pure pursuit
        mid_idx = min(len(pts) - 1, len(pts) * 2 // 3)
        target_pt = pts[mid_idx]
        
        target_x = target_pt.get('x', 0.0)
        target_y = target_pt.get('y', 0.0)

        import math
        L_NEAR = math.hypot(target_x, target_y)
        if L_NEAR < 0.01:
            return 0, 0, target_x, target_y

        heading_error_rad = math.atan2(target_y, target_x)
        heading_error_deg = math.degrees(heading_error_rad)
        err_abs = abs(heading_error_deg)

        max_fwd_speed = 100.0  # cm/s
        max_spin_speed = 40.0  # cm/s
        a_y_max = 1.0          # m/s^2
        B = 0.5                # Rozvor kol v metrech

        # Zpomalení dle vzdálenosti bodu (na kameře)
        dist_factor = min(1.0, max(0.3, L_NEAR / 2.0))
        
        # Zpomalení dle LiDARu (mezi 50 a 150 cm)
        lidar_factor = 1.0
        if 50.0 <= lidar_dist < 150.0:
            lidar_factor = max(0.2, (lidar_dist - 50.0) / 100.0)

        if err_abs < 15.0:
            v_center = max_fwd_speed
        elif err_abs < 60.0:
            v_center = max_fwd_speed * (1.0 - (err_abs - 15.0) / 45.0)
        else:
            v_center = 0.0

        v_center *= (dist_factor * lidar_factor)

        # Pure Pursuit
        kappa = 2.0 * target_y / (L_NEAR**2)

        if abs(kappa) > 0.001:
            v_ay_limit = math.sqrt(a_y_max / abs(kappa)) * 100.0
            v_center = min(v_center, v_ay_limit)

        v_turn_pp = v_center * kappa * B / 2.0
        
        spin_mag = max_spin_speed * min(1.0, err_abs / 60.0)
        v_turn_spin = math.copysign(spin_mag, target_y)
        
        blend_factor = min(1.0, err_abs / 60.0)
        v_turn = (1.0 - blend_factor) * v_turn_pp + blend_factor * v_turn_spin
        
        left_speed = v_center - v_turn
        right_speed = v_center + v_turn

        abs_limit = 100.0
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > abs_limit:
            left_speed = (left_speed / max_wheel) * abs_limit
            right_speed = (right_speed / max_wheel) * abs_limit

        return int(round(left_speed)), int(round(right_speed)), target_x, target_y

    def _run(self):
        print(f"🏎️ [ControlLoop] Smyčka spuštěna (10 Hz) s tokenem: '{self.active_token}'")
        self.drive.set_token(self.active_token)
        
        # Pro setrvačnost při ztrátě
        last_speed = 0.0
        last_steer = 0.0
        lost_frames = 0
        
        try:
            while not self.shutdown_event.is_set():
                start_time = time.time()
                
                self.drive.set_token(self.active_token)
                self.loop_counter += 1
                
                if self.is_paused:
                    self._sleep_until_next_frame(start_time)
                    continue
                
                # Snímek aktuálního stavu
                snap = self.state.get_snapshot()
                dist = snap["lidar_distance"]
                vision_age = start_time - snap["last_vision_time"]
                
                # 1) Bezpečnost: LiDAR překážka pod 50 cm
                if 0 <= dist < 50.0:
                    print(f"⚠️ [ControlLoop] PŘEKÁŽKA z LiDARu! Vzdálenost: {dist:.1f} cm. BREAK!")
                    self.drive.send_break()
                    self._sleep_until_next_frame(start_time)
                    continue

                # 2) Kontrola čerstvosti Vision dat
                if vision_age > 1.0:
                    print(f"⚠️ [ControlLoop] Ztráta kamery (data stará {vision_age:.1f}s). ZASTAVUJI!")
                    self.drive.send_drive(100, 0, 0)
                    self._sleep_until_next_frame(start_time)
                    continue

                # 3) Normální vyhodnocení čáry
                control = self._calculate_steering(snap["vision_pose_lines"], dist)
                
                # Výchozí hodnoty pro log
                cur_target_x = 0.0
                cur_target_y = 0.0
                
                if control is not None:
                    left, right, cur_target_x, cur_target_y = control
                    last_speed = left # Použijeme left pro setrvačnost (zjednodušení)
                    last_steer = right
                    lost_frames = 0
                else:
                    lost_frames += 1
                    if lost_frames < 5:
                        print(f"⚠️ [ControlLoop] Ztráta čáry (setrvačnost {lost_frames}/5).")
                        left = last_speed
                        right = last_steer
                    else:
                        print(f"⚠️ [ControlLoop] Dlouhodobá ztráta čáry. ZASTAVUJI!")
                        left = 0
                        right = 0
                
                # Odeslání
                try:
                    self.drive.send_drive(100, left, right)
                    print(f"🚀 [ControlLoop] OK -> Lidar:{dist:.1f}cm Odom:{snap['odom_speed_left']}/{snap['odom_speed_right']} X:{cur_target_x:.2f}m Y:{cur_target_y:.2f}m Motor:L={left} R={right}")
                except Exception as e:
                    print(f"⚠️ [ControlLoop] Chyba komunikace s Drive: {e}")

                # Pozitivní výpis (každý 10. průchod -> cca každou 1 vteřinu)
                if self.loop_counter % 10 == 0:
                    print(f"🚀 [ControlLoop] OK -> Lidar:{dist:.1f}cm Odom:{snap['odom_speed_left']}/{snap['odom_speed_right']} X:{cur_target_x:.2f}m Y:{cur_target_y:.2f}m Motor:L={left} R={right}")

                self._sleep_until_next_frame(start_time)

        except Exception as e:
            print(f"❌ [ControlLoop] Neočekávaná chyba: {e}")
        finally:
            try:
                self.drive.send_break()
            except:
                pass
            print("🛑 [ControlLoop] Vlákno ukončeno.")

    def _sleep_until_next_frame(self, start_time):
        elapsed = time.time() - start_time
        sleep_time = 0.1 - elapsed  # 10 Hz = 100 ms perioda
        if sleep_time > 0:
            time.sleep(sleep_time)
