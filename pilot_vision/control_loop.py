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
        self.last_v_center = 0.0

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
            self.last_v_center = 0.0
            try:
                self.drive.send_break()
            except:
                pass

    def update_token(self, token: str):
        self.active_token = token

    def pause(self):
        self.is_paused = True
        self.last_v_center = 0.0
        try:
            self.drive.send_break()
        except:
            pass

    def resume(self):
        self.is_paused = False

    def _calculate_steering(self, target_pt, lidar_dist=5000.0):
        if not target_pt:
            return None

        target_x, target_y = target_pt

        import math
        L_NEAR = math.hypot(target_x, target_y)
        if L_NEAR < 0.01:
            return 0, 0, target_x, target_y

        heading_error_rad = math.atan2(target_y, target_x)
        heading_error_deg = math.degrees(heading_error_rad)
        err_abs = abs(heading_error_deg)

        max_fwd_speed = 180.0  # cm/s
        max_spin_speed = 40.0  # cm/s
        a_y_max = 0.2          # m/s^2 (dostředivé zrychlení)
        a_x_max = 0.2          # m/s^2 (dopředné zrychlení)
        B = 0.58               # Rozvor kol v metrech

        # Zpomalení dle vzdálenosti bodu (na kameře)
        dist_factor = min(1.0, max(0.3, L_NEAR / 2.0))
        
        # Zpomalení dle LiDARu (mezi 50 a 150 cm)
        lidar_factor = 1.0
        if 50.0 <= lidar_dist < 150.0:
            lidar_factor = max(0.2, (lidar_dist - 50.0) / 100.0)

        if err_abs < 15.0:
            v_center_target = max_fwd_speed
        elif err_abs < 60.0:
            v_center_target = max_fwd_speed * (1.0 - (err_abs - 15.0) / 45.0)
        else:
            v_center_target = 0.0

        v_center_target *= (dist_factor * lidar_factor)

        # Pure Pursuit
        kappa = 2.0 * target_y / (L_NEAR**2)

        if abs(kappa) > 0.001:
            v_ay_limit = math.sqrt(a_y_max / abs(kappa)) * 100.0
            v_center_target = min(v_center_target, v_ay_limit)

        # Omezení dopředného zrychlení (rozjezdu)
        dt = 0.1  # 10 Hz smyčka
        max_dv_accel = a_x_max * 100.0 * dt  # max změna rychlosti v cm/s za jeden cyklus
        
        if v_center_target > self.last_v_center:
            v_center = min(v_center_target, self.last_v_center + max_dv_accel)
        else:
            # Brzdění necháme bez omezení pro bezpečnost
            v_center = v_center_target
            
        self.last_v_center = v_center

        v_turn_pp = v_center * kappa * B / 2.0
        
        spin_mag = max_spin_speed * min(1.0, err_abs / 60.0)
        v_turn_spin = math.copysign(spin_mag, target_y)
        
        blend_factor = min(1.0, err_abs / 60.0)
        v_turn = (1.0 - blend_factor) * v_turn_pp + blend_factor * v_turn_spin
        
        left_speed = v_center - v_turn
        right_speed = v_center + v_turn

        abs_limit = 200.0
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > abs_limit:
            left_speed = (left_speed / max_wheel) * abs_limit
            right_speed = (right_speed / max_wheel) * abs_limit

        return int(round(left_speed)), int(round(right_speed)), target_x, target_y

    def _run(self):
        print(f"🏎️ [ControlLoop] Smyčka spuštěna (10 Hz) s tokenem: '{self.active_token}'")
        self.drive.set_token(self.active_token)
        
        # Odstranena promenna lost_frames, protoze setrvacnost resi PursuitPointTracker
        
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
                    self.last_v_center = 0.0
                    self._sleep_until_next_frame(start_time)
                    continue

                # 2) Kontrola čerstvosti Vision dat
                if vision_age > 1.0:
                    print(f"⚠️ [ControlLoop] Ztráta kamery (data stará {vision_age:.1f}s). ZASTAVUJI!")
                    self.drive.send_drive(100, 0, 0)
                    self.last_v_center = 0.0
                    self._sleep_until_next_frame(start_time)
                    continue

                # 3) Normální vyhodnocení čáry
                if not snap.get("pursuit_ready", False):
                    pursuit_state = snap.get("pursuit_state", "UNKNOWN")
                    print(f"⚠️ [ControlLoop] PursuitPoint (stav {pursuit_state}) není ready. ZASTAVUJI!")
                    self.drive.send_drive(100, 0, 0)
                    self.last_v_center = 0.0
                    self._sleep_until_next_frame(start_time)
                    continue

                control = self._calculate_steering(snap["pursuit_target"], dist)
                
                if control is not None:
                    left, right, cur_target_x, cur_target_y = control
                else:
                    left, right, cur_target_x, cur_target_y = 0, 0, 0.0, 0.0
                
                # Odeslání
                try:
                    self.drive.send_drive(100, left, right)
                    #print(f"🚀 [ControlLoop] OK -> Lidar:{dist:.1f}cm Odom:{snap['odom_speed_left']}/{snap['odom_speed_right']} X:{cur_target_x:.2f}m Y:{cur_target_y:.2f}m Motor:L={left} R={right}")
                except Exception as e:
                    print(f"⚠️ [ControlLoop] Chyba komunikace s Drive: {e}")

                # Pozitivní výpis (každý 10. průchod -> cca každou 1 vteřinu)
                if self.loop_counter % 2 == 0:
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
