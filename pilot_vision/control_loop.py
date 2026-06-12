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

    def _calculate_steering(self, pose_lines):
        if not pose_lines:
            return None

        best_line = max(pose_lines, key=lambda l: l.get("line_conf", 0.0))
        pts = best_line.get("points", [])
        if not pts:
            return None

        pts.sort(key=lambda pt: pt.get('x', 0))
        mid_idx = len(pts) // 2
        target_pt = pts[mid_idx]
        
        target_x = target_pt.get('x', 0.0)
        target_y = target_pt.get('y', 0.0)

        min_speed = 30
        max_speed = 120
        base_speed = int(min_speed + (target_x / 3.0) * (max_speed - min_speed))
        base_speed = max(min_speed, min(max_speed, base_speed))
        
        if abs(target_y) > 0.5:
            base_speed = int(base_speed * 0.7)

        P_gain = 40.0
        steer = target_y * P_gain
        return base_speed, steer

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
                control = self._calculate_steering(snap["vision_pose_lines"])
                if control is not None:
                    base_speed, steer = control
                    last_speed = base_speed
                    last_steer = steer
                    lost_frames = 0
                else:
                    lost_frames += 1
                    if lost_frames < 5:
                        print(f"⚠️ [ControlLoop] Ztráta čáry (setrvačnost {lost_frames}/5).")
                        base_speed = last_speed
                        steer = last_steer
                    else:
                        print(f"⚠️ [ControlLoop] Dlouhodobá ztráta čáry. ZASTAVUJI!")
                        base_speed = 0
                        steer = 0

                left = int(max(-50, min(200, base_speed + steer)))
                right = int(max(-50, min(200, base_speed - steer)))
                
                # Odeslání
                try:
                    #self.drive.send_drive(100, left, right)
                    print(f"🚀 [ControlLoop] OK -> Lidar: {dist:.1f}cm, Odom: {snap['odom_speed_left']}/{snap['odom_speed_right']}, Motor: L={left} R={right}")
                except Exception as e:
                    print(f"⚠️ [ControlLoop] Chyba komunikace s Drive: {e}")

                # Pozitivní výpis (každý 10. průchod -> cca každou 1 vteřinu)
                if self.loop_counter % 10 == 0:
                    print(f"🚀 [ControlLoop] OK -> Lidar: {dist:.1f}cm, Odom: {snap['odom_speed_left']}/{snap['odom_speed_right']}, Motor: L={left} R={right}")

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
