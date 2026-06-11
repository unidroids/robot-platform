import threading
import zmq
import json
import time
from drive_client import DriveClient

class PilotVisionService:
    def __init__(self):
        self.is_running = False
        self.thread = None
        self.shutdown_event = threading.Event()
        self.active_token = None
        self.drive = DriveClient()
        
        # Interní paměť pro "fúzi s odometrií" (zatím primitivní setrvačnost)
        self.last_steer = 0.0
        self.last_speed = 0.0
        self.lost_frames = 0

    def start(self, token: str) -> bool:
        was_running = self.is_running
        self.active_token = token
        
        if not self.is_running:
            self.shutdown_event.clear()
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()
            self.is_running = True
            return True
        return False  # Vrací False pokud už běžel a jenom se updatnul token

    def stop(self) -> bool:
        if not self.is_running:
            return False
            
        self.shutdown_event.set()
        if self.thread:
            self.thread.join(timeout=3.0)
            
        self.is_running = False
        self.active_token = None
        # Zastavení robota bez ověření (bez tokenu) přes BREAK nebo s tokenem přes OFF
        # Pro jistotu použijeme drive client, abychom to zabrzdili
        try:
            self.drive.send_break()
        except:
            pass
        return True

    def get_status(self) -> str:
        if self.is_running:
            return f"RUNNING TOKEN={self.active_token}"
        return "STOPPED"

    def _calculate_steering(self, pose_lines):
        """Vypočítá řídící povely z přijatých bodů."""
        if not pose_lines:
            return None

        # Bereme první detekovanou čáru (v budoucnu fúze nebo výběr té nejlepší podle conf)
        best_line_obj = max(pose_lines, key=lambda l: l.get("line_conf", 0.0)) if pose_lines else None
        if not best_line_obj:
            return None
            
        line = best_line_obj.get("points", [])
        if not line:
            return None

        # Seřadíme body podle 'x' (vzdálenost před robotem, nejbližší první)
        line.sort(key=lambda pt: pt.get('x', 0))
        
        # Zvolíme prostřední bod dle návrhu uživatele
        mid_idx = len(line) // 2
        target_pt = line[mid_idx]
        
        target_x = target_pt.get('x', 0.0)
        target_y = target_pt.get('y', 0.0)

        # 1. Rychlost se odvíjí od toho, jak daleko je vybraný bod.
        # Čím dál vidíme, tím rychleji můžeme jet.
        # Např. x=3.0m => base_speed = 100, x=1.0m => base_speed = 40
        min_speed = 30
        max_speed = 120
        base_speed = int(min_speed + (target_x / 3.0) * (max_speed - min_speed))
        base_speed = max(min_speed, min(max_speed, base_speed))
        
        # Pokud je bod moc blízko a navíc hodně do boku (ostrá zatáčka), zpomalíme
        if abs(target_y) > 0.5:
            base_speed = int(base_speed * 0.7)

        # 2. Úhel zatáčení (boční odchylka bodu 'y')
        # P-regulátor. Pokud je bod vlevo (y > 0 nebo y < 0, musíme sladit znaménka).
        # Předpoklad: y > 0 znamená doleva? Záleží na souřadnicích kamery.
        # Řekněme, že P konstanta je 40
        P_gain = 40.0
        steer = target_y * P_gain

        return base_speed, steer

    def _run_loop(self):
        print(f"🚀 [Pilot-Vision] Vlákno spuštěno s tokenem: {self.active_token}")
        
        # Připojení k Drive (Získáme si vlastní instanci clienta pro toto vlákno, 
        # nebo updatneme token globálního clienta)
        self.drive.set_token(self.active_token)
        
        context = zmq.Context.instance()
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.CONFLATE, 1)
        sub.connect("ipc:///tmp/robot-vision")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        
        # Reset paměti
        self.last_steer = 0.0
        self.last_speed = 0.0
        self.lost_frames = 0
        
        try:
            while not self.shutdown_event.is_set():
                # Timeout
                socks = dict(poller.poll(500))
                
                # Zajištění platnosti tokenu (může se změnit za chodu)
                self.drive.set_token(self.active_token)
                
                if sub not in socks:
                    # Timeout z kamery (nedošly snímky). Robot by měl asi zastavit.
                    self.lost_frames += 1
                    if self.lost_frames > 2:
                        self.drive.send_drive(100, 0, 0)
                    continue
                    
                msg = sub.recv_string()
                # msg format: "vision/{"time": ..., "pose": [[...], ...]}"
                try:
                    topic, json_str = msg.split("/", 1)
                    data = json.loads(json_str)
                except Exception:
                    continue

                pose_lines = data.get("pose", [])
                
                control = self._calculate_steering(pose_lines)
                
                if control is not None:
                    base_speed, steer = control
                    self.last_speed = base_speed
                    self.last_steer = steer
                    self.lost_frames = 0
                else:
                    self.lost_frames += 1
                    # Fúze s odometrií by tady predikovala bod.
                    # Prozatím jednoduchá setrvačnost (pokud ztratíme čáru na 3 snímky, pokračujeme dál, pak zastavíme)
                    if self.lost_frames < 5:
                        base_speed = self.last_speed
                        steer = self.last_steer
                    else:
                        base_speed = 0
                        steer = 0

                left_speed = int(base_speed + steer)
                right_speed = int(base_speed - steer)

                # Oříznutí
                left_speed = max(-50, min(200, left_speed))
                right_speed = max(-50, min(200, right_speed))
                
                # Odeslání do Drive
                # Drive_client.py se automaticky stará o případný reconnect a odesílá `TOKEN` 
                try:
                    #self.drive.send_drive(100, left_speed, right_speed)
                    print(f"[Pilot-Vision] DRIVE: {left_speed} {right_speed}")
                except Exception as e:
                    print(f"⚠️ [Pilot-Vision] Chyba komunikace s Drive: {e}")

        except Exception as e:
            print(f"❌ [Pilot-Vision] Neočekávaná chyba: {e}")
        finally:
            sub.close()
            try:
                self.drive.send_break()
            except:
                pass
            print("🛑 [Pilot-Vision] Vlákno ukončeno.")

service = PilotVisionService()
