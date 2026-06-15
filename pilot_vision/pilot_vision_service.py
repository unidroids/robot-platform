from robot_state import RobotState
from data_receiver import DataReceiver
from control_loop import ControlLoop

class PilotVisionService:
    """
    Služba sjednocující logiku přijímání dat a řízení.
    Vystavuje čisté rozhraní pro start a stop celého pilot-vision procesu.
    """
    def __init__(self):
        self.state = RobotState()
        self.receiver = DataReceiver(self.state)
        self.control = ControlLoop(self.state)
        
        self.active_token = None

    def start(self, token: str, max_speed: int = 150, max_pwm: int = 150) -> bool:
        if self.control.is_running and self.receiver.is_running:
            # Už běží, jen aktualizujeme token
            self.active_token = token
            self.control.update_token(token)
            self.control.update_params(max_speed, max_pwm)
            return False

        self.state.reset()
        self.active_token = token
        print(f"🚀 [PilotVisionService] Startuji služby (Token: {token}, MaxSpeed: {max_speed}, MaxPWM: {max_pwm})")
        self.receiver.start()
        self.control.start(token, max_speed, max_pwm)
        return True

    def stop(self) -> bool:
        if not self.control.is_running and not self.receiver.is_running:
            return False
            
        print("🛑 [PilotVisionService] Zastavuji služby...")
        self.control.stop()
        self.receiver.stop()
        self.state.reset()
        self.active_token = None
        return True

    def pause(self) -> bool:
        if not self.control.is_running or self.control.is_paused:
            return False
        
        print("⏸️ [PilotVisionService] Pozastavuji (PAUSE)...")
        self.control.pause()
        return True

    def resume(self) -> bool:
        if not self.control.is_running or not self.control.is_paused:
            return False
            
        print("▶️ [PilotVisionService] Obnovuji (RESUME)...")
        self.control.resume()
        return True

    def get_status(self) -> str:
        if self.control.is_running and self.receiver.is_running:
            if self.control.is_paused:
                return f"PAUSED TOKEN={self.active_token}"
            return f"RUNNING TOKEN={self.active_token}"
        return "STOPPED"

# Globální instance služby pro API endpointy
service = PilotVisionService()
