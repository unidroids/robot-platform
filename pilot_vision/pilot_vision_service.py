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

    def start(self, token: str) -> bool:
        if self.control.is_running and self.receiver.is_running:
            # Už běží, jen aktualizujeme token
            self.active_token = token
            self.control.update_token(token)
            return False

        self.active_token = token
        print(f"🚀 [PilotVisionService] Startuji služby (Token: {token})")
        self.receiver.start()
        self.control.start(token)
        return True

    def stop(self) -> bool:
        if not self.control.is_running and not self.receiver.is_running:
            return False
            
        print("🛑 [PilotVisionService] Zastavuji služby...")
        self.control.stop()
        self.receiver.stop()
        self.active_token = None
        return True

    def get_status(self) -> str:
        if self.control.is_running and self.receiver.is_running:
            return f"RUNNING TOKEN={self.active_token}"
        return "STOPPED"

# Globální instance služby pro API endpointy
service = PilotVisionService()
