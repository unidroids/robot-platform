import threading
import time

class RobotState:
    """
    Sdílený objekt zapouzdřující aktuální stav robota ze všech senzorů.
    Vlákno přijímající data (DataReceiver) sem zapisuje, 
    zatímco řídící vlákno (ControlLoop) odtud čte.
    """
    def __init__(self):
        self._lock = threading.Lock()
        
        # Vision data
        self.vision_pose_lines = []
        self.last_vision_time = 0.0
        
        # Lidar data
        self.lidar_distance = -1.0
        self.last_lidar_time = 0.0
        
        # Odometry data
        self.odom_speed_left = 0
        self.odom_speed_right = 0
        self.last_odom_time = 0.0
        
    def update_vision(self, pose_lines):
        with self._lock:
            self.vision_pose_lines = pose_lines
            self.last_vision_time = time.time()
            
    def update_lidar(self, distance):
        with self._lock:
            self.lidar_distance = distance
            self.last_lidar_time = time.time()
            
    def update_odom(self, left, right):
        with self._lock:
            self.odom_speed_left = left
            self.odom_speed_right = right
            self.last_odom_time = time.time()
            
    def get_snapshot(self):
        """Vrátí kopii aktuálního stavu pro rychlé čtení v řídící smyčce."""
        with self._lock:
            return {
                "vision_pose_lines": list(self.vision_pose_lines),
                "last_vision_time": self.last_vision_time,
                "lidar_distance": self.lidar_distance,
                "last_lidar_time": self.last_lidar_time,
                "odom_speed_left": self.odom_speed_left,
                "odom_speed_right": self.odom_speed_right,
                "last_odom_time": self.last_odom_time
            }
