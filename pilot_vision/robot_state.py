import threading
import time
import math

class PursuitPointTracker:
    def __init__(self):
        self.state = "INIT"  # INIT, WARMUP, READY
        self.msg_count = 0
        self.init_points = []
        
        self.x_pamet = 0.0
        self.y_pamet = 0.0
        self.lost_frames = 0
        
    def reset(self):
        self.state = "INIT"
        self.msg_count = 0
        self.init_points = []
        self.lost_frames = 0
        
    def _get_2m_point(self, pts):
        if not pts:
            return None
            
        dist = 0.0
        last_x, last_y = 0.0, 0.0
        for i, pt in enumerate(pts):
            x, y = pt.get('x', 0.0), pt.get('y', 0.0)
            if i == 0:
                d = math.hypot(x, y)
            else:
                d = math.hypot(x - last_x, y - last_y)
                
            if dist + d >= 2.0:
                remain = 2.0 - dist
                ratio = remain / d if d > 0 else 0
                interp_x = last_x + ratio * (x - last_x)
                interp_y = last_y + ratio * (y - last_y)
                return interp_x, interp_y
                
            dist += d
            last_x, last_y = x, y
            
        return last_x, last_y

    def update(self, pose_lines):
        valid_lines = [l for l in pose_lines if l.get("line_conf", 0.0) > 0.5]
        
        points_2m = []
        for l in valid_lines:
            pts = l.get("points", [])
            pt = self._get_2m_point(pts)
            if pt:
                points_2m.append(pt)
                
        if self.state == "INIT":
            if points_2m:
                self.init_points.extend(points_2m)
            self.msg_count += 1
            if self.msg_count >= 40:
                if self.init_points:
                    self.x_pamet = sum(p[0] for p in self.init_points) / len(self.init_points)
                    self.y_pamet = sum(p[1] for p in self.init_points) / len(self.init_points)
                    self.state = "WARMUP"
                    self.msg_count = 0
                else:
                    self.reset()
            return

        best_pt = None
        min_dist = float('inf')
        for pt in points_2m:
            d = math.hypot(pt[0] - self.x_pamet, pt[1] - self.y_pamet)
            if d < min_dist:
                min_dist = d
                best_pt = pt
                
        if best_pt and min_dist <= 0.25:
            self.x_pamet = best_pt[0] * 0.3 + self.x_pamet * 0.7
            self.y_pamet = best_pt[1] * 0.3 + self.y_pamet * 0.7
            self.lost_frames = 0
            
            if self.state == "WARMUP":
                self.msg_count += 1
                if self.msg_count >= 20:
                    self.state = "READY"
        else:
            self.lost_frames += 1
            if self.lost_frames >= 10:
                self.reset()

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
        self.pursuit_tracker = PursuitPointTracker()
        
        # Lidar data
        self.lidar_distance = -1.0
        self.last_lidar_time = 0.0
        
        # Odometry data
        self.odom_speed_left = 0
        self.odom_speed_right = 0
        self.last_odom_time = 0.0
        
    def reset(self):
        with self._lock:
            self.vision_pose_lines = []
            self.last_vision_time = 0.0
            self.pursuit_tracker.reset()
            self.lidar_distance = -1.0
            self.last_lidar_time = 0.0
            self.odom_speed_left = 0
            self.odom_speed_right = 0
            self.last_odom_time = 0.0

    def update_vision(self, pose_lines):
        with self._lock:
            self.vision_pose_lines = pose_lines
            self.last_vision_time = time.time()
            self.pursuit_tracker.update(pose_lines)
            
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
            pursuit_ready = self.pursuit_tracker.state == "READY"
            pursuit_target = (self.pursuit_tracker.x_pamet, self.pursuit_tracker.y_pamet) if self.pursuit_tracker.state != "INIT" else None

            return {
                "vision_pose_lines": list(self.vision_pose_lines),
                "last_vision_time": self.last_vision_time,
                "lidar_distance": self.lidar_distance,
                "last_lidar_time": self.last_lidar_time,
                "odom_speed_left": self.odom_speed_left,
                "odom_speed_right": self.odom_speed_right,
                "last_odom_time": self.last_odom_time,
                "pursuit_ready": pursuit_ready,
                "pursuit_target": pursuit_target,
                "pursuit_state": self.pursuit_tracker.state
            }
