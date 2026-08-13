import threading
import json
import zmq
import math
from core import FusionCore

class FusionPoller:
    def __init__(self, core: FusionCore):
        self.core = core
        self.running = False
        self._zmq_context = zmq.Context.instance()
        self._zmq_sub = None
        self._thread = None

    def start(self):
        if self.running: return
        self._zmq_sub = self._zmq_context.socket(zmq.SUB)
        self._zmq_sub.connect("ipc:///tmp/robot-gnss-dual")
        self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "BESTNAV")
        self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "UNIHEADING")
        self._zmq_sub.connect("ipc:///tmp/robot-compass")
        self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "GYRO")
        self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "ANGLE")
        self._zmq_sub.connect("ipc:///tmp/robot-drive")
        self._zmq_sub.setsockopt_string(zmq.SUBSCRIBE, "ODM")

        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._zmq_sub:
            self._zmq_sub.close()
            self._zmq_sub = None
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _loop(self):
        while self.running and self._zmq_sub is not None:
            try:
                if self._zmq_sub.poll(100):
                    parts = self._zmq_sub.recv_multipart()
                    self._handle_msg(parts)
            except zmq.error.ContextTerminated:
                break
            except zmq.error.ZMQError:
                break
            except Exception as e:
                print(f"[ZMQ Listener Error] {e}")

    def _handle_msg(self, parts: list):
        if len(parts) == 2:
            topic = parts[0].decode('utf-8', errors='ignore')
            payload = parts[1].decode('utf-8', errors='ignore')
            if topic == "BESTNAV":
                self._process_bestnav(payload)
            elif topic == "UNIHEADING":
                self._process_uniheading(payload)
            elif topic == "ODM":
                data = json.loads(payload)
                left = data.get("left_speed", 0.0)
                right = data.get("right_speed", 0.0)
                self.core.update_odometry(left, right)
            elif topic == "GYRO":
                data = json.loads(payload)
                wz = data.get("wz", 0.0)
                ts = data.get("ts", 0.0)
                self.core.update_gyro(ts, wz)
            elif topic == "ANGLE":
                data = json.loads(payload)
                yaw = data.get("yaw", 0.0)
                self.core.update_compass_angle(yaw)
            else:
                print(f"[FusionPoller] Neznámý topic: {topic}")
        else:
            first_frame = parts[0].decode('utf-8', errors='ignore') if len(parts) > 0 else "EMPTY"
            print(f"[FusionPoller] Neplatný formát zprávy (očekáváno len=2), přijato parts: {len(parts)}, první frame: '{first_frame}'")

    def _process_bestnav(self, payload: str):
        data = json.loads(payload)
        lat = data.get("lat", 0.0)
        lon = data.get("lon", 0.0)
        lat_std = data.get("lat_std", 0.0)
        lon_std = data.get("lon_std", 0.0)
        hAcc = math.hypot(lat_std, lon_std)
        gpsSol = data.get("pos_type", "NONE")
        self.core.update_position(lat, lon, hAcc, gpsSol) 
        
        trk_gnd = data.get("trk_gnd", 0.0)
        hor_spd = data.get("hor_spd", 0.0)
        hor_spd_std = data.get("hor_spd_std", 0.0)
        if hor_spd > 0.1:
            hdg_acc = math.degrees(math.atan2(hor_spd_std, hor_spd))
        else:
            hdg_acc = 180.0
        self.core.update_gps_heading(trk_gnd, hdg_acc, gpsSol)
            
            
    def _process_uniheading(self, payload: str):
        data = json.loads(payload)
        heading = data.get("heading", 0.0)
        hdg_std = data.get("hdg_std", 180.0)
        headingSol = data.get("pos_type", "NONE")
        length = data.get("length", 0.0)
        self.core.update_dual_heading(heading, hdg_std, headingSol, length)
