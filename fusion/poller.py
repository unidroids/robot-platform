# fusion/poller.py
import json
import math
import threading
import zmq
from typing import Optional

from core import FusionCore

class FusionPoller:
    """
    Asynchronní odběr dat pro fúzi přes ZeroMQ:
    - gnss-gps (ipc:///tmp/robot-gnss-gps):
        * topic BESTNAV -> primární anténa UM980 [+32, 0] cm a kurz trk_gnd
    - gnss-dual (ipc:///tmp/robot-gnss-dual):
        * topic BESTNAV -> Master anténa UM982 [+25, +24] cm
        * topic BESTNAVH -> Slave anténa UM982 [+25, -24] cm
        * topic UNIHEADING -> úhel z duální antény
    - gnss-imu (ipc:///tmp/robot-gnss-imu):
        * topic GYRO -> 20Hz přírůstky delta_yaw, okamžité wz, pitch, roll, ax, ay, az
    - drive (ipc:///tmp/robot-drive):
        * topic ODM -> odometrická rychlost (left_speed, right_speed)
    (Služba robot-compass je odpojena).
    """

    def __init__(
        self,
        core: FusionCore,
        endpoint_gps: str = "ipc:///tmp/robot-gnss-gps",
        endpoint_dual: str = "ipc:///tmp/robot-gnss-dual",
        endpoint_imu: str = "ipc:///tmp/robot-gnss-imu",
        endpoint_drive: str = "ipc:///tmp/robot-drive"
    ):
        self.core = core
        self.endpoint_gps = endpoint_gps
        self.endpoint_dual = endpoint_dual
        self.endpoint_imu = endpoint_imu
        self.endpoint_drive = endpoint_drive

        self.running = False
        self._zmq_context = zmq.Context.instance()

        self._sub_gps: Optional[zmq.Socket] = None
        self._sub_dual: Optional[zmq.Socket] = None
        self._sub_imu: Optional[zmq.Socket] = None
        self._sub_drive: Optional[zmq.Socket] = None

        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self.running:
            return

        # 1. gnss-gps (UM980)
        self._sub_gps = self._zmq_context.socket(zmq.SUB)
        try:
            self._sub_gps.connect(self.endpoint_gps)
            self._sub_gps.setsockopt_string(zmq.SUBSCRIBE, "BESTNAV")
        except zmq.error.ZMQError as e:
            print(f"[FusionPoller] Warning: connect to {self.endpoint_gps} failed: {e}")

        # 2. gnss-dual (UM982)
        self._sub_dual = self._zmq_context.socket(zmq.SUB)
        try:
            self._sub_dual.connect(self.endpoint_dual)
            self._sub_dual.setsockopt_string(zmq.SUBSCRIBE, "BESTNAV")
            self._sub_dual.setsockopt_string(zmq.SUBSCRIBE, "BESTNAVH")
            self._sub_dual.setsockopt_string(zmq.SUBSCRIBE, "UNIHEADING")
        except zmq.error.ZMQError as e:
            print(f"[FusionPoller] Warning: connect to {self.endpoint_dual} failed: {e}")

        # 3. gnss-imu (nová IMU služba na portu 9016)
        self._sub_imu = self._zmq_context.socket(zmq.SUB)
        try:
            self._sub_imu.connect(self.endpoint_imu)
            self._sub_imu.setsockopt_string(zmq.SUBSCRIBE, "GYRO")
        except zmq.error.ZMQError as e:
            print(f"[FusionPoller] Warning: connect to {self.endpoint_imu} failed: {e}")

        # 4. drive (hoverboard odometrie)
        self._sub_drive = self._zmq_context.socket(zmq.SUB)
        try:
            self._sub_drive.connect(self.endpoint_drive)
            self._sub_drive.setsockopt_string(zmq.SUBSCRIBE, "ODM")
        except zmq.error.ZMQError as e:
            print(f"[FusionPoller] Warning: connect to {self.endpoint_drive} failed: {e}")

        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.running = False
        for s in [self._sub_gps, self._sub_dual, self._sub_imu, self._sub_drive]:
            if s is not None:
                try:
                    s.close(linger=0)
                except Exception:
                    pass
        self._sub_gps = None
        self._sub_dual = None
        self._sub_imu = None
        self._sub_drive = None

        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _loop(self) -> None:
        poller = zmq.Poller()
        if self._sub_gps:
            poller.register(self._sub_gps, zmq.POLLIN)
        if self._sub_dual:
            poller.register(self._sub_dual, zmq.POLLIN)
        if self._sub_imu:
            poller.register(self._sub_imu, zmq.POLLIN)
        if self._sub_drive:
            poller.register(self._sub_drive, zmq.POLLIN)

        while self.running:
            try:
                socks = dict(poller.poll(100))
            except (zmq.error.ContextTerminated, zmq.error.ZMQError):
                break
            except Exception as e:
                print(f"[FusionPoller Poll Error] {e}")
                break

            try:
                if self._sub_gps in socks and socks[self._sub_gps] == zmq.POLLIN:
                    parts = self._sub_gps.recv_multipart()
                    self._handle_gps_msg(parts)

                if self._sub_dual in socks and socks[self._sub_dual] == zmq.POLLIN:
                    parts = self._sub_dual.recv_multipart()
                    self._handle_dual_msg(parts)

                if self._sub_imu in socks and socks[self._sub_imu] == zmq.POLLIN:
                    parts = self._sub_imu.recv_multipart()
                    self._handle_imu_msg(parts)

                if self._sub_drive in socks and socks[self._sub_drive] == zmq.POLLIN:
                    parts = self._sub_drive.recv_multipart()
                    self._handle_drive_msg(parts)
            except zmq.error.ContextTerminated:
                break
            except Exception as e:
                print(f"[FusionPoller Message Error] {e}")

    def _handle_gps_msg(self, parts: list) -> None:
        if len(parts) != 2:
            return
        topic = parts[0].decode('utf-8', errors='ignore')
        payload = parts[1].decode('utf-8', errors='ignore')
        if topic == "BESTNAV":
            self._process_gps_bestnav(payload)

    def _handle_dual_msg(self, parts: list) -> None:
        if len(parts) != 2:
            return
        topic = parts[0].decode('utf-8', errors='ignore')
        payload = parts[1].decode('utf-8', errors='ignore')
        if topic == "BESTNAV":
            self._process_dual_master_bestnav(payload)
        elif topic == "BESTNAVH":
            self._process_dual_slave_bestnav(payload)
        elif topic == "UNIHEADING":
            self._process_uniheading(payload)

    def _handle_imu_msg(self, parts: list) -> None:
        if len(parts) != 2:
            return
        topic = parts[0].decode('utf-8', errors='ignore')
        payload = parts[1].decode('utf-8', errors='ignore')
        if topic == "GYRO":
            try:
                data = json.loads(payload)
                ts = float(data.get("ts", 0.0))
                delta_yaw = float(data.get("delta_yaw", 0.0))
                wz = float(data.get("wz", 0.0))
                pitch = float(data.get("pitch", 0.0))
                roll = float(data.get("roll", 0.0))
                ax = float(data.get("ax", 0.0))
                ay = float(data.get("ay", 0.0))
                az = float(data.get("az", 9.81))
                self.core.update_imu(ts, delta_yaw, wz, pitch, roll, ax, ay, az)
            except Exception as e:
                print(f"[FusionPoller IMU Error] {e}")

    def _handle_drive_msg(self, parts: list) -> None:
        if len(parts) != 2:
            return
        topic = parts[0].decode('utf-8', errors='ignore')
        payload = parts[1].decode('utf-8', errors='ignore')
        if topic == "ODM":
            try:
                data = json.loads(payload)
                left = float(data.get("left_speed", 0.0))
                right = float(data.get("right_speed", 0.0))
                left_steps = data.get("left_steps")
                right_steps = data.get("right_steps")
                if left_steps is not None:
                    left_steps = int(left_steps)
                if right_steps is not None:
                    right_steps = int(right_steps)
                ts = float(data.get("ts", 0.0))
                self.core.update_odometry(left, right, left_steps=left_steps, right_steps=right_steps, ts=ts)
            except Exception as e:
                print(f"[FusionPoller Drive Error] {e}")

    def _process_gps_bestnav(self, payload: str) -> None:
        try:
            data = json.loads(payload)
            lat = float(data.get("lat", 0.0))
            lon = float(data.get("lon", 0.0))
            lat_std = float(data.get("lat_std", 0.0))
            lon_std = float(data.get("lon_std", 0.0))
            hAcc = math.hypot(lat_std, lon_std)
            gpsSol = data.get("pos_type", "NONE")

            trk_gnd = float(data.get("trk_gnd", 0.0))
            hor_spd = float(data.get("hor_spd", 0.0))
            hor_spd_std = float(data.get("hor_spd_std", 0.0))
            if hor_spd > 0.1:
                hdg_acc = math.degrees(math.atan2(hor_spd_std, hor_spd))
            else:
                hdg_acc = 180.0

            self.core.update_gps_antenna(lat, lon, hAcc, gpsSol, trk_gnd=trk_gnd, hor_spd=hor_spd)
            self.core.update_gps_heading(trk_gnd, hdg_acc, gpsSol, hor_spd=hor_spd)
        except Exception as e:
            print(f"[FusionPoller GPS BESTNAV Error] {e}")

    def _process_dual_master_bestnav(self, payload: str) -> None:
        try:
            data = json.loads(payload)
            lat = float(data.get("lat", 0.0))
            lon = float(data.get("lon", 0.0))
            lat_std = float(data.get("lat_std", 0.0))
            lon_std = float(data.get("lon_std", 0.0))
            hAcc = math.hypot(lat_std, lon_std)
            gpsSol = data.get("pos_type", "NONE")
            trk_gnd = float(data.get("trk_gnd", 0.0))
            hor_spd = float(data.get("hor_spd", 0.0))

            self.core.update_master_antenna(lat, lon, hAcc, gpsSol, trk_gnd=trk_gnd, hor_spd=hor_spd)
        except Exception as e:
            print(f"[FusionPoller Dual Master Error] {e}")

    def _process_dual_slave_bestnav(self, payload: str) -> None:
        try:
            data = json.loads(payload)
            lat = float(data.get("lat", 0.0))
            lon = float(data.get("lon", 0.0))
            lat_std = float(data.get("lat_std", 0.0))
            lon_std = float(data.get("lon_std", 0.0))
            hAcc = math.hypot(lat_std, lon_std)
            gpsSol = data.get("pos_type", "NONE")
            trk_gnd = float(data.get("trk_gnd", 0.0))
            hor_spd = float(data.get("hor_spd", 0.0))

            self.core.update_slave_antenna(lat, lon, hAcc, gpsSol, trk_gnd=trk_gnd, hor_spd=hor_spd)
        except Exception as e:
            print(f"[FusionPoller Dual Slave Error] {e}")

    def _process_uniheading(self, payload: str) -> None:
        try:
            data = json.loads(payload)
            heading = float(data.get("heading", 0.0))
            hdg_std = float(data.get("hdg_std", 180.0))
            headingSol = data.get("pos_type", "NONE")
            length = float(data.get("length", 0.0))

            self.core.update_dual_heading(heading, hdg_std, headingSol, length)
        except Exception as e:
            print(f"[FusionPoller UNIHEADING Error] {e}")
