# service.py
import threading
import time
import json
import zmq
from datetime import datetime
from typing import Optional

from gnss_serial import GnssSerialIO, DEFAULT_DEVICE, DEFAULT_BAUDRATE
from light_fusion import LightFusion
from handlers.esf_raw_handler import EsfRawHandler

ZMQ_IPC_ENDPOINT = "ipc:///tmp/robot-gnss-imu"
ZMQ_TOPIC = b"GYRO"
PUBLISH_INTERVAL_SEC = 0.05  # 20 Hz (50 ms)
CALIBRATION_DURATION_SEC = 2.0  # 2 sekundy v klidu pro určení biasu

class GnssImuService:
    """
    Hlavní služba GNSS-IMU (port 9016):
    - Řídí životní cyklus (START, STOP, STATUS)
    - Propojuje GnssSerialIO -> EsfRawHandler -> LightFusion
    - Provádí synchronní 2s kalibraci nulového bodu při STARTu s kontrolou klidu
    - Publikuje 20Hz přírůstky úhlu (delta_yaw) a okamžitou úhlovou rychlost (wz) přes ZMQ
    """

    def __init__(self, device: str = DEFAULT_DEVICE, baudrate: int = DEFAULT_BAUDRATE):
        self.device = device
        self.baudrate = baudrate

        self.running = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self.gnss_serial: Optional[GnssSerialIO] = None
        self.light_fusion = LightFusion()
        self.esf_raw_handler = EsfRawHandler(self.light_fusion)

        self.zmq_context: Optional[zmq.Context] = None
        self.zmq_pub: Optional[zmq.Socket] = None

        self._dispatcher_thread: Optional[threading.Thread] = None
        self._publisher_thread: Optional[threading.Thread] = None

        self.stats_published = 0
        self.last_published_json = "{}"

    def start(self) -> str:
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"

            # 1. Inicializace ZMQ
            self.zmq_context = zmq.Context.instance()
            self.zmq_pub = self.zmq_context.socket(zmq.PUB)
            self.zmq_pub.bind(ZMQ_IPC_ENDPOINT)

            # 2. Otevření sériového portu (čas logu je dán okamžikem příkazu START)
            start_time = datetime.now()
            self.gnss_serial = GnssSerialIO(self.device, self.baudrate)
            try:
                self.gnss_serial.open(start_time=start_time)
            except Exception as e:
                self._cleanup_resources()
                return f"ERR SERIAL_OPEN_FAILED: {e}"

            self._stop_event.clear()

            # 3. Spuštění čtecího/dispečerského vlákna pro příjem ESF-RAW
            self._dispatcher_thread = threading.Thread(target=self._dispatcher_loop, daemon=True)
            self._dispatcher_thread.start()

            # 4. Synchronní 2sekundová kalibrace gyra v klidu
            print(f"[SERVICE] Starting {CALIBRATION_DURATION_SEC}s stationary calibration...")
            self.light_fusion.start_calibration()
            time.sleep(CALIBRATION_DURATION_SEC)
            calib_ok, calib_msg = self.light_fusion.finish_calibration()

            if not calib_ok:
                print(f"[SERVICE] Calibration failed: {calib_msg}")
                self._stop_event.set()
                self._cleanup_resources()
                return calib_msg

            print(f"[SERVICE] Calibration succeeded: {calib_msg}")

            # 5. Spuštění 20Hz publisher vlákna
            self.stats_published = 0
            self._publisher_thread = threading.Thread(target=self._publisher_loop, daemon=True)
            self._publisher_thread.start()

            self.running = True
            print("[SERVICE] GNSS-IMU STARTED")
            return "OK"

    def stop(self) -> str:
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"

            self._stop_event.set()
            self._cleanup_resources()
            self.running = False
            print("[SERVICE] GNSS-IMU STOPPED")
            return "OK"

    def _cleanup_resources(self) -> None:
        """Bezpečné uvolnění prostředků a vláken."""
        if self._publisher_thread and self._publisher_thread.is_alive():
            self._publisher_thread.join(timeout=1.0)
            self._publisher_thread = None

        if self.gnss_serial:
            self.gnss_serial.close()
            self.gnss_serial = None

        if self._dispatcher_thread and self._dispatcher_thread.is_alive():
            self._dispatcher_thread.join(timeout=1.0)
            self._dispatcher_thread = None

        if self.zmq_pub:
            try:
                self.zmq_pub.close(linger=0)
            except Exception:
                pass
            self.zmq_pub = None

    def get_status(self) -> str:
        with self._lock:
            if not self.running:
                return "IDLE"

            corrupted, chk_err = self.gnss_serial.get_error_counters() if self.gnss_serial else (0, 0)
            fusion_stats = self.light_fusion.get_stats()
            handled = self.esf_raw_handler.handled_count

            status_data = {
                "handled": handled,
                "published": self.stats_published,
                "bias_z": round(fusion_stats.get("bias_z", 0.0), 4),
                "latest_wz": round(fusion_stats.get("latest_wz", 0.0), 3),
                "rx_log": self.gnss_serial.rx_log_path if self.gnss_serial else None,
                "corrupted": corrupted,
                "chk_err": chk_err
            }
            return f"RUNNING {json.dumps(status_data)} {self.last_published_json}"

    def _dispatcher_loop(self) -> None:
        """Smyčka vyčítající UBX zprávy z GnssSerialIO a volající handler."""
        while not self._stop_event.is_set():
            if not self.gnss_serial:
                break
            msg = self.gnss_serial.get_message(timeout=0.1)
            if msg:
                msg_class, msg_id, payload = msg
                # Reagujeme na UBX-ESF-RAW (0x10, 0x03)
                if msg_class == 0x10 and msg_id == 0x03:
                    self.esf_raw_handler.handle(msg_class, msg_id, payload)

    def _publisher_loop(self) -> None:
        """Smyčka publikující fúzovaná data každých 50 ms (20 Hz) přes ZeroMQ."""
        next_publish_time = time.monotonic() + PUBLISH_INTERVAL_SEC
        last_log_time = time.monotonic()

        while not self._stop_event.is_set():
            now = time.monotonic()
            sleep_duration = next_publish_time - now
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            next_publish_time += PUBLISH_INTERVAL_SEC
            if next_publish_time < time.monotonic():
                next_publish_time = time.monotonic() + PUBLISH_INTERVAL_SEC

            ts, delta_yaw, wz, samples = self.light_fusion.pop_20hz_increment()

            data = {
                "ts": round(ts, 4),
                "delta_yaw": round(delta_yaw, 4),
                "wz": round(wz, 4)
            }
            json_str = json.dumps(data)
            self.last_published_json = json_str

            if self.zmq_pub:
                try:
                    self.zmq_pub.send_multipart([ZMQ_TOPIC, json_str.encode('utf-8')])
                    self.stats_published += 1
                except Exception as e:
                    print(f"[SERVICE] ZMQ publish error: {e}")

            # Výpis do logu cca 1x za 2 sekundy pro monitoring
            if time.monotonic() - last_log_time >= 2.0:
                print(f"[IMU 20Hz] {json_str} (samples={samples})")
                last_log_time = time.monotonic()
