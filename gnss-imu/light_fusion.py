# light_fusion.py
import math
import threading
import time
from typing import Optional, Tuple, List

# Orientace: -1.0 převádí standardní ENU gyroZ (kde CCW je +) na kompasovou orientaci (kde CW / doprava je +)
DEFAULT_ORIENTATION_SIGN = -1.0

class LightFusion:
    """
    Light fúze pro zpracování 100Hz vzorků z UBX-ESF-RAW:
    - 2s synchronní kalibrace nulového biasu s detekcí klidu přes akcelerometr
    - Výpočet normálového gravitačního vektoru g0 a kalibrace nulového náklonu (pitch_bias, roll_bias)
    - Výpočet časového kroku dt z hardwarového sTtag (s ošetřením 32-bit rolloveru)
    - Numerická integrace přírůstku úhlu delta_yaw kolem osy Z (kompasová konvence: CW = +)
    - Dynamický komplementární filtr pro předklon (pitch) a boční náklon (roll)
    - Poskytování delta_yaw, wz, pitch, roll a zrychlení pro 20Hz publikaci
    """

    def __init__(self, orientation_sign: float = DEFAULT_ORIENTATION_SIGN):
        self.orientation_sign = orientation_sign
        self._lock = threading.Lock()

        # Kalibrace
        self.is_calibrating = False
        self.bias_z = 0.0
        self.pitch_bias = 0.0
        self.roll_bias = 0.0
        self.gravity_norm = 9.81
        self._calib_gyro_samples: List[float] = []
        self._calib_acc_samples: List[Tuple[float, float, float]] = []

        # Stav integrace (100 Hz)
        self._last_sTtag: Optional[int] = None
        self._accumulated_delta_yaw: float = 0.0
        self._latest_wz: float = 0.0
        self._latest_ts: float = 0.0
        self._sample_count: int = 0
        self._total_samples: int = 0

        # Stav náklonu (pitch, roll) a zrychlení
        self._pitch: Optional[float] = None
        self._roll: Optional[float] = None
        self._latest_ax: float = 0.0
        self._latest_ay: float = 0.0
        self._latest_az: float = 9.81

    def start_calibration(self) -> None:
        """Přepne fúzi do režimu sběru vzorků pro kalibraci."""
        with self._lock:
            self.is_calibrating = True
            self._calib_gyro_samples.clear()
            self._calib_acc_samples.clear()
            self._last_sTtag = None
            self._accumulated_delta_yaw = 0.0
            self._sample_count = 0
            self._pitch = None
            self._roll = None

    def finish_calibration(self, max_acc_std: float = 0.6, max_gyro_std: float = 1.5) -> Tuple[bool, str]:
        """
        Ukončí kalibraci, vyhodnotí klid robota, spočítá gyro bias a gravitační vektor (nulový náklon).
        Vrací (úspěch: bool, zpráva: str).
        """
        with self._lock:
            self.is_calibrating = False
            n = len(self._calib_gyro_samples)
            if n < 30:
                return False, f"ERR NOT ENOUGH SAMPLES ({n} < 30)"

            # Kontrola stability akcelerometru (klid robota)
            norms = [math.sqrt(ax * ax + ay * ay + az * az) for ax, ay, az in self._calib_acc_samples]
            mean_norm = sum(norms) / n
            acc_var = sum((x - mean_norm) ** 2 for x in norms) / n
            acc_std = math.sqrt(acc_var)

            # Rozptyl na gyru Z
            mean_gyro = sum(self._calib_gyro_samples) / n
            gyro_var = sum((x - mean_gyro) ** 2 for x in self._calib_gyro_samples) / n
            gyro_std = math.sqrt(gyro_var)

            if acc_std > max_acc_std or gyro_std > max_gyro_std:
                return False, f"ERR MOTION DETECTED (acc_std={acc_std:.3f}>{max_acc_std}, gyro_std={gyro_std:.3f}>{max_gyro_std})"

            # Průměrný vektor tíhového zrychlení v klidu
            mean_ax = sum(ax for ax, ay, az in self._calib_acc_samples) / n
            mean_ay = sum(ay for ax, ay, az in self._calib_acc_samples) / n
            mean_az = sum(az for ax, ay, az in self._calib_acc_samples) / n

            self.bias_z = mean_gyro
            self.gravity_norm = mean_norm

            # Referenční úhly náklonu při stání na rovině (montážní offset senzoru)
            self.pitch_bias = math.degrees(math.atan2(mean_ax, math.sqrt(mean_ay * mean_ay + mean_az * mean_az)))
            self.roll_bias = math.degrees(math.atan2(mean_ay, mean_az))

            self._accumulated_delta_yaw = 0.0
            self._last_sTtag = None
            self._sample_count = 0
            self._pitch = None
            self._roll = None

            return True, f"OK bias_z={self.bias_z:.4f} pitch_0={self.pitch_bias:.2f}° roll_0={self.roll_bias:.2f}° g={self.gravity_norm:.2f} samples={n}"

    def update_sample(
        self,
        gyroX: float,
        gyroY: float,
        gyroZ: float,
        accX: float,
        accY: float,
        accZ: float,
        sTtag: int,
        rx_mono: float
    ) -> None:
        """
        Zpracování jednoho 100Hz vzorku ze zprávy UBX-ESF-RAW.
        """
        with self._lock:
            self._total_samples += 1

            if self.is_calibrating:
                self._calib_gyro_samples.append(gyroZ)
                self._calib_acc_samples.append((accX, accY, accZ))
                return

            # Výpočet delta t ze senzorového sTtag (v ms)
            if self._last_sTtag is not None:
                # 32-bit unsigned rollover
                dt_ms = (sTtag - self._last_sTtag) & 0xFFFFFFFF
                if 0 < dt_ms < 500:  # Ochrana před výpadky senzoru delšími než 500 ms
                    dt_sec = dt_ms / 1000.0
                else:
                    dt_sec = 0.01  # Nominální hodnota pro 100 Hz
            else:
                dt_sec = 0.0

            self._last_sTtag = sTtag

            # 1. Yaw & Wz: odečtení biasu a převod na kompasovou konvenci (CW = +)
            wz_calib = (gyroZ - self.bias_z) * self.orientation_sign
            self._accumulated_delta_yaw += wz_calib * dt_sec
            self._latest_wz = wz_calib
            self._latest_ts = rx_mono
            self._sample_count += 1

            # 2. Zrychlení
            self._latest_ax = accX
            self._latest_ay = accY
            self._latest_az = accZ

            # 3. Pitch & Roll: statický odhad z akcelerometru minus kalibrovaný offset
            acc_pitch = math.degrees(math.atan2(accX, math.sqrt(accY * accY + accZ * accZ))) - self.pitch_bias
            acc_roll = math.degrees(math.atan2(accY, accZ)) - self.roll_bias

            # Komplementární filtr: 98 % integrace z gyra, 2 % dotahování k akcelerometru
            if self._pitch is None or dt_sec <= 0:
                self._pitch = acc_pitch
                self._roll = acc_roll
            else:
                alpha = 0.98
                # gyroY je rychlost klopení (pitch rate), gyroX je rychlost klonění (roll rate)
                self._pitch = alpha * (self._pitch + gyroY * dt_sec) + (1.0 - alpha) * acc_pitch
                self._roll = alpha * (self._roll + gyroX * dt_sec) + (1.0 - alpha) * acc_roll

    def pop_20hz_increment(self) -> Tuple[float, float, float, int, float, float, float, float, float]:
        """
        Atomicky vrátí:
        (ts, delta_yaw, wz, samples_count, pitch, roll, ax, ay, az)
        za uplynulou 20Hz periodu a vynuluje akumulátor delta_yaw pro další periodu.
        """
        with self._lock:
            ts = self._latest_ts if self._latest_ts > 0.0 else time.monotonic()
            delta_yaw = self._accumulated_delta_yaw
            wz = self._latest_wz
            samples = self._sample_count
            pitch = self._pitch if self._pitch is not None else 0.0
            roll = self._roll if self._roll is not None else 0.0
            ax = self._latest_ax
            ay = self._latest_ay
            az = self._latest_az

            # Nulování pro další 20Hz periodu
            self._accumulated_delta_yaw = 0.0
            self._sample_count = 0

            return ts, delta_yaw, wz, samples, pitch, roll, ax, ay, az

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "bias_z": self.bias_z,
                "pitch_bias": self.pitch_bias,
                "roll_bias": self.roll_bias,
                "gravity_norm": self.gravity_norm,
                "latest_pitch": self._pitch if self._pitch is not None else 0.0,
                "latest_roll": self._roll if self._roll is not None else 0.0,
                "total_samples": self._total_samples,
                "latest_wz": self._latest_wz,
                "is_calibrating": self.is_calibrating
            }
