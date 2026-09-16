# esf_raw_handler.py
import time
from typing import Optional
from light_fusion import LightFusion

# Mapování UBX-ESF-RAW typů na indexy
# 14: gyroX, 13: gyroY, 5: gyroZ, 16: accX, 17: accY, 18: accZ, 12: temp
_TYPE_TO_IDX = {
    14: 0,  # gyroX
    13: 1,  # gyroY
    5:  2,  # gyroZ
    16: 3,  # accX
    17: 4,  # accY
    18: 5,  # accZ
    12: 6,  # temp
}

class EsfRawHandler:
    """
    Handler pro zprávu UBX-ESF-RAW (třída 0x10, ID 0x03).
    Dekóduje jednotlivé vzorky ze surového payloadu a předává je instanci LightFusion.
    """
    __slots__ = ("light_fusion", "handled_count", "_last_sttag", "_last_gyro_z")

    def __init__(self, light_fusion: LightFusion):
        self.light_fusion = light_fusion
        self.handled_count = 0
        self._last_sttag = 0
        self._last_gyro_z = 0.0

    def handle(self, msg_class: int, msg_id: int, payload: bytes) -> bool:
        """
        RT-safe zpracování payloadu UBX-ESF-RAW:
        Formát: [4B reserved][N * (4B data + 4B sTtag)]
        """
        if msg_class != 0x10 or msg_id != 0x03 or len(payload) < 12:
            return False

        self.handled_count += 1
        now = time.monotonic()

        # N = (len(payload) - 4) // 8
        n_samples = (len(payload) - 4) // 8
        if n_samples <= 0:
            return False

        base = 4
        frame = [0.0] * 7  # [gx, gy, gz, ax, ay, az, temp]
        last_sttag = 0

        for i in range(n_samples):
            o = base + i * 8
            d = int.from_bytes(payload[o:o+4], "little", signed=False)
            df = d & 0xFFFFFF
            # Sign-extend 24-bit
            if df & 0x800000:
                df -= 1 << 24
            dtype = (d >> 24) & 0xFF
            last_sttag = int.from_bytes(payload[o+4:o+8], "little", signed=False)

            idx = _TYPE_TO_IDX.get(dtype)
            if idx is not None:
                if dtype in (5, 13, 14):     # Gyro v deg/s * 2^-12
                    frame[idx] = df / 4096.0
                elif dtype in (16, 17, 18):  # Acc v m/s^2 * 2^-10
                    frame[idx] = df / 1024.0
                elif dtype == 12:            # Teplota v °C * 10^-2
                    frame[idx] = df / 100.0

        self._last_sttag = last_sttag
        self._last_gyro_z = frame[2]

        # Předání vzorku do light fusion engine
        self.light_fusion.update_sample(
            gyroX=frame[0],
            gyroY=frame[1],
            gyroZ=frame[2],
            accX=frame[3],
            accY=frame[4],
            accZ=frame[5],
            sTtag=last_sttag,
            rx_mono=now
        )
        return True

    def get_last_info(self) -> dict:
        return {
            "handled_count": self.handled_count,
            "last_sttag": self._last_sttag,
            "last_gyro_z": self._last_gyro_z
        }
