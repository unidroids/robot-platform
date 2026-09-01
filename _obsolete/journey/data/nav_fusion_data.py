# data/nav_fusion_data.py
from __future__ import annotations
from dataclasses import dataclass
import struct
from typing import ClassVar
import json


@dataclass
class NavFusionData:
    """
    Kompaktní 2D stav pro PILOTA, verze 1.

    Binární formát (Little-Endian), velikost 51 B:

        B      version     (uint8)   - musí být 1
        d      ts_mono     (float64) - monotonic čas [s]
        d      lat         (float64) - WGS84 [deg]
        d      lon         (float64) - WGS84 [deg]
        f      hAcc        (float32) - horizontální přesnost [m]
        f      heading     (float32) - heading [deg]
        f      headingAcc  (float32) - přesnost heading [deg]
        f      speed       (float32) - rychlost [m/s]
        f      sAcc        (float32) - přesnost rychlosti [m/s]
        f      gyroZ       (float32) - gyro Z [deg/s]
        f      gyroZAcc    (float32) - přesnost gyroZ [deg/s]
        B      gnssFixOK   (uint8)   - 0/1
        B      drUsed      (uint8)   - 0/1
    """

    VERSION: ClassVar[int] = 1

    # --- čas ---
    ts_mono: float

    # --- poloha/orientace/pohyb ---
    lat: float
    lon: float
    hAcc: float
    heading: float
    headingAcc: float
    speed: float
    sAcc: float
    gyroZ: float
    gyroZAcc: float

    # --- flagy ---
    gnssFixOK: bool
    drUsed: bool

    # --- binární formát ---
    _STRUCT_FMT: ClassVar[str] = "<B d d d f f f f f f f B B"
    _STRUCT: ClassVar[struct.Struct] = struct.Struct(_STRUCT_FMT)

    # --- API ---
    def to_bytes(self) -> bytes:
        """Zabalí objekt do LE binárního streamu (51 B)."""
        return self._STRUCT.pack(
            int(self.VERSION),
            float(self.ts_mono),
            float(self.lat),
            float(self.lon),
            float(self.hAcc),
            float(self.heading),
            float(self.headingAcc),
            float(self.speed),
            float(self.sAcc),
            float(self.gyroZ),
            float(self.gyroZAcc),
            1 if bool(self.gnssFixOK) else 0,
            1 if bool(self.drUsed) else 0,
        )

    @classmethod
    def from_bytes(cls, blob: bytes) -> "NavFusionData":
        """Načte objekt z binárního streamu (51 B)."""
        if len(blob) != cls._STRUCT.size:
            raise ValueError(f"Expected {cls._STRUCT.size} bytes, got {len(blob)}")
        unpacked = cls._STRUCT.unpack(blob)

        version = int(unpacked[0])
        if version != cls.VERSION:
            raise ValueError(f"Unsupported version {version} (expected {cls.VERSION})")

        return cls(
            ts_mono=float(unpacked[1]),
            lat=float(unpacked[2]),
            lon=float(unpacked[3]),
            hAcc=float(unpacked[4]),
            heading=float(unpacked[5]),
            headingAcc=float(unpacked[6]),
            speed=float(unpacked[7]),
            sAcc=float(unpacked[8]),
            gyroZ=float(unpacked[9]),
            gyroZAcc=float(unpacked[10]),
            gnssFixOK=bool(unpacked[11]),
            drUsed=bool(unpacked[12]),
        )

    @classmethod
    def byte_size(cls) -> int:
        """Vrátí velikost binární reprezentace (51 B)."""
        return cls._STRUCT.size

    def to_json(self) -> str:
        """Vrátí obsah objektu jako JSON string."""
        return json.dumps({
            "ts_mono": self.ts_mono,
            "lat": self.lat,
            "lon": self.lon,
            "hAcc": self.hAcc,
            "heading": self.heading,
            "headingAcc": self.headingAcc,
            "speed": self.speed,
            "sAcc": self.sAcc,
            "gyroZ": self.gyroZ,
            "gyroZAcc": self.gyroZAcc,
            "gnssFixOK": bool(self.gnssFixOK),
            "drUsed": bool(self.drUsed),
        })

    @classmethod
    def from_json(cls, json_str: str) -> "NavFusionData":
        """
        Vytvoří NavFusionData z JSON stringu (např. z odpovědi příkazu GNSS:DATA).

        Očekávané klíče odpovídají `to_json()` (ts_mono, lat, lon, hAcc, heading, headingAcc,
        speed, sAcc, gyroZ, gyroZAcc, gnssFixOK, drUsed).
        """
        s = json_str.strip()
        obj = json.loads(s)
        return cls(
            ts_mono=float(obj.get("ts_mono", 0.0)),
            lat=float(obj["lat"]),
            lon=float(obj["lon"]),
            hAcc=float(obj.get("hAcc", float("inf"))),
            heading=float(obj.get("heading", 0.0)),
            headingAcc=float(obj.get("headingAcc", float("inf"))),
            speed=float(obj.get("speed", 0.0)),
            sAcc=float(obj.get("sAcc", float("inf"))),
            gyroZ=float(obj.get("gyroZ", 0.0)),
            gyroZAcc=float(obj.get("gyroZAcc", float("inf"))),
            gnssFixOK=bool(obj.get("gnssFixOK", False)),
            drUsed=bool(obj.get("drUsed", False)),
        )

    @classmethod
    def from_json_bytes(cls, json_bytes: bytes) -> "NavFusionData":
        """Dekóduje UTF-8 bytes (často zakončené \n) a zavolá from_json()."""
        return cls.from_json(json_bytes.decode("utf-8", errors="strict"))


# --- self-test ---
if __name__ == "__main__":
    state = NavFusionData(
        ts_mono=12345.678,
        lat=49.0001234,
        lon=17.0005678,
        hAcc=0.25,
        heading=92.4,
        headingAcc=1.2,
        speed=0.54,
        sAcc=0.05,
        gyroZ=-12.3,
        gyroZAcc=0.8,
        gnssFixOK=True,
        drUsed=False,
    )
    blob = state.to_bytes()
    print("Byte size:", len(blob), "expected:", NavFusionData.byte_size())
    restored = NavFusionData.from_bytes(blob)
    print("Restored:", restored)
    print("to_json:", restored.to_json())
