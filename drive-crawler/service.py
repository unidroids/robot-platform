import time
import threading
from typing import Any, Dict, Optional
import serial

class DriveServiceConfig:
    def __init__(self):
        self.device = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_32002C001951333230363332-if02"
        self.baudrate = 115200

class DriveService:
    """Vysoká vrstva pro ovládání přímo řízených motorů (Drive-Crawler) přes Lua skript."""

    def __init__(self, cfg: Optional[DriveServiceConfig] = None):
        self.cfg = cfg or DriveServiceConfig()
        self._ser: Optional[serial.Serial] = None
        
        self._lock = threading.Lock()
        self._tx_mutex = threading.Lock()  # serializace TX příkazů
        self._running = False
        self._started_at = 0.0
        self._last_cmd_at = 0.0
        self._tx_ok = 0
        self._tx_fail = 0
        self._active_token: Optional[str] = None

    # --------------- lifecycle ---------------
    def start(self, token: Optional[str] = None) -> str:
        """Spustí službu: otevře UART spojení."""
        with self._lock:
            self._active_token = token
            if self._running:
                return f"RUNNING (token={self._active_token})"
            
            try:
                print(f"[DRIVE-CRAWLER] Opening serial port {self.cfg.device} @ {self.cfg.baudrate}...")
                self._ser = serial.Serial(self.cfg.device, self.cfg.baudrate, timeout=1, exclusive=False)
                self._running = True
                self._started_at = time.monotonic()
                print("[DRIVE-CRAWLER] Serial port opened successfully.")
            except Exception as e:
                print(f"[DRIVE-CRAWLER] Error opening serial port: {e}")
                raise RuntimeError(f"Failed to open serial port {self.cfg.device}: {e}")
                
        return f"RUNNING (token={self._active_token})"

    def stop(self, force: bool = False) -> str:
        """Zastaví službu: pošle STOP motorům a zavře UART."""
        stop_err = None
        try:
            self.motors_off()
        except Exception as e:
            stop_err = e
            if not force:
                raise

        with self._lock:
            if not self._running:
                return "STOPPED"
            if self._ser:
                try:
                    print(f"[DRIVE-CRAWLER] Closing serial port...")
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self._running = False
            print("[DRIVE-CRAWLER] Serial port closed.")

        if stop_err and force:
            print(f"[DRIVE-CRAWLER] stop(force=True): ignoring motors_stop error: {type(stop_err).__name__}: {stop_err}")
        return "STOPPED"

    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def is_token_required(self) -> bool:
        """Vrátí True, pokud je aktuálně vyžadován token pro řízení."""
        with self._lock:
            return self._active_token is not None

    def check_token(self, token: str) -> bool:
        """Ověří, zda daný token odpovídá aktivnímu tokenu."""
        with self._lock:
            return self._active_token == token

    # --------------- API – jednoduché příkazy ---------------
    def ping(self) -> str:
        return "PONG DRIVE"

    def motors_on(self) -> bool:
        # V tomto protokolu ON nedělá nic specielního, vracíme OK.
        return True

    def motors_off(self) -> bool:
        if not self.is_running():
            return "NOT RUNNING"
        # Místo speciálního OFF pošleme stop, aby se zastavily motory.
        return self._send_cmd("stop")

    def power_off(self) -> bool:
        # Power off hoverboardu nedává smysl, odpovíme True.
        return True

    def halt(self) -> bool:
        if not self.is_running():
            self.start()
        return self._send_cmd("stop")

    def brake(self) -> bool:
        if not self.is_running():
            return "NOT RUNNING"
        return self._send_cmd("stop")

    def drive(self, max_pwm: int, left_speed: int, right_speed: int) -> bool:
        if not self.is_running():
            return "NOT RUNNING"
        # Klient posílá rychlost v cm/s (např. 100 = 1 m/s)
        # Lua skript očekává m/s
        vL_ms = left_speed / 100.0
        vR_ms = right_speed / 100.0
        return self._send_cmd(f"speed {vL_ms} {vR_ms}")

    def pwm(self, left_pwm: int, right_pwm: int) -> bool:
        if not self.is_running():
            return "NOT RUNNING"
        return self._send_cmd(f"rpm {left_pwm} {right_pwm}")

    # --------------- low‑level TX ---------------
    def _send_cmd(self, cmd_str: str) -> bool:
        if not self._tx_mutex.acquire(timeout=0.3):
            raise RuntimeError("Timeout acquiring TX mutex")

        try:
            with self._lock:
                if not self._ser or not self._ser.is_open:
                    self._tx_fail += 1
                    raise RuntimeError("Serial port not open")

            cmd_bytes = (cmd_str + "\n").encode("ascii")
            print(f"[DRIVE-CRAWLER] UART TX: {cmd_str}")
            self._ser.write(cmd_bytes)
            self._ser.flush()

            # Přečteme řádku(y) z Lua skriptu. Hledáme ACK nebo PONG (pro ping, i když ten tady přes UART posílat nebudeme)
            while True:
                resp = self._ser.readline()
                if not resp:
                    # Timeout
                    with self._lock:
                        self._tx_fail += 1
                    print(f"[DRIVE-CRAWLER] UART RX: <TIMEOUT>")
                    raise TimeoutError("No ACK received from Lua script")
                
                resp_str = resp.decode("ascii", errors="replace").strip()
                if not resp_str:
                    continue  # Přeskoč prázdné řádky
                
                print(f"[DRIVE-CRAWLER] UART RX: {resp_str}")
                if "ACK" in resp_str or "PONG" in resp_str:
                    with self._lock:
                        self._last_cmd_at = time.monotonic()
                        self._tx_ok += 1
                    return True
                elif "NACK" in resp_str:
                    with self._lock:
                        self._tx_fail += 1
                    raise RuntimeError(f"Received NACK from Lua script: {resp_str}")
                # Jinak ignorujeme jiný text (např. starší data, i když buffer by měl být čistý)

        finally:
            self._tx_mutex.release()

    # --------------- stav/diagnostika ---------------
    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            running = self._running
            started_at = self._started_at
            last_cmd_at = self._last_cmd_at
            tx_ok = self._tx_ok
            tx_fail = self._tx_fail

        return {
            "service": "DRIVE-CRAWLER",
            "status": "RUNNING" if running else "STOPPED",
            "started_at_mono": started_at,
            "last_cmd_at_mono": last_cmd_at,
            "tx_ok": tx_ok,
            "tx_fail": tx_fail,
            "serial": {
                "device": self.cfg.device,
                "baud": self.cfg.baudrate,
            }
        }
