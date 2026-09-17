# mission-robotour/microservices.py
from __future__ import annotations
import socket
from typing import Dict, List, Tuple, Any, Optional

DEFAULT_TIMEOUT = 2.0

# Seznam 13 vyžadovaných mikroslužeb: (Klíč, Port, Seznam akceptovaných PONG odpovědí)
MICROSERVICES_CONFIG: Dict[str, Dict[str, Any]] = {
    "QRSCANER": {
        "port": 9021,
        "expected_pong": ["PONG QRSCANER"]
    },
    "TERMINAL": {
        "port": 9022,
        "expected_pong": ["PONG TERMINAL"]
    },
    "DRIVE": {
        "port": 9003,
        "expected_pong": ["PONG DRIVE"]
    },
    "GNSS-DUAL": {
        "port": 9006,
        "expected_pong": ["PONG GNSS-DUAL", "PONG GNSS_DUAL"]
    },
    "GNSS-GPS": {
        "port": 9004,
        "expected_pong": ["PONG GNSS-GPS"]
    },
    "RTK": {
        "port": 9015,
        "expected_pong": ["PONG GNSS-RTK", "PONG RTK"]
    },
    "GNSS-IMU": {
        "port": 9016,
        "expected_pong": ["PONG GNSS-IMU"]
    },
    "LOGGER": {
        "port": 9012,
        "expected_pong": ["PONG LOGGER"]
    },
    "FUSION": {
        "port": 9009,
        "expected_pong": ["PONG FUSION"]
    },
    "MAPS": {
        "port": 9040,
        "expected_pong": ["PONG MAPS"]
    },
    "LIDAR": {
        "port": 9002,
        "expected_pong": ["PONG LIDAR"]
    },
    "OOW-BRIDGE": {
        "port": 9030,
        "expected_pong": ["PONG OOW", "PONG OOW-BRIDGE"]
    },
    "PILOT-ROBOTOUR": {
        "port": 9104,
        "expected_pong": ["PONG PILOT_ROBOTOUR"]
    }
}


def send_tcp_command(host: str, port: int, cmd: str, timeout: float = DEFAULT_TIMEOUT) -> Tuple[bool, str]:
    """
    Odešle TCP příkaz zakončený \n a vrátí (success, response_line).
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        s.sendall((cmd.strip() + "\n").encode("utf-8"))
        f = s.makefile("rwb", buffering=0)
        line = f.readline().decode("utf-8").strip()
        return True, line
    except Exception as e:
        return False, str(e)
    finally:
        try:
            s.close()
        except Exception:
            pass


def ping_service(name: str, host: str = "127.0.0.1", port: Optional[int] = None, timeout: float = DEFAULT_TIMEOUT) -> Tuple[bool, str]:
    """
    Ověří dostupnost a správný název služby pomocí PING -> PONG <NAZEV>.
    """
    cfg = MICROSERVICES_CONFIG.get(name)
    target_port = port if port is not None else (cfg["port"] if cfg else None)
    if target_port is None:
        return False, f"Neznámá služba {name}"

    expected = cfg["expected_pong"] if cfg else [f"PONG {name}"]

    ok, resp = send_tcp_command(host, target_port, "PING", timeout=timeout)
    if not ok:
        return False, f"Nedostupná (port {target_port}): {resp}"

    if resp not in expected:
        return False, f"Chybný název na portu {target_port}: očekáváno {expected}, obdrženo '{resp}'"

    return True, resp


def check_all_services(host: str = "127.0.0.1", timeout: float = DEFAULT_TIMEOUT) -> Tuple[bool, Dict[str, str]]:
    """
    Krok 0: Zkontroluje všech 13 mikroslužeb.
    Vrátí (all_ok, chyby).
    """
    errors = {}
    for name, cfg in MICROSERVICES_CONFIG.items():
        ok, msg = ping_service(name, host=host, port=cfg["port"], timeout=timeout)
        if not ok:
            errors[name] = msg
    return len(errors) == 0, errors
