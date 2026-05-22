import threading
import time
import socket
import json
import re

import traceback
from pathlib import Path
from typing import Optional

from services import send_command
from util import log_event, parse_lidar_distance
from data.waypoints_data import WayPointsData, Waypoint

# Používané služby:
PORT_LIDAR   = 9002
PORT_DRIVE   = 9003
PORT_PILOT   = 9008
PORT_GNSS    = 9006
PORT_PPOINT  = 9007
PORT_FUSION  = 9009 
PORT_HEADING = 9010

auto_running = threading.Event()
_stop_requested = threading.Event()

_client_conn_lock = threading.Lock()
_client_conn: Optional[socket.socket] = None


def _safe_send_to_client(text: str) -> None:
    with _client_conn_lock:
        conn = _client_conn
    if not conn:
        return
    try:
        conn.sendall(text.encode())
    except Exception:
        pass


def _send_and_report(port: int, cmd: str, expect_response: bool = True) -> str:
    resp = send_command(port, cmd, expect_response=expect_response)
    line = f"SERVICE[{port}] {cmd} -> {resp}"
    _safe_send_to_client(line + "\n")
    return resp


def _read_route() -> WayPointsData:
    p = Path(__file__).parent / "waypoints" / "_route.json"
    txt = p.read_text(encoding="utf-8")
    return WayPointsData.from_json(txt)


def _hacc_mm_from_gnss_data(s: str) -> Optional[float]:
    if not s:
        return None
    try:
        start = s.find("{")
        end = s.rfind("}")
        payload = s[start:end + 1] if (start != -1 and end != -1 and end > start) else s
        obj = json.loads(payload)
        def find_hacc(o) -> Optional[float]:
            if isinstance(o, dict):
                if "hAcc" in o and isinstance(o["hAcc"], (int, float)):
                    return float(o["hAcc"])
                for v in o.values():
                    r = find_hacc(v)
                    if r is not None:
                        return r
            elif isinstance(o, list):
                for v in o:
                    r = find_hacc(v)
                    if r is not None:
                        return r
            return None
        val = find_hacc(obj)
        return float(val) if val is not None else None
    except Exception:
        m = re.search(r'"?hAcc"?\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)', s)
        return float(m.group(1)) if m else None


def _distance_cm() -> Optional[float]:
    resp = _send_and_report(PORT_LIDAR, "DISTANCE")
    idx, dist = parse_lidar_distance(resp)
    if idx is None or idx == -1:
        return None
    return dist


def _gnss_hacc_mm() -> Optional[float]:
    resp = _send_and_report(PORT_FUSION, "DATA")
    return _hacc_mm_from_gnss_data(resp)


def _pilot_status() -> str:
    resp = _send_and_report(PORT_PILOT, "STATUS")
    return resp.strip().upper()


def _safe_to_go(hacc_mm: Optional[float], dist_cm: Optional[float]) -> bool:
    return (hacc_mm is not None and hacc_mm < 500.0) and (dist_cm is not None and dist_cm > 50.0)


def _unsafe(hacc_mm: Optional[float], dist_cm: Optional[float]) -> bool:
    if hacc_mm is not None and hacc_mm > 800.0:
        return True
    if dist_cm is not None and dist_cm < 40.0:
        return True
    return False


def _await_gnss_ok(threshold_mm: float = 300.0, timeout_s: float = 120.0, poll_s: float = 0.5) -> bool:
    t0 = time.time()
    while not _stop_requested.is_set() and (time.time() - t0) < timeout_s:
        hacc = _gnss_hacc_mm()
        if hacc is not None:
            _safe_send_to_client(f"GNSS hAcc: {hacc:.0f} mm\n")
            if hacc < threshold_mm:
                return True
        time.sleep(poll_s)
    return False


def _await_lidar_ok(timeout_s: float = 30.0, poll_s: float = 0.2) -> bool:
    t0 = time.time()
    while not _stop_requested.is_set() and (time.time() - t0) < timeout_s:
        d = _distance_cm()
        if d is not None:
            _safe_send_to_client(f"LIDAR DISTANCE: {d:.0f} cm\n")
            return True
        time.sleep(poll_s)
    return False


def _auto_workflow():
    try:
        log_event("AUTO workflow: START")
        _safe_send_to_client("WORKFLOW AUTO START\n")

        # --- Skupina 1: GNSS + PointPerfect + DRIVE ---
        _send_and_report(PORT_FUSION, "PING")
        _send_and_report(PORT_PPOINT, "PING")
        _send_and_report(PORT_DRIVE,  "PING")
        _send_and_report(PORT_HEADING,"PING")

        _send_and_report(PORT_FUSION, "RESTART")
        _send_and_report(PORT_HEADING,"START")
        _send_and_report(PORT_GNSS,   "START")
        _send_and_report(PORT_PPOINT, "START")
        _send_and_report(PORT_DRIVE,  "START")

        _safe_send_to_client("WAIT GNSS(hAcc<300mm)...\n")
        if not _await_gnss_ok(threshold_mm=300.0, timeout_s=120.0, poll_s=0.5):
            _safe_send_to_client("ERROR: GNSS not ready (hAcc >= 300mm) within timeout.\n")
            return

        # --- Skupina 2: PILOT + LIDAR (až když je GNSS OK) ---
        _send_and_report(PORT_PILOT,  "PING")
        _send_and_report(PORT_LIDAR,  "PING")

        _send_and_report(PORT_PILOT,  "START")
        _send_and_report(PORT_LIDAR,  "START")

        _safe_send_to_client("VALIDATE GNSS & LIDAR...\n")
        gnss_still_ok = _await_gnss_ok(threshold_mm=300.0, timeout_s=60.0, poll_s=1.0)
        lidar_ok      = _await_lidar_ok(timeout_s=60.0, poll_s=1.0)
        if not (gnss_still_ok and lidar_ok):
            if not gnss_still_ok:
                _safe_send_to_client("ERROR: GNSS lost accuracy during validation.\n")
            if not lidar_ok:
                _safe_send_to_client("ERROR: LIDAR not providing DISTANCE.\n")
            return

        # -------------- Hlavní smyčka ----------------
        last_gnss_ts   = 0.0
        last_status_ts = 0.0

        hacc_mm: Optional[float] = None
        dist_cm: Optional[float] = None
        last_status = ""

        route_sent = False
        brake_phase = False

        while not _stop_requested.is_set():
            now = time.time()

            dist_cm = _distance_cm()

            if now - last_gnss_ts >= 1.0:
                hacc_mm = _gnss_hacc_mm()
                last_gnss_ts = now

            if now - last_status_ts >= 1.0:
                try:
                    st = _pilot_status()
                    if st != last_status:
                        _safe_send_to_client(f"PILOT STATUS: {st}\n")
                        last_status = st
                    if st == "GOAL_REACHED":
                        log_event("AUTO workflow: REACHED – končím.")
                        break
                    elif st == "GOAL_NOT_REACHED":
                        log_event("AUTO workflow: GOAL_NOT_REACHED – končím.")
                        break
                except Exception:
                    pass
                last_status_ts = now

            if _unsafe(hacc_mm, dist_cm):
                if not brake_phase:
                    _safe_send_to_client("STATE: UNSAFE -> HALT\n")
                    _send_and_report(PORT_DRIVE, "HALT", expect_response=True)
                    brake_phase = True
                time.sleep(0.10)
                continue

            if brake_phase and not _safe_to_go(hacc_mm, dist_cm):
                time.sleep(0.10)
                continue

            if _safe_to_go(hacc_mm, dist_cm):
                if brake_phase:
                    _safe_send_to_client("STATE: SAFE -> BREAK\n")
                    _send_and_report(PORT_DRIVE, "BREAK", expect_response=True)
                    brake_phase = False

                if not route_sent:
                    start_lat = None
                    start_lon = None
                    resp = _send_and_report(PORT_FUSION, "DATA")
                    try:
                        start_payload = json.loads(resp[resp.find("{"):resp.rfind("}")+1])
                        start_lat = float(start_payload.get("lat", 0.0))
                        start_lon = float(start_payload.get("lon", 0.0))
                    except Exception:
                        _safe_send_to_client("ERROR: Nelze načíst aktuální GNSS pozici pro WAYPOINTS.\n")
                        continue

                    try:
                        route = _read_route()
                        start_wp = Waypoint(
                            lat=start_lat,
                            lon=start_lon,
                            curvature=0.0,
                            path_width_m=1.0,
                            rel_azimuth_deg=0.0,
                            corridors=[]
                        )
                        route.waypoints.insert(0, start_wp)
                        
                        cmd = f"WAYPOINTS {route.to_json()}"
                        _send_and_report(PORT_PILOT, cmd)
                        route_sent = True
                        brake_phase = False
                        _safe_send_to_client("SENT: WAYPOINTS <route_json>\n")
                    except Exception as ex:
                        _safe_send_to_client(f"ERROR: Selhalo načtení nebo odeslání _route.json: {ex}\n")

            time.sleep(0.10)

        _safe_send_to_client("WORKFLOW AUTO END\n")

    except Exception as e:
        log_event(f"AUTO WORKFLOW ERROR: {e}")
        _safe_send_to_client(f"WORKFLOW ERROR: {e}\n")
        #trace
        log_event(f"AUTO WORKFLOW ERROR: {traceback.format_exc()}")
        _safe_send_to_client(f"WORKFLOW ERROR: {traceback.format_exc()}\n")

    except KeyboardInterrupt:
        log_event("AUTO stop requested")
        _safe_send_to_client("WORKFLOW AUTO STOP\n  }")

    finally:
        try:
            _send_and_report(PORT_PILOT,  "STOP")
            _send_and_report(PORT_DRIVE,  "STOP")
            _send_and_report(PORT_LIDAR,  "STOP")
            _send_and_report(PORT_PPOINT, "STOP")
            _send_and_report(PORT_GNSS,   "STOP")
            _send_and_report(PORT_HEADING,"STOP")
            _send_and_report(PORT_FUSION, "RESTART")
        except Exception as e:
            log_event(f"AUTO stop cleanup error: {e}")

        auto_running.clear()
        _stop_requested.clear()
        log_event("AUTO workflow: END")


def start_auto_workflow(client_conn: Optional[socket.socket]) -> None:
    if auto_running.is_set():
        raise RuntimeError("AUTO already running")
    auto_running.set()
    _stop_requested.clear()
    with _client_conn_lock:
        global _client_conn
        _client_conn = client_conn
    t = threading.Thread(target=_auto_workflow, daemon=True)
    t.start()


def stop_auto_workflow() -> None:
    _stop_requested.set()
    try:
        _send_and_report(PORT_PILOT,  "STOP")
        _send_and_report(PORT_DRIVE,  "STOP")
        _send_and_report(PORT_LIDAR,  "STOP")
        _send_and_report(PORT_PPOINT, "STOP")
        _send_and_report(PORT_GNSS,   "STOP")
        _send_and_report(PORT_HEADING,"STOP")
        _send_and_report(PORT_FUSION, "RESTART")
    except Exception:
        pass
