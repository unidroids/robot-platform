import threading
import time
import socket
import os
import math
from datetime import datetime
from typing import Optional, Tuple, List

from services import send_command
from util import log_event, parse_lidar_distance

from data.nav_fusion_data import NavFusionData
from data.waypoints_data import WayPointsData, Waypoint

# Používané služby pro MANUAL:
PORT_GAMEPAD = 9005
PORT_LIDAR   = 9002
PORT_DRIVE   = 9003
PORT_GNSS    = 9006
PORT_PPOINT  = 9007  # PointPerfect

# Journey output:
JOURNEY_DIR = "/data/robot/journey"

# Parametry fixace bodu:
GNSS_POLL_HZ = 1.0
GNSS_HACC_OK_M = 0.05          # 50 mm
GNSS_STABLE_TOL_M = 0.03       # ±30 mm (max vzdálenost mezi 3 body <= 0.03 m)
GNSS_WAIT_TIMEOUT_S = 180.0    # 3 minuty

# Ochrana proti duplicitnímu zápisu:
WAYPOINT_MIN_DIST_M = 1.0

manual_running = threading.Event()
_stop_requested = threading.Event()

# Interní reference na klienta, který startoval workflow – výsledky mu zkusíme posílat.
_client_conn_lock = threading.Lock()
_client_conn: Optional[socket.socket] = None

# Waypoints recorder state
_waypoints_lock = threading.Lock()
_waypoints_data: Optional[WayPointsData] = None
_waypoints_path: Optional[str] = None


def _safe_send_to_client(text: str) -> None:
    """Nepřeruší workflow ani při odpojeném klientovi."""
    with _client_conn_lock:
        conn = _client_conn
    if not conn:
        return
    try:
        conn.sendall(text.encode())
    except Exception:
        pass


def _send_and_report(port: int, cmd: str, expect_response: bool = True) -> str:
    """
    Pošle příkaz službě a vrátí odpověď.
    Do klienta pošle jednu řádku pro dohledatelnost (neblokující).
    """
    resp = send_command(port, cmd, expect_response=expect_response)
    _safe_send_to_client(f"SERVICE[{port}] {cmd} -> {resp}\n")
    return resp


def _atomic_write_text(path: str, text: str) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _waypoints_init_new_file() -> None:
    """
    Při startu MANUAL vytvoří nový:
      /data/robot/journey/waypoints-YYYY-mm-dd-HH-MM-SS.json
    """
    os.makedirs(JOURNEY_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    path = os.path.join(JOURNEY_DIR, f"waypoints-{ts}.json")

    data = WayPointsData(waypoints=[])
    _atomic_write_text(path, data.to_json(indent=2))

    with _waypoints_lock:
        global _waypoints_data, _waypoints_path
        _waypoints_data = data
        _waypoints_path = path

    log_event(f"MANUAL waypoints file created: {path}")
    _safe_send_to_client(f"WAYPOINTS_FILE {path}\n")


def _enu_from_latlon_m(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """
    Lokální aproximace EN (East, North) v metrech.
    R = 6378137 (WGS84 semi-major axis).
    """
    R = 6378137.0
    lat0_rad = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * R
    east = dlon * R * math.cos(lat0_rad)
    return east, north


def _latlon_from_enu_m(east: float, north: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Inverze k _enu_from_latlon_m()."""
    R = 6378137.0
    lat0_rad = math.radians(lat0)
    lat = lat0 + math.degrees(north / R)
    denom = (R * math.cos(lat0_rad))
    if denom == 0.0:
        lon = lon0
    else:
        lon = lon0 + math.degrees(east / denom)
    return lat, lon


def _max_pairwise_dist_m(points_enu: List[Tuple[float, float]]) -> float:
    """Maximum vzdálenosti mezi všemi dvojicemi (v ENU 2D)."""
    max_d = 0.0
    for i in range(len(points_enu)):
        ex, nx = points_enu[i]
        for j in range(i + 1, len(points_enu)):
            ey, ny = points_enu[j]
            d = math.hypot(ex - ey, nx - ny)
            if d > max_d:
                max_d = d
    return max_d


def _distance_m_enu(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """2D vzdálenost v metrech přes lokální ENU aproximaci."""
    e, n = _enu_from_latlon_m(lat2, lon2, lat1, lon1)
    return math.hypot(e, n)


def _parse_buttons(resp: str) -> Optional[List[int]]:
    """Očekává: "<b0> <b1> <b2> <b3> <b4>" kde bX je 0/1"""
    parts = resp.strip().split()
    if len(parts) < 5:
        return None
    try:
        vals = [int(parts[i]) for i in range(5)]
    except ValueError:
        return None
    for v in vals:
        if v not in (0, 1):
            return None
    return vals


def _loop_until_distance_valid() -> Tuple[Optional[int], Optional[float]]:
    """Opakovaně čte LIDAR:DISTANCE dokud nevrátí validní vzdálenost (ne -1) nebo STOP."""
    while not _stop_requested.is_set():
        resp = _send_and_report(PORT_LIDAR, "DISTANCE")
        idx, dist = parse_lidar_distance(resp)
        if idx is not None and idx != -1:
            return idx, dist
        time.sleep(0.05)
    return None, None


def _append_waypoint(lat: float, lon: float) -> bool:
    """
    Přidá waypoint do interní struktury a uloží JSON.
    Vrací True pokud byl waypoint uložen, jinak False.
    """
    with _waypoints_lock:
        global _waypoints_data, _waypoints_path
        if _waypoints_data is None or _waypoints_path is None:
            log_event("WAYPOINT SKIP: recorder not initialized")
            _safe_send_to_client("WAYPOINT SKIP recorder-not-initialized\n")
            return False

        if _waypoints_data.waypoints:
            last = _waypoints_data.waypoints[-1]
            d = _distance_m_enu(last.lat, last.lon, lat, lon)
            if d < WAYPOINT_MIN_DIST_M:
                log_event(f"WAYPOINT SKIP: too close to last ({d:.3f} m)")
                _safe_send_to_client(f"WAYPOINT SKIP too-close {d:.3f}m\n")
                return False

        wp = Waypoint(
            lat=float(lat),
            lon=float(lon),
            curvature=0.0,
            path_width_m=1.0,
            rel_azimuth_deg=0.0,
            corridors=[],
        )
        _waypoints_data.waypoints.append(wp)
        _atomic_write_text(_waypoints_path, _waypoints_data.to_json(indent=2))

        log_event(f"WAYPOINT ADD: lat={lat:.9f} lon={lon:.9f} count={len(_waypoints_data.waypoints)}")
        _safe_send_to_client(f"WAYPOINT ADD lat={lat:.9f} lon={lon:.9f} count={len(_waypoints_data.waypoints)}\n")
        return True


def _capture_waypoint_with_gnss() -> None:
    """
    Režim fixace bodu po stisku tlačítka:
    - DRIVE BREAK
    - pull GNSS:DATA 1 Hz, čekat max 3 minuty
    - podmínka: hAcc <= 50mm
    - stabilita: 3 po sobě jdoucí vzorky, max vzdálenost mezi nimi <= 30mm
    - uložit průměr (v ENU) jako waypoint
    """
    _send_and_report(PORT_DRIVE, "BREAK")

    t0 = time.monotonic()
    log_event("WAYPOINT WAIT START (button press)")
    _safe_send_to_client("WAYPOINT WAIT START\n")

    stable_samples: List[NavFusionData] = []

    while not _stop_requested.is_set():
        elapsed = time.monotonic() - t0
        if elapsed > GNSS_WAIT_TIMEOUT_S:
            log_event("WAYPOINT WAIT TIMEOUT (3 min)")
            _safe_send_to_client("WAYPOINT WAIT TIMEOUT\n")
            return

        resp = _send_and_report(PORT_GNSS, "DATA")
        try:
            data = NavFusionData.from_json(resp)
        except Exception as e:
            log_event(f"WAYPOINT WAIT GNSS PARSE ERROR: {e}")
            _safe_send_to_client(f"WAYPOINT WAIT GNSS PARSE ERROR: {e}\n")
            stable_samples.clear()
            time.sleep(1.0 / GNSS_POLL_HZ)
            continue

        log_event(
            f"WAYPOINT WAIT GNSS: lat={data.lat:.9f} lon={data.lon:.9f} "
            f"hAcc={data.hAcc:.4f}m gnssFixOK={int(bool(data.gnssFixOK))} drUsed={int(bool(data.drUsed))}"
        )

        if data.hAcc <= GNSS_HACC_OK_M:
            stable_samples.append(data)
            if len(stable_samples) > 3:
                stable_samples.pop(0)

            if len(stable_samples) == 3:
                lat0 = stable_samples[0].lat
                lon0 = stable_samples[0].lon
                pts = [_enu_from_latlon_m(s.lat, s.lon, lat0, lon0) for s in stable_samples]
                max_d = _max_pairwise_dist_m(pts)

                log_event(f"WAYPOINT WAIT STABILITY: max_pairwise={max_d:.4f}m (tol={GNSS_STABLE_TOL_M:.4f}m)")
                _safe_send_to_client(f"WAYPOINT WAIT STABILITY max_pairwise={max_d:.4f}m\n")

                if max_d <= GNSS_STABLE_TOL_M:
                    mean_e = sum(p[0] for p in pts) / 3.0
                    mean_n = sum(p[1] for p in pts) / 3.0
                    lat_avg, lon_avg = _latlon_from_enu_m(mean_e, mean_n, lat0, lon0)

                    log_event(f"WAYPOINT WAIT SUCCESS: avg lat={lat_avg:.9f} lon={lon_avg:.9f}")
                    _safe_send_to_client(f"WAYPOINT WAIT SUCCESS lat={lat_avg:.9f} lon={lon_avg:.9f}\n")

                    saved = _append_waypoint(lat_avg, lon_avg)
                    log_event("WAYPOINT WAIT END: " + ("SAVED" if saved else "NOT SAVED"))
                    _safe_send_to_client("WAYPOINT WAIT END " + ("SAVED\n" if saved else "NOT-SAVED\n"))
                    return
        else:
            stable_samples.clear()

        # udržuj robot zastavený i během čekání
        _send_and_report(PORT_DRIVE, "BREAK")
        time.sleep(1.0 / GNSS_POLL_HZ)


def _control_loop() -> None:
    """
    Hlavní smyčka MANUAL:
    - řízení: LIDAR bezpečnost + GAMEPAD DATA -> DRIVE
    - trigger: GAMEPAD BUTTONS b0 edge -> fixace bodu přes GNSS -> uložení -> návrat do řízení
    """
    last_b0 = 0

    while not _stop_requested.is_set():
        resp = _send_and_report(PORT_LIDAR, "DISTANCE")
        _, dist = parse_lidar_distance(resp)

        if dist is not None and dist < 50.0:
            _send_and_report(PORT_DRIVE, "BREAK")
        else:
            data = _send_and_report(PORT_GAMEPAD, "DATA")
            pwm = data.split('#', 1)[0].rstrip()
            _send_and_report(PORT_DRIVE, pwm)

        btns_resp = _send_and_report(PORT_GAMEPAD, "BUTTONS")
        btns = _parse_buttons(btns_resp)
        if btns is not None:
            b0 = btns[0]
            if last_b0 == 0 and b0 == 1:
                last_b0 = 1
                _capture_waypoint_with_gnss()
            else:
                last_b0 = b0

        time.sleep(0.03)


def _manual_workflow():
    try:
        log_event("MANUAL workflow: START")
        _safe_send_to_client("WORKFLOW MANUAL START\n")

        _waypoints_init_new_file()

        _send_and_report(PORT_GAMEPAD, "PING")
        _send_and_report(PORT_GAMEPAD, "START")

        _send_and_report(PORT_LIDAR, "PING")
        _send_and_report(PORT_LIDAR, "START")

        _send_and_report(PORT_DRIVE, "PING")
        _send_and_report(PORT_DRIVE, "START")
        _send_and_report(PORT_DRIVE, "ON")

        _send_and_report(PORT_GNSS, "PING")
        _send_and_report(PORT_GNSS, "START")

        _send_and_report(PORT_PPOINT, "PING")
        _send_and_report(PORT_PPOINT, "START")

        _loop_until_distance_valid()
        _control_loop()

    except Exception as e:
        log_event(f"WORKFLOW ERROR: {e}")
        _safe_send_to_client(f"WORKFLOW ERROR: {e}\n")

    finally:
        try:
            _send_and_report(PORT_DRIVE, "BREAK")
            _send_and_report(PORT_DRIVE, "OFF")
            _send_and_report(PORT_GAMEPAD, "STOP")
            _send_and_report(PORT_DRIVE,  "STOP")
            _send_and_report(PORT_LIDAR,  "STOP")
            _send_and_report(PORT_PPOINT, "STOP")
            _send_and_report(PORT_GNSS,   "STOP")
        except Exception as e:
            log_event(f"MANUAL stop cleanup error: {e}")

        manual_running.clear()
        _stop_requested.clear()
        _safe_send_to_client("WORKFLOW MANUAL END\n")
        log_event("MANUAL workflow: END")


def start_manual_workflow(client_conn: Optional[socket.socket]) -> None:
    """Spustí MANUAL workflow (pokud už neběží)."""
    if manual_running.is_set():
        raise RuntimeError("MANUAL already running")
    manual_running.set()
    _stop_requested.clear()
    with _client_conn_lock:
        global _client_conn
        _client_conn = client_conn
    t = threading.Thread(target=_manual_workflow, daemon=True)
    t.start()


def stop_manual_workflow() -> None:
    """Požádá o STOP a rovnou pošle STOP do všech relevantních služeb (idempotentní)."""
    _stop_requested.set()
    try:
        _send_and_report(PORT_GAMEPAD, "STOP")
        _send_and_report(PORT_DRIVE,  "STOP")
        _send_and_report(PORT_LIDAR,  "STOP")
        _send_and_report(PORT_GNSS,   "STOP")
        _send_and_report(PORT_PPOINT, "STOP")
    except Exception:
        pass
