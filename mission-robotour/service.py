# mission-robotour/service.py
from __future__ import annotations
import asyncio
import json
import math
import os
import threading
import time
from typing import Optional, Dict, Any, List, Tuple

try:
    import zmq
except ImportError:
    zmq = None

try:
    from .data_logger import MissionDataLogger
    from .microservices import (
        send_tcp_command,
        ping_service,
        check_all_services,
        MICROSERVICES_CONFIG
    )
    from .terminal_client import TerminalClient
except (ImportError, ValueError):
    from data_logger import MissionDataLogger
    from microservices import (
        send_tcp_command,
        ping_service,
        check_all_services,
        MICROSERVICES_CONFIG
    )
    from terminal_client import TerminalClient


def calculate_geodesic_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Vypočte přibližnou vzdálenost mezi dvěma GPS souřadnicemi v metrech (WGS84 aproximace).
    """
    lat_mid = math.radians((lat1 + lat2) / 2.0)
    m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_mid) + 1.175 * math.cos(4 * lat_mid)
    m_per_deg_lon = 111412.84 * math.cos(lat_mid) - 93.5 * math.cos(3 * lat_mid)
    dx = (lon2 - lon1) * m_per_deg_lon
    dy = (lat2 - lat1) * m_per_deg_lat
    return math.hypot(dx, dy)


class MissionRobotourService:
    """
    Hlavní stavový automat mise Robotour 2025/2026.
    Řídí kroky 0 až 23 podle specifikace.
    """

    def __init__(self, host: str = "127.0.0.1"):
        self.host = host
        self.running = False
        self.current_step = 0
        self.state_name = "IDLE"

        self.logger = MissionDataLogger()
        self.terminal = TerminalClient(host=self.host, port=9022)

        self._stop_requested = False
        self._loop_task: Optional[asyncio.Task] = None
        self._state_lock = threading.Lock()

        # Poslední přijatá tlačítka a QR kódy
        self._last_button: Optional[str] = None
        self._button_event = asyncio.Event()

        self._last_qr_code: Optional[str] = None
        self._qr_event = asyncio.Event()

        # Běžící data mise
        self.target_lat: Optional[float] = None
        self.target_lon: Optional[float] = None
        self.start_lat: Optional[float] = None
        self.start_lon: Optional[float] = None
        self.route_json_str: Optional[str] = None
        self.pilot_status: Dict[str, Any] = {}

        # ZMQ odběr
        self._zmq_thread: Optional[threading.Thread] = None

    def get_status_dict(self) -> Dict[str, Any]:
        """Vrátí aktuální stav pro příkaz STATUS."""
        with self._state_lock:
            return {
                "service": "MISSION-ROBOTOUR",
                "running": self.running,
                "step": self.current_step,
                "state": self.state_name,
                "target_lat": self.target_lat,
                "target_lon": self.target_lon,
                "pilot_status": self.pilot_status
            }

    def start_mission(self) -> Tuple[bool, str]:
        """Příkaz START: zahájí stavový automat mise."""
        if self.running:
            return False, "ALREADY_RUNNING"
        self.running = True
        self._stop_requested = False
        self._start_zmq_subscriber()
        self._loop_task = asyncio.create_task(self._run_mission_workflow())
        return True, "OK"

    def stop_mission(self) -> Tuple[bool, str]:
        """Příkaz STOP: ukončí probíhající misi a zastaví spuštěné mikroslužby."""
        if not self.running:
            return False, "WAS_NOT_RUNNING"
        self._stop_requested = True
        self.running = False
        if self._loop_task and not self._loop_task.done():
            self._loop_task.cancel()

        # Konec mise: zastavíme všechny služby
        self._stop_all_services()
        self.logger.close()
        self.state_name = "STOPPED"
        self.current_step = 0
        return True, "OK"

    def shutdown(self):
        """Ukončení celého procesu."""
        self.stop_mission()

    # =========================================================================
    # ZMQ Listener (tlačítka z terminálu & QR kód)
    # =========================================================================

    def _start_zmq_subscriber(self):
        if zmq is None or self._zmq_thread is not None:
            return

        def zmq_worker():
            ctx = zmq.Context()
            sub_term = ctx.socket(zmq.SUB)
            sub_qr = ctx.socket(zmq.SUB)

            try:
                sub_term.connect("ipc:///tmp/robot-terminal")
                sub_term.setsockopt_string(zmq.SUBSCRIBE, "")
            except Exception as e:
                print(f"[MissionService] ZMQ connect robot-terminal warning: {e}")

            try:
                sub_qr.connect("ipc:///tmp/robot-qrscaner")
                sub_qr.setsockopt_string(zmq.SUBSCRIBE, "")
            except Exception as e:
                print(f"[MissionService] ZMQ connect robot-qrscaner warning: {e}")

            poller = zmq.Poller()
            poller.register(sub_term, zmq.POLLIN)
            poller.register(sub_qr, zmq.POLLIN)

            while self.running:
                try:
                    socks = dict(poller.poll(500))
                    if sub_term in socks:
                        msg = sub_term.recv_multipart()
                        if len(msg) >= 2:
                            frame_type = msg[0].decode("utf-8", errors="ignore")
                            frame_val = msg[1].decode("utf-8", errors="ignore")
                            if frame_type == "button":
                                self.on_button_pressed(frame_val)

                    if sub_qr in socks:
                        msg = sub_qr.recv_multipart()
                        if len(msg) >= 2:
                            frame_val = msg[1].decode("utf-8", errors="ignore")
                            self.on_qr_scanned(frame_val)
                except Exception as e:
                    if self.running:
                        time.sleep(0.1)

            sub_term.close()
            sub_qr.close()
            ctx.term()

        self._zmq_thread = threading.Thread(target=zmq_worker, daemon=True)
        self._zmq_thread.start()

    def on_button_pressed(self, btn_id: str):
        """Callback při kliknutí na tlačítko na terminálu."""
        if btn_id == "chcek_again":
            btn_id = "check_again"
        print(f"[MissionService] Stisknuto tlačítko: {btn_id}")
        self.logger.log("BUTTON_CLICKED", {"button": btn_id, "step": self.current_step})
        self._last_button = btn_id
        self._button_event.set()

    def on_qr_scanned(self, qr_text: str):
        """Callback při detekci QR kódu."""
        print(f"[MissionService] Načten QR kód: {qr_text}")
        self.logger.log("QR_DETECTED", {"raw": qr_text, "step": self.current_step})
        self._last_qr_code = qr_text
        self._qr_event.set()

    async def _wait_for_button(self, allowed_buttons: List[str], timeout: Optional[float] = None) -> Optional[str]:
        """Čeká na stisk povoleného tlačítka."""
        start_t = time.time()
        while self.running and not self._stop_requested:
            if self._last_button in allowed_buttons:
                btn = self._last_button
                self._last_button = None
                return btn

            self._button_event.clear()
            try:
                to = timeout - (time.time() - start_t) if timeout else 1.0
                if to <= 0:
                    break
                await asyncio.wait_for(self._button_event.wait(), timeout=min(to, 1.0))
            except asyncio.TimeoutError:
                pass

            if self._last_button in allowed_buttons:
                btn = self._last_button
                self._last_button = None
                return btn

            if timeout and (time.time() - start_t) >= timeout:
                break
        return None

    # =========================================================================
    # Hlavní Workflow (Kroky 0 až 23)
    # =========================================================================

    async def _run_mission_workflow(self):
        print("[MissionService] Spouštím workflow Robotour...")
        try:
            # -------------------------------------------------------------
            # Krok 0: Kontrola potřebných mikroslužeb (PING -> PONG <NAZEV>)
            # -------------------------------------------------------------
            while self.running and not self._stop_requested:
                self.current_step = 0
                self.state_name = "STEP_0_CHECK_SERVICES"
                self.logger.log("STEP_0_START")

                all_ok, errors = check_all_services(host=self.host, timeout=1.5)
                if all_ok:
                    print("[MissionService] Krok 0 OK: Všech 13 mikroslužeb odpovídá správným PONG.")
                    self.logger.log("STEP_0_SUCCESS")
                    break
                else:
                    err_desc = ", ".join([f"{k}: {v}" for k, v in errors.items()])
                    print(f"[MissionService] Krok 0 CHYBA: {err_desc}")
                    self.logger.log("STEP_0_FAILED", {"errors": errors})
                    self.terminal.show_message(
                        header="Chyba komunikace",
                        text=f"Nedostupné služby nebo chybný název: {err_desc}",
                        buttons=[{"id": "try_again", "text": "Zkusit znovu"}]
                    )
                    btn = await self._wait_for_button(["try_again"])
                    if btn != "try_again":
                        return

            # -------------------------------------------------------------
            # Krok 1: Start polohových a senzorických služeb
            # -------------------------------------------------------------
            services_to_start = ["LOGGER", "DRIVE", "GNSS-DUAL", "GNSS-GPS", "GNSS-IMU", "RTK", "FUSION"]
            while self.running and not self._stop_requested:
                self.current_step = 1
                self.state_name = "STEP_1_START_SERVICES"
                self.logger.log("STEP_1_START", {"services": services_to_start})

                failed_starts = []
                for s_name in services_to_start:
                    port = MICROSERVICES_CONFIG[s_name]["port"]
                    ok, resp = send_tcp_command(self.host, port, "START", timeout=5.0)
                    if not ok or not resp.startswith("OK"):
                        failed_starts.append(f"{s_name} ({resp})")

                if not failed_starts:
                    print("[MissionService] Krok 1 OK: Polohové služby nastartovány.")
                    self.logger.log("STEP_1_SUCCESS")
                    break
                else:
                    print(f"[MissionService] Krok 1 CHYBA: {failed_starts}")
                    self.logger.log("STEP_1_FAILED", {"failed": failed_starts})
                    self.terminal.show_message(
                        header="Chyba při startu služby",
                        text=f"Služby neodpověděly OK: {', '.join(failed_starts)}",
                        buttons=[{"id": "try_again", "text": "Zkusit znovu"}]
                    )
                    btn = await self._wait_for_button(["try_again"])
                    if btn != "try_again":
                        return

            # -------------------------------------------------------------
            # Smyčka mise (od kroku 2)
            # -------------------------------------------------------------
            while self.running and not self._stop_requested:
                await self._run_mission_cycle()

        except asyncio.CancelledError:
            print("[MissionService] Workflow mise byla zrušena.")
        except Exception as e:
            print(f"[MissionService] Chyba ve workflow: {e}")
            self.logger.log("WORKFLOW_EXCEPTION", {"error": str(e)})

    async def _run_mission_cycle(self):
        """Jednotlivý cyklus mise od kroku 2 (úvodní obrazovka) po cíl / zastavení."""
        # -------------------------------------------------------------
        # Krok 2: Úvodní dialog & otevření logu mise
        # -------------------------------------------------------------
        self.current_step = 2
        self.state_name = "STEP_2_INITIAL_SCREEN"
        self.logger.start_mission()
        self.logger.log("STEP_2_DISPLAY_INITIAL")

        self.terminal.show_message(
            header="Robotour",
            text="Jdeme na to!",
            buttons=[{"id": "scan_qrcode", "text": "Scan QR Code"}]
        )

        btn = await self._wait_for_button(["scan_qrcode"])
        if btn != "scan_qrcode":
            return

        # -------------------------------------------------------------
        # Krok 4 až 9: Skenování a ověření QR kódu
        # -------------------------------------------------------------
        while self.running and not self._stop_requested:
            self.current_step = 4
            self.state_name = "STEP_4_SCAN_QR"
            self.logger.log("STEP_4_QR_START")

            # Spuštění QR scanneru
            send_tcp_command(self.host, MICROSERVICES_CONFIG["QRSCANER"]["port"], "START", timeout=1.5)
            self._last_qr_code = None
            self._qr_event.clear()

            # Čekání až 120s na QR kód
            qr_text = None
            start_qr_t = time.time()
            while (time.time() - start_qr_t) < 120.0 and self.running and not self._stop_requested:
                if self._last_qr_code:
                    qr_text = self._last_qr_code
                    self._last_qr_code = None
                    break
                try:
                    await asyncio.wait_for(self._qr_event.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    pass
                if self._last_qr_code:
                    qr_text = self._last_qr_code
                    self._last_qr_code = None
                    break

            # Timeout nebo neplatný
            if not qr_text:
                # Krok 6: Timeout 120s
                send_tcp_command(self.host, MICROSERVICES_CONFIG["QRSCANER"]["port"], "STOP", timeout=1.5)
                self.logger.log("STEP_6_QR_TIMEOUT")
                self.terminal.show_message(
                    header="Robotour - Nenačteny cílové souřadnice",
                    text="Během posledních dvou minut nebyl zaznamenán QR Code.",
                    buttons=[{"id": "rescan_qrcode", "text": "Rescan QR Code"}]
                )
                btn = await self._wait_for_button(["rescan_qrcode"])
                if btn == "rescan_qrcode":
                    continue
                return

            # Krok 7 & 8: Ověření formátu geo:<lat>,<lon>
            cleaned_qr = qr_text.strip()
            if cleaned_qr.startswith("geo:"):
                raw_coords = cleaned_qr[4:]
            else:
                raw_coords = cleaned_qr

            parts = [p.strip() for p in raw_coords.split(",") if p.strip()]
            valid = False
            if len(parts) == 2:
                try:
                    self.target_lat = float(parts[0])
                    self.target_lon = float(parts[1])
                    valid = True
                except ValueError:
                    pass

            if not valid:
                print(f"[MissionService] Neplatný formát QR kódu: {qr_text}")
                self.logger.log("STEP_8_QR_INVALID", {"raw": qr_text})
                continue

            # Krok 9: Formát OK
            send_tcp_command(self.host, MICROSERVICES_CONFIG["QRSCANER"]["port"], "STOP", timeout=1.5)
            self.terminal.sound("notification")
            self.logger.log("STEP_9_QR_VALID", {"lat": self.target_lat, "lon": self.target_lon})
            break

        if not self.running or self._stop_requested:
            return

        # -------------------------------------------------------------
        # Krok 10 až 12: Ověření připravenosti GPS (FUSION DATA)
        # -------------------------------------------------------------
        self.current_step = 10
        self.state_name = "STEP_10_WAIT_FOR_GPS"
        fusion_port = MICROSERVICES_CONFIG["FUSION"]["port"]

        while self.running and not self._stop_requested:
            self.logger.log("STEP_10_FUSION_QUERY")
            ok, resp = send_tcp_command(self.host, fusion_port, "DATA", timeout=2.0)
            fusion_data = {}
            if ok and resp and resp.startswith("{"):
                try:
                    fusion_data = json.loads(resp)
                except Exception:
                    pass

            gps_sol = fusion_data.get("gpsSol", "NONE")
            h_acc_raw = fusion_data.get("hAcc", 99999)
            try:
                h_acc_mm = float(h_acc_raw)
            except (ValueError, TypeError):
                h_acc_mm = 99999.0

            h_acc_int = int(round(h_acc_mm))
            lat = fusion_data.get("lat", 0.0)
            lon = fusion_data.get("lon", 0.0)

            # Podmínka: gpsSol platné a hAcc <= 10000 mm (10 m)
            if gps_sol and str(gps_sol).upper() not in ["NONE", "NULL"] and h_acc_mm <= 10000.0 and lat != 0.0:
                self.start_lat = lat
                self.start_lon = lon
                self.logger.log("STEP_11_GPS_READY", {"lat": lat, "lon": lon, "gpsSol": gps_sol, "hAcc": h_acc_int})
                break

            # Krok 12: Zobrazení hlášky o čekání
            self.terminal.show_message(
                header="Robotour - Čekání na polohu",
                text=f"Poloha robota nebyla vyhodnocena. Aktuální stav řešení polohy je {gps_sol}, přesnost polohy je {h_acc_int} mm. Gps poloha je {lat}, {lon}.",
                buttons=[{"id": "cancel_mission", "text": "Zrušit misi"}]
            )
            btn = await self._wait_for_button(["cancel_mission"], timeout=1.0)
            if btn == "cancel_mission":
                self.logger.log("MISSION_CANCELLED_AT_GPS")
                return

        if not self.running or self._stop_requested:
            return

        # -------------------------------------------------------------
        # Krok 13 až 16: Výpočet vzdálenosti k cíli a potvrzení
        # -------------------------------------------------------------
        dist_m = calculate_geodesic_distance_m(self.start_lat, self.start_lon, self.target_lat, self.target_lon)
        self.logger.log("STEP_13_DISTANCE_CALCULATED", {"distance_m": dist_m})

        if dist_m >= 3000.0:
            # Krok 14: Cíl je příliš daleko
            self.current_step = 14
            self.terminal.show_message(
                header="Robotour - Cíl je příliš daleko",
                text=f"Cílové souřadnice jsou geo:{self.target_lat},{self.target_lon}. Vzdušná vzdálenost k cíli je {round(dist_m)} m a je mimo parametry soutěže Robotour.",
                buttons=[{"id": "rescan_qrcode", "text": "Re-Scan QR Code"}]
            )
            btn = await self._wait_for_button(["rescan_qrcode"])
            return

        # Krok 15 & 16: Potvrzení cíle
        self.current_step = 15
        self.terminal.show_message(
            header="Robotour - Potvrzení cíle",
            text=f"Cílové souřadnice jsou geo:{self.target_lat},{self.target_lon}. Vzdušná vzdálenost k cíli je {round(dist_m)} m.",
            buttons=[
                {"id": "destination_ok", "text": "Cíl je OK"},
                {"id": "rescan_qrcode", "text": "Opakovat QR Scan"},
                {"id": "cancel_mission", "text": "Zrušit misi"}
            ]
        )

        btn = await self._wait_for_button(["destination_ok", "rescan_qrcode", "cancel_mission"])
        if btn == "rescan_qrcode":
            return
        elif btn == "cancel_mission":
            return
        elif btn != "destination_ok":
            return

        # -------------------------------------------------------------
        # Krok 16.1: Start MAPS a LIDAR
        # -------------------------------------------------------------
        self.current_step = 16
        while self.running and not self._stop_requested:
            maps_port = MICROSERVICES_CONFIG["MAPS"]["port"]
            lidar_port = MICROSERVICES_CONFIG["LIDAR"]["port"]

            ok1, resp1 = send_tcp_command(self.host, maps_port, "START", timeout=2.0)
            ok2, resp2 = send_tcp_command(self.host, lidar_port, "START", timeout=2.0)

            if ok1 and resp1.startswith("OK") and ok2 and resp2.startswith("OK"):
                self.logger.log("STEP_16_1_SERVICES_STARTED")
                break
            else:
                err_text = f"MAPS: {resp1}, LIDAR: {resp2}"
                self.logger.log("STEP_16_1_FAILED", {"error": err_text})
                self.terminal.show_message(
                    header="Robotour - Chyba při startu služby",
                    text=f"Některá služba neodpověděla OK: {err_text}",
                    buttons=[
                        {"id": "try_again", "text": "Zkusit znovu"},
                        {"id": "cancel_mission", "text": "Zrušit misi"}
                    ]
                )
                btn = await self._wait_for_button(["try_again", "cancel_mission"])
                if btn == "try_again":
                    continue
                return

        # -------------------------------------------------------------
        # Krok 17 až 17.3: Hledání trasy přes MAPS FIND_ROUTE
        # -------------------------------------------------------------
        self.current_step = 17
        maps_port = MICROSERVICES_CONFIG["MAPS"]["port"]
        fusion_port = MICROSERVICES_CONFIG["FUSION"]["port"]

        while self.running and not self._stop_requested:
            self.terminal.show_message(
                header="Robotour",
                text="Hledáme cestu k cíli...",
                buttons=[{"id": "cancel_mission", "text": "Zrušit misi"}]
            )

            find_cmd = f"FIND_ROUTE {self.start_lat} {self.start_lon} {self.target_lat} {self.target_lon}"
            self.logger.log("STEP_17_1_FIND_ROUTE", {"cmd": find_cmd})
            ok, route_resp = send_tcp_command(self.host, maps_port, find_cmd, timeout=5.0)

            route_data = {}
            if ok and route_resp.startswith("{"):
                try:
                    route_data = json.loads(route_resp)
                except Exception:
                    pass

            meta = route_data.get("metadata", {})
            search_res = meta.get("search_result", "cesta nenalezena")

            if search_res == "found":
                # Krok 17.3: Cesta nalezena
                self.route_json_str = route_resp
                route_len = meta.get("route_length_m", 0.0)
                self.logger.log("STEP_17_3_ROUTE_FOUND", {"length_m": route_len})
                self.terminal.sound("notification")

                self.terminal.show_message(
                    header="Robotour - Cesta nalezena",
                    text=f"Vzdálenost k cíli po cestě: {round(route_len, 1)} m.",
                    buttons=[
                        {"id": "mission_go", "text": "Vydat se na cestu"},
                        {"id": "rescan_qrcode", "text": "Re-Scan QR Code"},
                        {"id": "cancel_mission", "text": "Zrušit misi"}
                    ]
                )

                btn = await self._wait_for_button(["mission_go", "rescan_qrcode", "cancel_mission"])
                if btn != "mission_go":
                    return
                break  # Nalezena a schválena trasa -> přechod na Krok 18

            # Cesta nenalezena
            reason = meta.get("reason", "Cesta k cíli nebyla v mapovém podkladu nalezena.")
            start_dist = meta.get("start_distance_to_map_m", 0.0)
            goal_dist = meta.get("goal_distance_to_map_m", 0.0)
            self.logger.log("STEP_17_2_ROUTE_NOT_FOUND", {"reason": reason, "start_dist": start_dist, "goal_dist": goal_dist})

            # Pokud je start příliš daleko od mapy, umožníme obsluze robota přisunout blíže
            # Nabídneme tlačítko [check_again] "Už tam jsem?" a automaticky ověřujeme polohu na pozadí
            is_start_far = start_dist > 5.0 or "Start je dále" in reason

            if is_start_far:
                self.terminal.show_message(
                    header="Robotour - Vzdálen od mapy",
                    text=f"{reason}\n\nPřesuňte robota blíže k cestě.",
                    buttons=[
                        {"id": "check_again", "text": "Už tam jsem?"},
                        {"id": "rescan_qrcode", "text": "Re-Scan QR Code"},
                        {"id": "cancel_mission", "text": "Zrušit misi"}
                    ]
                )

                # Čekáme na stisk tlačítka nebo timeout 2.5s pro automatické periodické zjištění
                btn = await self._wait_for_button(["check_again", "chcek_again", "rescan_qrcode", "cancel_mission"], timeout=2.5)
                if btn in ["rescan_qrcode", "cancel_mission"]:
                    return

                # Aktualizujeme polohu z FUSION pro další pokus FIND_ROUTE
                ok_f, resp_f = send_tcp_command(self.host, fusion_port, "DATA", timeout=1.5)
                if ok_f and resp_f.startswith("{"):
                    try:
                        f_data = json.loads(resp_f)
                        f_lat = f_data.get("lat", 0.0)
                        f_lon = f_data.get("lon", 0.0)
                        if f_lat != 0.0 and f_lon != 0.0:
                            self.start_lat = f_lat
                            self.start_lon = f_lon
                    except Exception:
                        pass
                continue
            else:
                # Cíl je mimo mapu nebo neexistuje propojení v grafu
                self.terminal.show_message(
                    header="Robotour - Cesta nebyla nalezena",
                    text=reason,
                    buttons=[
                        {"id": "rescan_qrcode", "text": "Re-Scan QR Code"},
                        {"id": "cancel_mission", "text": "Zrušit misi"}
                    ]
                )
                btn = await self._wait_for_button(["rescan_qrcode", "cancel_mission"])
                return

        # -------------------------------------------------------------
        # Krok 18: Spuštění jízdy
        # -------------------------------------------------------------
        self.current_step = 18
        self.state_name = "STEP_18_START_DRIVING"
        self.logger.log("STEP_18_GO")

        # 1. Zapnutí motorů DRIVE ON
        send_tcp_command(self.host, MICROSERVICES_CONFIG["DRIVE"]["port"], "ON", timeout=2.0)

        # 2. Spuštění pilota s předanou trasou
        pilot_cmd = f"START {self.route_json_str}"
        send_tcp_command(self.host, MICROSERVICES_CONFIG["PILOT-ROBOTOUR"]["port"], pilot_cmd, timeout=3.0)

        # 3. Zvuková a vizuální signalizace
        self.terminal.sound("barking")
        self.terminal.blink("#FFA500", 2.0, 3000)

        # -------------------------------------------------------------
        # Kroky 19 až 23: Monitorovací smyčka jízdy
        # -------------------------------------------------------------
        pilot_port = MICROSERVICES_CONFIG["PILOT-ROBOTOUR"]["port"]
        while self.running and not self._stop_requested:
            self.current_step = 19
            self.logger.log("STEP_19_CHECK_PILOT")

            ok, st_resp = send_tcp_command(self.host, pilot_port, "STATUS", timeout=2.0)
            status_data = {}
            if ok and st_resp.startswith("{"):
                try:
                    status_data = json.loads(st_resp)
                except Exception:
                    pass

            self.pilot_status = status_data
            p_state = status_data.get("state", "IDLE")
            p_info = status_data.get("info", "")
            wp_idx = status_data.get("wp_index", 0)
            wp_tot = status_data.get("wp_total", 0)
            dist_left = status_data.get("distance_to_goal_m", 0.0)
            speed = status_data.get("speed", 0.0)
            gps_sol = status_data.get("gps_sol", "NONE")

            # Krok 20: PILOT STOPPED
            if p_state == "STOPPED":
                self.current_step = 20
                self.state_name = "STEP_20_PILOT_STOPPED"
                self.logger.log("STEP_20_PILOT_STOPPED", {"info": p_info})
                self.terminal.sound("game-over")
                self.terminal.blink("#FF0000", 2.0, 3000)
                self.terminal.show_message(
                    header="Robotour - Ukončení jízdy",
                    text=f"Jízda byla ukončena: {p_info}",
                    buttons=[{"id": "acknowledge", "text": "Rozumím"}]
                )
                await self._wait_for_button(["acknowledge"])
                self._stop_driving_services()
                return

            # Krok 20.1: PILOT FINISHED
            elif p_state == "FINISHED":
                self.current_step = 20
                self.state_name = "STEP_20_1_PILOT_FINISHED"
                self.logger.log("STEP_20_1_PILOT_FINISHED", {"info": p_info})
                self.terminal.sound("meow")
                self.terminal.blink("#00FF00", 2.0, 2000)
                self.terminal.show_message(
                    header="Robotour - Jsme v cíli!",
                    text="Gratulujeme, dokončili jste misi!",
                    buttons=[{"id": "acknowledge", "text": "Rozumím"}]
                )
                await self._wait_for_button(["acknowledge"])
                self._stop_driving_services()
                return

            # Krok 21: Jízda běží (nebo stav není STOPPED)
            elif p_state == "RUNNING":
                self.current_step = 21
                self.state_name = "STEP_21_DRIVING"
                self.terminal.show_message(
                    header="Robotour - Jízda",
                    text=f"Waypoint {wp_idx}/{wp_tot} | Zbývá {dist_left} m\nRychlost {speed} m/s | GPS: {gps_sol}",
                    buttons=[
                        {"id": "pause_mission", "text": "Pause"},
                        {"id": "stop_mission", "text": "Stop"}
                    ]
                )

                # Čekáme až 2 sekundy na případný stisk tlačítka Pause/Stop
                btn = await self._wait_for_button(["pause_mission", "stop_mission"], timeout=2.0)

                if btn == "stop_mission":
                    self.logger.log("USER_STOP_MISSION")
                    send_tcp_command(self.host, pilot_port, "STOP", timeout=2.0)
                    self._stop_driving_services()
                    return

                elif btn == "pause_mission":
                    # Krok 21b: Pozastavení
                    self.logger.log("USER_PAUSE_MISSION")
                    send_tcp_command(self.host, pilot_port, "PAUSE", timeout=2.0)

                    # Krok 22: Stav pozastavení
                    self.current_step = 22
                    self.terminal.show_message(
                        header="Robotour - Pozastaveno",
                        text="Robot je pozastaven. Čekáme na vstup uživatele.",
                        buttons=[
                            {"id": "resume_mission", "text": "Pokračovat"},
                            {"id": "cancel_mission", "text": "Zrušit misi"}
                        ]
                    )

                    btn_pause = await self._wait_for_button(["resume_mission", "cancel_mission"])
                    if btn_pause == "resume_mission":
                        self.logger.log("USER_RESUME_MISSION")
                        send_tcp_command(self.host, pilot_port, "RESUME", timeout=2.0)
                        continue
                    else:
                        self.logger.log("USER_CANCEL_FROM_PAUSE")
                        send_tcp_command(self.host, pilot_port, "STOP", timeout=2.0)
                        self._stop_driving_services()
                        return

            # Krok 23: Perioda cyklu
            await asyncio.sleep(1.0)

    # =========================================================================
    # Ukončování služeb
    # =========================================================================

    def _stop_driving_services(self):
        """
        Zastaví pouze služby specifické pro jízdu (konec mise, acknowledge, cancel):
        DRIVE OFF, PILOT-ROBOTOUR STOP, MAPS STOP, LIDAR STOP.
        Polohové služby (GNSS, RTK, FUSION, LOGGER, DRIVE proces) zůstávají běžet.
        """
        print("[MissionService] Zastavuji jízdní služby (konec etapy)...")
        send_tcp_command(self.host, MICROSERVICES_CONFIG["PILOT-ROBOTOUR"]["port"], "STOP", timeout=1.5)
        send_tcp_command(self.host, MICROSERVICES_CONFIG["DRIVE"]["port"], "OFF", timeout=1.5)
        send_tcp_command(self.host, MICROSERVICES_CONFIG["MAPS"]["port"], "STOP", timeout=1.5)
        send_tcp_command(self.host, MICROSERVICES_CONFIG["LIDAR"]["port"], "STOP", timeout=1.5)
        self.logger.close()

    def _stop_all_services(self):
        """
        Zastaví VŠECHNY spuštěné služby při ukončení celé služby MISSION-ROBOTOUR (STOP / SHUTDOWN).
        """
        print("[MissionService] Zastavuji VŠECHNY spuštěné služby (celkový shutdown)...")
        all_stoppable = [
            "PILOT-ROBOTOUR",
            "MAPS",
            "LIDAR",
            "LOGGER",
            "FUSION",
            "RTK",
            "GNSS-DUAL",
            "GNSS-GPS",
            "GNSS-IMU",
            "QRSCANER"
        ]
        # Motory nejdříve vypneme
        send_tcp_command(self.host, MICROSERVICES_CONFIG["DRIVE"]["port"], "OFF", timeout=1.5)
        send_tcp_command(self.host, MICROSERVICES_CONFIG["DRIVE"]["port"], "STOP", timeout=1.5)

        for s_name in all_stoppable:
            port = MICROSERVICES_CONFIG[s_name]["port"]
            send_tcp_command(self.host, port, "STOP", timeout=1.0)
