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
        self._loop: Optional[asyncio.AbstractEventLoop] = None
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

    def _set_state(self, step: int, state_name: str, desc: str = ""):
        """Změna stavu mise se zobrazením v konzoli (print) a uložením do dataloggeru."""
        self.current_step = step
        self.state_name = state_name
        detail = f" - {desc}" if desc else ""
        print(f"[MissionService] [Krok {step}] Přechod do stavu: {state_name}{detail}")
        self.logger.log(state_name, {"step": step, "description": desc} if desc else {"step": step})

    def _send_cmd(self, service_name: str, cmd: str, timeout: Optional[float] = None) -> Tuple[bool, str]:
        """Odešle TCP příkaz do vybrané mikroslužby s detailním výpisem (print) pro debug."""
        cfg = MICROSERVICES_CONFIG.get(service_name)
        if not cfg:
            print(f"[MissionService] [TCP CHYBA] Neznámá služba {service_name}")
            return False, f"Unknown service {service_name}"
        port = cfg["port"]
        to = timeout if timeout is not None else 2.0
        short_cmd = (cmd[:70] + "...") if len(cmd) > 70 else cmd
        print(f"[MissionService] [TCP ODESLÁNO] -> {service_name} (port {port}): '{short_cmd}'")
        ok, resp = send_tcp_command(self.host, port, cmd, timeout=to)
        if ok:
            short_resp = (resp[:70] + "...") if len(resp) > 70 else resp
            print(f"[MissionService] [TCP ODPOVĚĎ] <- {service_name} (port {port}): ok=True, resp='{short_resp}'")
        else:
            print(f"[MissionService] [TCP CHYBA] <- {service_name} (port {port}): ok=False, err='{resp}'")
        return ok, resp

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
        try:
            self._loop = asyncio.get_running_loop()
        except RuntimeError:
            self._loop = None
        self._start_zmq_subscriber()
        self._loop_task = asyncio.create_task(self._run_mission_workflow())
        return True, "OK"

    def stop_mission(self) -> Tuple[bool, str]:
        """Příkaz STOP: ukončí probíhající misi a zastaví spuštěné mikroslužby."""
        if hasattr(self, "terminal") and self.terminal:
            self.terminal.hide_message()

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
        if hasattr(self, "terminal") and self.terminal:
            self.terminal.hide_message()
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
        print(f"[MissionService] [ZMQ] Stisknuto tlačítko z terminálu: '{btn_id}' (v kroku {self.current_step})")
        self.logger.log("BUTTON_CLICKED", {"button": btn_id, "step": self.current_step})
        self._last_button = btn_id
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._button_event.set)
        else:
            self._button_event.set()

    def on_qr_scanned(self, qr_text: str):
        """Callback při detekci QR kódu."""
        print(f"[MissionService] [ZMQ] Načten QR kód: '{qr_text}' (v kroku {self.current_step})")
        self.logger.log("QR_DETECTED", {"raw": qr_text, "step": self.current_step})
        self._last_qr_code = qr_text
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._qr_event.set)
        else:
            self._qr_event.set()

    async def _wait_for_button(self, allowed_buttons: List[str], timeout: Optional[float] = None) -> Optional[str]:
        """Čeká na stisk povoleného tlačítka."""
        to_str = f"{timeout} s" if timeout else "bez limitu"
        print(f"[MissionService] Čekám na stisk tlačítka z {allowed_buttons} (timeout: {to_str})...")
        start_t = time.time()
        while self.running and not self._stop_requested:
            if self._last_button in allowed_buttons:
                btn = self._last_button
                self._last_button = None
                print(f"[MissionService] _wait_for_button: zachyceno tlačítko '{btn}'")
                return btn

            self._button_event.clear()
            if self._last_button in allowed_buttons:
                btn = self._last_button
                self._last_button = None
                print(f"[MissionService] _wait_for_button: zachyceno tlačítko '{btn}'")
                return btn

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
                print(f"[MissionService] _wait_for_button: zachyceno tlačítko '{btn}'")
                return btn

            if timeout and (time.time() - start_t) >= timeout:
                break
        print(f"[MissionService] _wait_for_button: vypršel limit (žádné tlačítko)")
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
                self._set_state(0, "STEP_0_CHECK_SERVICES", "Kontrola 13 mikroslužeb (PING -> PONG)")
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
                self._set_state(1, "STEP_1_START_SERVICES", f"Start polohových služeb: {services_to_start}")
                failed_starts = []
                for s_name in services_to_start:
                    ok, resp = self._send_cmd(s_name, "START", timeout=5.0)
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
                try:
                    await self._run_mission_cycle()
                finally:
                    # Po jakémkoliv ukončení mise (cíl, zrušení cancel_mission, návrat) vždy zastavíme jízdní služby
                    self._stop_driving_services()

        except asyncio.CancelledError:
            print("[MissionService] Workflow mise byla zrušena.")
        except Exception as e:
            print(f"[MissionService] Chyba ve workflow: {e}")
            self.logger.log("WORKFLOW_EXCEPTION", {"error": str(e)})
        finally:
            if hasattr(self, "terminal") and self.terminal and (not self.running or self._stop_requested):
                self.terminal.hide_message()

    async def _run_mission_cycle(self):
        """Jednotlivý cyklus mise od kroku 2 (úvodní obrazovka) po cíl / zastavení."""
        # -------------------------------------------------------------
        # Krok 2: Úvodní dialog & otevření logu mise
        # -------------------------------------------------------------
        self._set_state(2, "STEP_2_INITIAL_SCREEN", "Úvodní dialog (MESSAGE 'Jdeme na to!')")
        self.logger.start_mission()

        self.terminal.show_message(
            header="Robotour",
            text="Jdeme na to!",
            buttons=[{"id": "scan_qrcode", "text": "Scan QR Code"}]
        )

        btn = await self._wait_for_button(["scan_qrcode"])
        if btn != "scan_qrcode":
            print(f"[MissionService] Krok 2: Konec cyklu mise (tlačítko: '{btn}')")
            return

        # -------------------------------------------------------------
        # Krok 4 až 9: Skenování a ověření QR kódu
        # -------------------------------------------------------------
        while self.running and not self._stop_requested:
            self._set_state(4, "STEP_4_SCAN_QR", "Spuštění QR scanneru (START -> QRSCANER)")

            # Spuštění QR scanneru
            ok, resp = self._send_cmd("QRSCANER", "START", timeout=1.5)
            self._last_qr_code = None
            self._qr_event.clear()

            # Čekání až 120s na QR kód
            print(f"[MissionService] Krok 5: Čekám až 120 s na načtení QR kódu přes ZMQ...")
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
                self._set_state(6, "STEP_6_QR_TIMEOUT", "Vypršel limit 120 s pro načtení QR kódu")
                self._send_cmd("QRSCANER", "STOP", timeout=1.5)
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
            self._set_state(7, "STEP_7_CHECK_QR", f"Ověření formátu QR kódu: '{qr_text}'")
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
                self._set_state(8, "STEP_8_QR_INVALID", f"Neplatný formát QR kódu: '{qr_text}', opakuji...")
                continue

            # Krok 9: Formát OK
            self._set_state(9, "STEP_9_QR_VALID", f"QR kód platný: lat={self.target_lat}, lon={self.target_lon}")
            self._send_cmd("QRSCANER", "STOP", timeout=1.5)
            self.terminal.sound("notification")
            break

        if not self.running or self._stop_requested:
            return

        # -------------------------------------------------------------
        # Krok 10 až 12: Ověření připravenosti GPS (FUSION DATA)
        # -------------------------------------------------------------
        self._set_state(10, "STEP_10_WAIT_FOR_GPS", "Dotaz na stav GPS polohy (FUSION DATA)")

        while self.running and not self._stop_requested:
            ok, resp = self._send_cmd("FUSION", "DATA", timeout=2.0)
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
                self._set_state(11, "STEP_11_GPS_READY", f"GPS poloha nalezena: lat={lat}, lon={lon}, hAcc={h_acc_int} mm")
                break

            # Krok 12: Zobrazení hlášky o čekání
            self._set_state(12, "STEP_12_WAITING_FOR_FIX", f"Čekání na přesnější polohu (hAcc={h_acc_int} mm)")
            self.terminal.show_message(
                header="Robotour - Čekání na polohu",
                text=f"Poloha robota nebyla vyhodnocena. Aktuální stav řešení polohy je {gps_sol}, přesnost polohy je {h_acc_int} mm. Gps poloha je {lat}, {lon}.",
                buttons=[{"id": "cancel_mission", "text": "Zrušit misi"}]
            )
            btn = await self._wait_for_button(["cancel_mission"], timeout=1.0)
            if btn == "cancel_mission":
                self.logger.log("MISSION_CANCELLED_AT_GPS")
                print("[MissionService] Krok 12: Uživatel zrušil misi při čekání na GPS.")
                return

        if not self.running or self._stop_requested:
            return

        # -------------------------------------------------------------
        # Krok 13 až 16: Výpočet vzdálenosti k cíli a potvrzení
        # -------------------------------------------------------------
        dist_m = calculate_geodesic_distance_m(self.start_lat, self.start_lon, self.target_lat, self.target_lon)
        self._set_state(13, "STEP_13_DISTANCE_CALCULATED", f"Vzdálenost vzdušnou čarou: {round(dist_m, 1)} m")

        if dist_m >= 3000.0:
            # Krok 14: Cíl je příliš daleko
            self._set_state(14, "STEP_14_TARGET_TOO_FAR", f"Cíl příliš daleko ({round(dist_m)} m >= 3000 m)")
            self.terminal.show_message(
                header="Robotour - Cíl je příliš daleko",
                text=f"Cílové souřadnice jsou geo:{self.target_lat},{self.target_lon}. Vzdušná vzdálenost k cíli je {round(dist_m)} m a je mimo parametry soutěže Robotour.",
                buttons=[{"id": "rescan_qrcode", "text": "Re-Scan QR Code"}]
            )
            btn = await self._wait_for_button(["rescan_qrcode"])
            return

        # Krok 15 & 16: Potvrzení cíle
        self._set_state(15, "STEP_15_CONFIRM_TARGET", f"Potvrzení cíle ({round(dist_m)} m)")
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
        # Krok 16: Start MAPS (příprava mapových podkladů)
        # -------------------------------------------------------------
        self._set_state(16, "STEP_16_START_MAPS", "Start mapové služby MAPS")
        while self.running and not self._stop_requested:
            ok, resp = self._send_cmd("MAPS", "START", timeout=3.0)
            if ok and resp.startswith("OK"):
                break
            else:
                err_text = f"MAPS: {resp}"
                self.logger.log("STEP_16_FAILED", {"error": err_text})
                self.terminal.show_message(
                    header="Robotour - Chyba při startu mapové služby",
                    text=f"Služba MAPS neodpověděla OK: {err_text}",
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
        self._set_state(17, "STEP_17_FIND_ROUTE", "Hledání trasy v mapových podkladech")

        # Úvodní zobrazení hlášky o vyhledávání trasy
        self.terminal.show_message(
            header="Robotour",
            text="Hledáme cestu k cíli...",
            buttons=[{"id": "cancel_mission", "text": "Zrušit misi"}]
        )

        find_cmd = f"FIND_ROUTE {self.start_lat} {self.start_lon} {self.target_lat} {self.target_lon}"
        self.logger.log("STEP_17_1_FIND_ROUTE", {"cmd": find_cmd})
        ok, route_resp = self._send_cmd("MAPS", find_cmd, timeout=5.0)

        route_data = {}
        if ok and route_resp.startswith("{"):
            try:
                route_data = json.loads(route_resp)
            except Exception:
                pass

        meta = route_data.get("metadata", {})
        search_res = meta.get("search_result", "cesta nenalezena")

        if search_res == "found":
            # Krok 17.3: Cesta nalezena na první pokus
            self.route_json_str = route_resp
            route_len = meta.get("route_length_m", 0.0)
            self._set_state(17, "STEP_17_3_ROUTE_FOUND", f"Trasa nalezena! Délka: {round(route_len, 1)} m")
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
        else:
            # Cesta nenalezena
            reason = meta.get("reason", "Cesta k cíli nebyla v mapovém podkladu nalezena.")
            start_dist = meta.get("start_distance_to_map_m", 0.0)
            goal_dist = meta.get("goal_distance_to_map_m", 0.0)
            self.logger.log("STEP_17_2_ROUTE_NOT_FOUND", {"reason": reason, "start_dist": start_dist, "goal_dist": goal_dist})
            print(f"[MissionService] Krok 17.2: Trasa nenalezena: {reason}")

            is_start_far = start_dist > 5.0 or "Start je dále" in reason
            if not is_start_far:
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

            # Pokud je start příliš daleko od mapy, nabídneme tlačítko [check_again] "Už tam jsem?"
            # a automaticky ověřujeme polohu na pozadí.
            # Zůstáváme na stejné obrazovce a mění se POUZE text (žádné problikávání černé obrazovky).
            far_buttons = [
                {"id": "check_again", "text": "Už tam jsem?"},
                {"id": "rescan_qrcode", "text": "Re-Scan QR Code"},
                {"id": "cancel_mission", "text": "Zrušit misi"}
            ]

            while self.running and not self._stop_requested:
                self.terminal.show_message(
                    header="Robotour - Vzdálen od mapy",
                    text=f"{reason}\n\nPřesuňte robota blíže k cestě.",
                    buttons=far_buttons
                )

                # Čekáme na stisk tlačítka nebo timeout 2.5s pro automatické periodické zjištění
                btn = await self._wait_for_button(["check_again", "chcek_again", "rescan_qrcode", "cancel_mission"], timeout=2.5)
                if btn in ["rescan_qrcode", "cancel_mission"]:
                    return

                # Aktualizujeme polohu z FUSION pro další pokus FIND_ROUTE
                ok_f, resp_f = self._send_cmd("FUSION", "DATA", timeout=1.5)
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

                # Hledáme trasu na pozadí BEZ probliknutí meziobrazovky
                find_cmd = f"FIND_ROUTE {self.start_lat} {self.start_lon} {self.target_lat} {self.target_lon}"
                self.logger.log("STEP_17_RETRY_FIND_ROUTE", {"cmd": find_cmd})
                ok, route_resp = self._send_cmd("MAPS", find_cmd, timeout=5.0)

                route_data = {}
                if ok and route_resp.startswith("{"):
                    try:
                        route_data = json.loads(route_resp)
                    except Exception:
                        pass

                meta = route_data.get("metadata", {})
                search_res = meta.get("search_result", "cesta nenalezena")

                if search_res == "found":
                    # Trasa nalezena!
                    self.route_json_str = route_resp
                    route_len = meta.get("route_length_m", 0.0)
                    self._set_state(17, "STEP_17_3_ROUTE_FOUND", f"Trasa nalezena! Délka: {round(route_len, 1)} m")
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
                    break  # Trasa nalezena a schválena -> Krok 18

                # Stále nenalezena - aktualizujeme reason a vzdálenost
                reason = meta.get("reason", "Cesta k cíli nebyla v mapovém podkladu nalezena.")
                start_dist = meta.get("start_distance_to_map_m", 0.0)
                goal_dist = meta.get("goal_distance_to_map_m", 0.0)
                self.logger.log("STEP_17_RETRY_NOT_FOUND", {"reason": reason, "start_dist": start_dist, "goal_dist": goal_dist})
                print(f"[MissionService] Krok 17 opakování: trasa nenalezena ({reason}, start_dist={start_dist}m)")

                is_start_far = start_dist > 5.0 or "Start je dále" in reason
                if not is_start_far:
                    # Start je již blízko, ale nastala jiná chyba cesty
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
        # Krok 18: Spuštění jízdy (LIDAR, DRIVE, PILOT)
        # -------------------------------------------------------------
        self._set_state(18, "STEP_18_START_DRIVING", "Spuštění LiDARu, DRIVE ON a PILOT-ROBOTOUR")

        # 1. Spuštění LiDARu
        self._send_cmd("LIDAR", "START", timeout=2.0)

        # 2. Zapnutí motorů DRIVE ON
        self._send_cmd("DRIVE", "ON", timeout=2.0)

        # 3. Spuštění pilota s předanou trasou
        pilot_cmd = f"START {self.route_json_str}"
        self._send_cmd("PILOT-ROBOTOUR", pilot_cmd, timeout=3.0)

        # 4. Zvuková a vizuální signalizace
        self.terminal.sound("barking")
        self.terminal.blink("#FFA500", 2.0, 3000)

        # -------------------------------------------------------------
        # Kroky 19 až 23: Monitorovací smyčka jízdy
        # -------------------------------------------------------------
        while self.running and not self._stop_requested:
            self._set_state(19, "STEP_19_CHECK_PILOT", "Kontrola stavu pilota")

            ok, st_resp = self._send_cmd("PILOT-ROBOTOUR", "STATUS", timeout=2.0)
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
                self._set_state(20, "STEP_20_PILOT_STOPPED", f"Pilot zastaven: {p_info}")
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
                self._set_state(20, "STEP_20_1_PILOT_FINISHED", f"Cíl dosažen: {p_info}")
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
                self._set_state(21, "STEP_21_DRIVING", f"WP {wp_idx}/{wp_tot}, zbývá {dist_left} m, rychlost {speed} m/s")
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
                    self._send_cmd("PILOT-ROBOTOUR", "STOP", timeout=2.0)
                    self._stop_driving_services()
                    return

                elif btn == "pause_mission":
                    # Krok 21b: Pozastavení
                    self.logger.log("USER_PAUSE_MISSION")
                    self._send_cmd("PILOT-ROBOTOUR", "PAUSE", timeout=2.0)

                    # Krok 22: Stav pozastavení
                    self._set_state(22, "STEP_22_PAUSED", "Robot je pozastaven")
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
                        self._send_cmd("PILOT-ROBOTOUR", "RESUME", timeout=2.0)
                        continue
                    else:
                        self.logger.log("USER_CANCEL_FROM_PAUSE")
                        self._send_cmd("PILOT-ROBOTOUR", "STOP", timeout=2.0)
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
        self._send_cmd("PILOT-ROBOTOUR", "STOP", timeout=1.5)
        self._send_cmd("DRIVE", "OFF", timeout=1.5)
        self._send_cmd("MAPS", "STOP", timeout=1.5)
        self._send_cmd("LIDAR", "STOP", timeout=1.5)
        self.logger.close()

    def _stop_all_services(self):
        """
        Zastaví VŠECHNY spuštěné služby při ukončení celé služby MISSION-ROBOTOUR (STOP / SHUTDOWN).
        """
        print("[MissionService] Zastavuji VŠECHNY spuštěné služby (celkový shutdown)...")
        if hasattr(self, "terminal") and self.terminal:
            self.terminal.hide_message()
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
        self._send_cmd("DRIVE", "OFF", timeout=1.5)
        self._send_cmd("DRIVE", "STOP", timeout=1.5)

        for s_name in all_stoppable:
            self._send_cmd(s_name, "STOP", timeout=1.0)

