# mission-robotour/test_mission_robotour.py
import asyncio
import json
import os
import shutil
import socket
import sys
import tempfile
import threading
import time
import unittest

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from importlib import import_module
data_logger_mod = import_module("mission-robotour.data_logger")
MissionDataLogger = data_logger_mod.MissionDataLogger

microservices_mod = import_module("mission-robotour.microservices")
ping_service = microservices_mod.ping_service
check_all_services = microservices_mod.check_all_services
send_tcp_command = microservices_mod.send_tcp_command
MICROSERVICES_CONFIG = microservices_mod.MICROSERVICES_CONFIG

service_mod = import_module("mission-robotour.service")
MissionRobotourService = service_mod.MissionRobotourService
calculate_geodesic_distance_m = service_mod.calculate_geodesic_distance_m

terminal_mod = import_module("mission-robotour.terminal_client")
TerminalClient = terminal_mod.TerminalClient

main_mod = import_module("mission-robotour.main")
handle_client = main_mod.handle_client


class MockTcpServer:
    """Jednoduchý mock TCP server pro testování odpovědí mikroslužeb."""

    def __init__(self, port: int, response_map: dict):
        self.port = port
        self.response_map = response_map
        self.received_cmds = []
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(("127.0.0.1", self.port))
        self.server_sock.listen(5)
        self.server_sock.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                conn, _ = self.server_sock.accept()
                with conn:
                    conn.settimeout(1.0)
                    f = conn.makefile("rwb", buffering=0)
                    line = f.readline()
                    if line:
                        cmd = line.decode("utf-8").strip()
                        self.received_cmds.append(cmd)
                        cmd_word = cmd.split()[0].upper() if cmd else ""
                        resp = self.response_map.get(cmd_word, self.response_map.get(cmd, "OK\n"))
                        if isinstance(resp, str):
                            if not resp.endswith("\n"):
                                resp += "\n"
                            conn.sendall(resp.encode("utf-8"))
            except socket.timeout:
                continue
            except Exception:
                break

    def stop(self):
        self.running = False
        try:
            self.server_sock.close()
        except Exception:
            pass
        self.thread.join(timeout=1.0)


class TestMissionRobotourBasics(unittest.TestCase):
    """Testy základních funkcí: výpočet vzdálenosti, logování a PING."""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.test_dir, ignore_errors=True)

    def test_geodesic_distance(self):
        # Kramolín start a bod ~200 m severovýchodně
        lat1, lon1 = 49.554131, 12.741158
        lat2, lon2 = 49.555201, 12.743162
        dist = calculate_geodesic_distance_m(lat1, lon1, lat2, lon2)
        self.assertGreater(dist, 180.0)
        self.assertLess(dist, 250.0)

    def test_data_logger(self):
        logger = MissionDataLogger(base_dir=self.test_dir)
        path = logger.start_mission()
        self.assertTrue(os.path.exists(path))
        logger.log("TEST_EVENT", {"param": 123})
        logger.close()

        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
            self.assertIn("MISSION_INIT", content)
            self.assertIn("TEST_EVENT", content)
            self.assertIn("123", content)
            self.assertIn("MISSION_CLOSED", content)

    def test_ping_service_verification(self):
        # Spustíme mock server, který odpoví očekávaný PONG
        mock = MockTcpServer(19009, {"PING": "PONG FUSION"})
        try:
            ok, resp = ping_service("FUSION", host="127.0.0.1", port=19009)
            self.assertTrue(ok)
            self.assertEqual(resp, "PONG FUSION")

            # Test při neočekávaném PONG
            ok_bad, msg_bad = ping_service("PILOT-ROBOTOUR", host="127.0.0.1", port=19009)
            self.assertFalse(ok_bad)
            self.assertIn("Chybný název", msg_bad)
        finally:
            mock.stop()


class TestTerminalClient(unittest.TestCase):
    """Test komunikace s TERMINAL klientem."""

    def test_terminal_messages(self):
        mock = MockTcpServer(19022, {"MESSAGE": "OK", "SOUND": "OK", "BLINK": "OK"})
        try:
            client = TerminalClient(host="127.0.0.1", port=19022)
            self.assertTrue(client.show_message("Header", "Text", [{"id": "btn", "text": "OK"}]))
            self.assertTrue(client.sound("barking"))
            self.assertTrue(client.blink("#FFA500", 2.0, 3000))
            self.assertEqual(len(mock.received_cmds), 3)
            self.assertTrue(mock.received_cmds[0].startswith("MESSAGE"))
            self.assertTrue(mock.received_cmds[1].startswith("SOUND barking"))
            self.assertTrue(mock.received_cmds[2].startswith("BLINK #FFA500 2.0 3000"))
        finally:
            mock.stop()


class TestMissionServiceWorkflow(unittest.IsolatedAsyncioTestCase):
    """Test stavového automatu mise Robotour s mockovanými mikroslužbami."""

    async def asyncSetUp(self):
        # Spustíme mock servery pro vybrané klíčové mikroslužby na dedikovaných portech
        # Pro účely testu vytvoříme servis s upraveným port configem
        self.ports = {
            "QRSCANER": 29021,
            "TERMINAL": 29022,
            "DRIVE": 29003,
            "GNSS-DUAL": 29006,
            "GNSS-GPS": 29004,
            "RTK": 29015,
            "GNSS-IMU": 29016,
            "LOGGER": 29012,
            "FUSION": 29009,
            "MAPS": 29040,
            "LIDAR": 29002,
            "OOW-BRIDGE": 29030,
            "PILOT-ROBOTOUR": 29104
        }
        
        # Uložíme původní porty
        self.orig_config = {k: v["port"] for k, v in MICROSERVICES_CONFIG.items()}
        for k, p in self.ports.items():
            MICROSERVICES_CONFIG[k]["port"] = p

        # Nastavíme mock servery
        self.mocks = {}
        for name, port in self.ports.items():
            resp_map = {
                "PING": MICROSERVICES_CONFIG[name]["expected_pong"][0],
                "START": "OK",
                "STOP": "OK",
                "ON": "OK",
                "OFF": "OK"
            }
            if name == "FUSION":
                resp_map["DATA"] = json.dumps({
                    "lat": 49.554131,
                    "lon": 12.741158,
                    "gpsSol": "FIX",
                    "hAcc": 150
                })
            elif name == "MAPS":
                resp_map["FIND_ROUTE"] = json.dumps({
                    "metadata": {
                        "search_result": "found",
                        "route_length_m": 210.5
                    },
                    "nodes": [
                        {"id": "n1", "lat": 49.554131, "lon": 12.741158},
                        {"id": "n2", "lat": 49.555201, "lon": 12.743162}
                    ],
                    "edges": []
                })
            elif name == "PILOT-ROBOTOUR":
                resp_map["STATUS"] = json.dumps({
                    "state": "RUNNING",
                    "wp_index": 1,
                    "wp_total": 2,
                    "distance_to_goal_m": 150.0,
                    "speed": 0.8,
                    "gps_sol": "FIX"
                })

            self.mocks[name] = MockTcpServer(port, resp_map)

        self.service = MissionRobotourService()
        self.service.terminal = TerminalClient(host="127.0.0.1", port=self.ports["TERMINAL"])

    async def asyncTearDown(self):
        self.service.stop_mission()
        for k, p in self.orig_config.items():
            MICROSERVICES_CONFIG[k]["port"] = p
        for mock in self.mocks.values():
            mock.stop()

    async def test_full_mission_flow(self):
        # 1. Spustíme misi
        ok, msg = self.service.start_mission()
        self.assertTrue(ok)
        self.assertEqual(msg, "OK")

        # Počkáme chvíli, než proběhne krok 0 a krok 1
        await asyncio.sleep(0.4)
        self.assertEqual(self.service.current_step, 2)

        # 2. Simulace stisku 'scan_qrcode' na úvodní obrazovce
        self.service.on_button_pressed("scan_qrcode")
        await asyncio.sleep(0.2)
        self.assertEqual(self.service.current_step, 4)

        # 3. Simulace příjmu QR kódu
        self.service.on_qr_scanned("geo:49.555201,12.743162")
        await asyncio.sleep(0.3)

        # Měl by projít přes krok 10 (FUSION DATA), krok 13 (vzdálenost) na krok 15 (potvrzení cíle)
        self.assertEqual(self.service.current_step, 15)

        # 4. Simulace stisku 'destination_ok'
        self.service.on_button_pressed("destination_ok")
        await asyncio.sleep(0.3)

        # 5. Simulace stisku 'mission_go' po nalezení trasy
        self.service.on_button_pressed("mission_go")
        await asyncio.sleep(0.3)

        # Nyní jsme v monitorovací smyčce kroku 19/21
        for _ in range(30):
            if self.service.current_step in (19, 21):
                break
            await asyncio.sleep(0.1)
        self.assertIn(self.service.current_step, (19, 21))

        # Ověříme, že DRIVE dostal příkaz ON a PILOT-ROBOTOUR dostal START a LIDAR dostal START
        drive_cmds = self.mocks["DRIVE"].received_cmds
        pilot_cmds = self.mocks["PILOT-ROBOTOUR"].received_cmds
        lidar_cmds = self.mocks["LIDAR"].received_cmds
        self.assertIn("ON", drive_cmds)
        self.assertIn("START", lidar_cmds)
        self.assertTrue(any(c.startswith("START") for c in pilot_cmds))

        # 6. Změníme odpověď pilota na FINISHED
        self.mocks["PILOT-ROBOTOUR"].response_map["STATUS"] = json.dumps({
            "state": "FINISHED",
            "info": "Goal reached"
        })

        # Počkáme až stavový automat zaregistruje FINISHED (krok 20)
        for _ in range(40):
            if self.service.current_step == 20:
                break
            await asyncio.sleep(0.1)

        self.assertEqual(self.service.current_step, 20)

        # 7. Simulace potvrzení 'acknowledge' v cíli
        self.service.on_button_pressed("acknowledge")
        await asyncio.sleep(0.3)

        # Mělo by dojít k návratu na úvodní obrazovku (krok 2)
        for _ in range(30):
            if self.service.current_step == 2:
                break
            await asyncio.sleep(0.1)

        self.assertEqual(self.service.current_step, 2)

        # A motory byly vypnuty DRIVE OFF
        self.assertIn("OFF", self.mocks["DRIVE"].received_cmds)

    async def test_workflow_start_far_and_check_again(self):
        """Ověření, že při startu dále než 5m z mapy systém nabídne check_again a po přiblížení najde trasu."""
        # MAPS zpočátku vrátí, že start je moc daleko
        self.mocks["MAPS"].response_map["FIND_ROUTE"] = json.dumps({
            "metadata": {
                "search_result": "cesta nenalezena",
                "reason": "Start je dále než 5m od nejbližšího místa na mapě. Nejbližší místo je 8.50 m.",
                "start_distance_to_map_m": 8.5,
                "goal_distance_to_map_m": 1.2
            }
        })

        ok, msg = self.service.start_mission()
        self.assertTrue(ok)
        await asyncio.sleep(0.4)

        # Krok 2 -> scan_qrcode
        self.service.on_button_pressed("scan_qrcode")
        await asyncio.sleep(0.2)

        # Krok 4 -> QR kód
        self.service.on_qr_scanned("geo:49.555201,12.743162")
        await asyncio.sleep(0.3)

        # Krok 15 -> destination_ok
        self.service.on_button_pressed("destination_ok")
        await asyncio.sleep(0.4)

        # Jsme v kroku 17 a MAPS vrátilo, že start je daleko.
        # Simulujeme posun robota blíže k mapě a aktualizaci odpovědi MAPS
        self.mocks["MAPS"].response_map["FIND_ROUTE"] = json.dumps({
            "metadata": {
                "search_result": "found",
                "route_length_m": 210.5
            },
            "nodes": [
                {"id": "n1", "lat": 49.554131, "lon": 12.741158},
                {"id": "n2", "lat": 49.555201, "lon": 12.743162}
            ],
            "edges": []
        })

        # Simulace stisku 'check_again' ("Už tam jsem?")
        self.service.on_button_pressed("check_again")
        await asyncio.sleep(0.3)

        # Nyní by měl systém najít trasu a čekat na 'mission_go'
        self.service.on_button_pressed("mission_go")
        await asyncio.sleep(0.3)

        # Ověříme, že jsme se úspěšně dostali do jízdního režimu
        self.assertIn(self.service.current_step, (18, 19, 21))

    async def test_cancel_mission_stops_all_driving_services(self):
        """Ověření, že při zrušení mise (cancel_mission) se spolehlivě zastaví všechny pohybové služby."""
        ok, msg = self.service.start_mission()
        self.assertTrue(ok)
        await asyncio.sleep(0.4)

        # Krok 2 -> scan_qrcode
        self.service.on_button_pressed("scan_qrcode")
        await asyncio.sleep(0.2)

        # Krok 4 -> QR kód
        self.service.on_qr_scanned("geo:49.555201,12.743162")
        await asyncio.sleep(0.3)

        # Krok 15 -> cancel_mission
        self.service.on_button_pressed("cancel_mission")
        await asyncio.sleep(0.4)

        # Po zrušení musí být robot vrácen na Krok 2
        self.assertEqual(self.service.current_step, 2)

        # A všechny pohybové služby musí obdržet STOP / OFF
        self.assertIn("OFF", self.mocks["DRIVE"].received_cmds)
        self.assertIn("STOP", self.mocks["PILOT-ROBOTOUR"].received_cmds)
        self.assertIn("STOP", self.mocks["LIDAR"].received_cmds)
        self.assertIn("STOP", self.mocks["MAPS"].received_cmds)


class TestMissionRobotourTCP(unittest.TestCase):
    """Test TCP rozhraní služby MISSION-ROBOTOUR na portu 9031."""

    @classmethod
    def setUpClass(cls):
        cls.test_port = 19031
        cls.service = MissionRobotourService()
        cls.server_ready = threading.Event()

        def run_server():
            async def amain():
                cls.stop_event = asyncio.Event()
                cls.server = await asyncio.start_server(
                    lambda r, w: handle_client(r, w, cls.service),
                    '127.0.0.1', cls.test_port
                )
                cls.server_ready.set()
                async with cls.server:
                    await cls.stop_event.wait()

            cls.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(cls.loop)
            cls.loop.run_until_complete(amain())
            cls.loop.close()

        cls.thread = threading.Thread(target=run_server, daemon=True)
        cls.thread.start()
        cls.server_ready.wait(timeout=2.0)

    @classmethod
    def tearDownClass(cls):
        cls.service.shutdown()
        if hasattr(cls, 'loop') and hasattr(cls, 'stop_event'):
            cls.loop.call_soon_threadsafe(cls.stop_event.set)
        if hasattr(cls, 'thread'):
            cls.thread.join(timeout=1.0)

    def _send_cmd(self, cmd: str) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", self.test_port))
        s.sendall((cmd + "\n").encode("utf-8"))
        f = s.makefile("rwb", buffering=0)
        resp = f.readline().decode("utf-8").strip()
        s.close()
        return resp

    def test_ping(self):
        resp = self._send_cmd("PING")
        self.assertEqual(resp, "PONG MISSION_ROBOTOUR")

    def test_status(self):
        resp = self._send_cmd("STATUS")
        data = json.loads(resp)
        self.assertEqual(data["service"], "MISSION-ROBOTOUR")
        self.assertIn("state", data)
        self.assertIn("step", data)

    def test_stop_command(self):
        resp = self._send_cmd("STOP")
        self.assertIn(resp, ["OK", "ERROR WAS_NOT_RUNNING"])


if __name__ == "__main__":
    unittest.main()
