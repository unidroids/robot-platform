# pilot_maraton/test_pilot_maraton.py
import asyncio
import math
import os
import socket
import sys
import threading
import time
import unittest
import json
from unittest.mock import MagicMock, patch

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from pilot_maraton.geo_utils import deg2rad, rad2deg, lla_to_ecef, ecef_to_lla, rrp_to_nose
from pilot_maraton.near_waypoint import NearWaypoint
from pilot_maraton.path_tracker import PathTracker
from pilot_maraton.service import MaratonPilotService
from pilot_maraton.main import handle_client


class TestPilotMaratonLogic(unittest.TestCase):
    """Jednotkové testy logiky sledování trasy a stavového automatu pro Maraton."""

    def setUp(self):
        self.route_data = {
            "metadata": {"area_name": "UnitTest"},
            "nodes": [
                {"id": "n1", "lat": 49.5541, "lon": 12.7411},
                {"id": "n2", "lat": 49.5545, "lon": 12.7415},
                {"id": "n3", "lat": 49.5550, "lon": 12.7420}
            ],
            "edges": []
        }
        self.service = MaratonPilotService()

    def tearDown(self):
        self.service.shutdown()

    def test_path_tracker_loaded(self):
        tracker = PathTracker(self.route_data, L_near_m=2.0)
        self.assertEqual(len(tracker.waypoints), 3)
        first_wp = tracker.waypoints[0]
        self.assertAlmostEqual(first_wp.lat, 49.5541)
        self.assertAlmostEqual(first_wp.lon, 12.7411)
        self.assertEqual(tracker.current_wp_index, 0)

    def test_path_tracker_always_starts_at_zero(self):
        """Ověření, že PathTracker vždy začíná od segmentu 0 a nepřeskakuje na vzdálenější segmenty."""
        tracker = PathTracker(self.route_data, L_near_m=2.0)
        self.assertEqual(tracker.current_wp_index, 0)
        # První update se souřadnicemi startu
        st = tracker.update(49.5541, 12.7411)
        self.assertIsNotNone(st)
        self.assertEqual(tracker.current_wp_index, 0)

    def test_state_transitions(self):
        self.assertEqual(self.service.state, "IDLE")
        
        # STOP
        self.service.stop_service()
        self.assertEqual(self.service.state, "STOPPED")
        
        # PAUSE / RESUME
        self.service.pause_service(source="TEST", info="testing pause")
        self.assertEqual(self.service.state, "STOPPED")  # from STOPPED it doesn't pause
        
        self.service.state = "RUNNING"
        self.service.pause_service(source="TEST", info="testing pause")
        self.assertEqual(self.service.state, "PAUSED")
        
        self.service.resume_service(source="TEST", info="testing resume")
        self.assertEqual(self.service.state, "RUNNING")

    def test_steering_calculation(self):
        # Heading 0°, target 0° -> left == right (rovně)
        left, right, err = self.service._calculate_steering(heading=0.0, target_heading=0.0, lidar_dist=200.0, current_speed=100.0)
        self.assertAlmostEqual(err, 0.0)
        self.assertAlmostEqual(left, right, delta=0.5)

        # Heading 0°, target 30° (doprava) -> left > right
        left, right, err = self.service._calculate_steering(heading=0.0, target_heading=30.0, lidar_dist=200.0, current_speed=100.0)
        self.assertAlmostEqual(err, 30.0)
        self.assertGreater(left, right)

    def test_acceleration_limits(self):
        # Ze stavu klidu (last_v=0) skok na target 100 -> oříznuto na max_fwd_accel_step
        self.service.last_v = 0.0
        self.service.last_w = 0.0
        l, r = self.service._apply_acceleration_limits(100.0, 100.0)
        v_res = (l + r) / 2.0
        self.assertLessEqual(v_res, self.service.max_fwd_accel_step + 1.0)

    def test_status_fields_and_rounding(self):
        # Bez dat z lidaru a fúze
        st_json = self.service.get_status()
        st = json.loads(st_json)
        self.assertIn("obstacle_distance_cm", st)
        self.assertEqual(st["obstacle_distance_cm"], -1.0)
        self.assertIn("h_acc_mm", st)
        self.assertIsInstance(st["h_acc_mm"], int)
        self.assertEqual(st["h_acc_mm"], 9999)

        # S daty lidaru a fúze s plovoucí desetinnou čárkou
        self.service.update_lidar({"distance": 85.38})
        self.service.update_fusion({"hAcc": 124.6, "lat": 49.5, "lon": 12.5, "heading": 90.0, "gpsSol": "FIX"})
        st = json.loads(self.service.get_status())
        self.assertEqual(st["obstacle_distance_cm"], 85.4)
        self.assertEqual(st["h_acc_mm"], 125)
        self.assertIsInstance(st["h_acc_mm"], int)

    def test_oow_messages(self):
        self.service.state = "RUNNING"
        # OOW ZMQ OFF
        self.service.update_oow_zmq("STATUS OFF")
        self.assertEqual(self.service.state, "PAUSED")
        self.assertEqual(self.service.source, "OOW_ZMQ")
        self.assertEqual(self.service.status_info, "OOW dohled odpojen (timeout/ztráta BLE)")

        # OOW ZMQ ON
        self.service.oow_tcp_ok = True
        self.service.update_oow_zmq("STATUS ON")
        self.assertEqual(self.service.state, "RUNNING")
        self.assertEqual(self.service.status_info, "OOW dohled obnoven")

        # OOW TCP výpadek
        self.service.set_oow_tcp_ok(False)
        self.assertEqual(self.service.state, "PAUSED")
        self.assertEqual(self.service.source, "OOW_TCP")
        self.assertEqual(self.service.status_info, "OOW spojení přerušeno")

    def test_rrp_to_nose_projection(self):
        """Test přepočtu RRP -> čumák pro různé azimuty."""
        lat0, lon0 = 50.0, 14.0
        offset_m = 0.40

        # Sever (heading 0°): posun pouze na sever
        n_lat, n_lon = rrp_to_nose(lat0, lon0, heading_deg=0.0, offset_fwd_m=offset_m)
        self.assertGreater(n_lat, lat0)
        self.assertAlmostEqual(n_lon, lon0, places=9)
        dist_n = (n_lat - lat0) * 111132.95
        self.assertAlmostEqual(dist_n, offset_m, places=3)

        # Východ (heading 90°): posun pouze na východ
        e_lat, e_lon = rrp_to_nose(lat0, lon0, heading_deg=90.0, offset_fwd_m=offset_m)
        self.assertAlmostEqual(e_lat, lat0, places=9)
        self.assertGreater(e_lon, lon0)
        dist_e = (e_lon - lon0) * (111412.84 * math.cos(math.radians(lat0)))
        self.assertAlmostEqual(dist_e, offset_m, places=3)

        # Jih (heading 180°): posun pouze na jih
        s_lat, s_lon = rrp_to_nose(lat0, lon0, heading_deg=180.0, offset_fwd_m=offset_m)
        self.assertLess(s_lat, lat0)
        self.assertAlmostEqual(s_lon, lon0, places=9)

        # Západ (heading 270°): posun pouze na západ
        w_lat, w_lon = rrp_to_nose(lat0, lon0, heading_deg=270.0, offset_fwd_m=offset_m)
        self.assertAlmostEqual(w_lat, lat0, places=9)
        self.assertLess(w_lon, lon0)

    def test_drive_firmware_workaround_on_start(self):
        """Ověření inicializačního workaroundu pro firmware (START -> DRIVE 1 1 1 -> sleep -> DRIVE 1 0 0)."""
        if self.service.drive:
            self.service.drive.disconnect()
        mock_drive = MagicMock()
        self.service.drive = mock_drive

        ok, msg = self.service.start_service(max_speed=100, max_pwm=150, route_input=self.route_data)
        self.assertTrue(ok)
        
        # Ověření, že send_start byl zavolán
        mock_drive.send_start.assert_called_once()
        
        # Ověření, že send_drive byl zavolán právě 2x s pwm=1 a speed=1 a pak speed=0
        drive_calls = [c for c in mock_drive.method_calls if c[0] == 'send_drive']
        self.assertGreaterEqual(len(drive_calls), 2)
        self.assertEqual(drive_calls[0][1], (1, 1, 1))
        self.assertEqual(drive_calls[1][1], (1, 0, 0))

    def test_csv_logging_columns(self):
        """Ověření, že CSV log obsahuje rozšířenou hlavičku s 31 sloupci."""
        with patch("pilot_maraton.service.DataLogger") as mock_logger_cls:
            mock_logger = MagicMock()
            mock_logger_cls.return_value = mock_logger
            self.service.drive = MagicMock()

            ok, msg = self.service.start_service(max_speed=100, max_pwm=150, route_input=self.route_data)
            self.assertTrue(ok)

            mock_logger.print.assert_called()
            header = mock_logger.print.call_args_list[0][0][0]
            cols = header.split(",")
            self.assertEqual(len(cols), 31)
            expected_new_cols = [
                "target_wp_index", "target_wp_lat", "target_wp_lon",
                "near_lat", "near_lon", "closest_lat", "closest_lon",
                "lookahead_dist_m", "geom_curvature", "geom_radius_m", "geom_turn_direction"
            ]
            for c in expected_new_cols:
                self.assertIn(c, cols)

    def test_collisions_corridor_and_ttc_speed_regulation(self):
        """Ověření výpočtu vzdálenosti v předním koridoru a brzdného pravidla 2/5 TTC."""
        # 72 sektorů po 5°: přední polokoule 0..17 a 54..71 volná (2000 cm), zadní 18..53 slepá (-1)
        distances = [2000 if (b < 18 or b >= 54) else -1 for b in range(72)]
        delta_d = [0] * 72

        self.service.update_collisions({
            "timestamp": time.time(),
            "heading": 45.0,
            "min_ttc_s": 999.0,
            "min_ttc_angle_deg": -1.0,
            "distances_cm": distances,
            "delta_d_cms": delta_d,
        })
        self.assertAlmostEqual(self.service._get_forward_corridor_distance_cm(), 2000.0)

        # Volno >= 6 m (600 cm) a žádné TTC -> plná rychlost 200 cm/s (2.0 m/s)
        v_lim, brk_step, reason = self.service._evaluate_collision_limits(200.0, 600.0)
        self.assertAlmostEqual(v_lim, 200.0)
        self.assertAlmostEqual(brk_step, 20.0)
        self.assertEqual(reason, "")

        # Překážka na 0.7 m (70 cm) -> okamžité zastavení (target_v = 0, nouzové brzdění 40.0)
        v_lim, brk_step, reason = self.service._evaluate_collision_limits(200.0, 70.0)
        self.assertAlmostEqual(v_lim, 0.0)
        self.assertAlmostEqual(brk_step, 40.0)
        self.assertTrue(len(reason) > 0)

        # Kolizní kurz s min_ttc_s = 1.25 s -> 2/5 TTC = 0.5 s -> max rychlost při 2 m/s^2 je 1.0 m/s (100 cm/s)
        self.service.current_speed = 200.0
        self.service.update_collisions({
            "timestamp": time.time(),
            "heading": 45.0,
            "min_ttc_s": 1.25,
            "min_ttc_angle_deg": 5.0,
            "distances_cm": distances,
            "delta_d_cms": delta_d,
        })
        v_lim, brk_step, reason = self.service._evaluate_collision_limits(200.0, 800.0)
        self.assertAlmostEqual(v_lim, 100.0)
        # Ze 200 cm/s za 0.5 s vyžaduje deceleraci 400 cm/s^2 -> krok 40.0 (0.4g)
        self.assertAlmostEqual(brk_step, 40.0)

    def test_traversability_grid_and_corridor(self):
        """Ověření dekódování 80x80 potenciálové mapy sjízdnosti (grid_b64) a napojení na koridor."""
        import base64
        grid = bytearray([100] * 6400)
        # ix=20 (x=0.0 m), iy=40 (y=0.0 m) = pozice robota -> nastavíme cost = 5 (hladký asfalt)
        grid[20 * 80 + 40] = 5
        # ix=35 (x=1.5 m vpřed), iy=42 (y=0.2 m vpravo CW) -> nastavíme cost = 42
        grid[35 * 80 + 42] = 42

        b64 = base64.b64encode(bytes(grid)).decode("ascii")
        self.service.update_traversability({
            "timestamp": time.time(),
            "heading": 90.0,
            "pitch": 1.2,
            "roll": -0.5,
            "grid_nx": 80,
            "grid_ny": 80,
            "cell_size_cm": 10,
            "origin_ix": 20,
            "origin_iy": 40,
            "valid_cells": 1200,
            "blocked_cells": 300,
            "min_corridor_clearance_cm": 185.0,
            "grid_b64": b64,
        })

        self.assertEqual(self.service.get_traversability_cost_at(0.05, 0.05), 5)
        self.assertEqual(self.service.get_traversability_cost_at(1.55, 0.25), 42)
        self.assertEqual(self.service.get_traversability_cost_at(-1.0, 0.0), 100)
        self.assertEqual(self.service.get_traversability_cost_at(10.0, 0.0), 100)  # mimo rozsah -> 100
        self.assertAlmostEqual(self.service._get_effective_obstacle_distance_cm(), 185.0)


class TestPilotMaratonTCP(unittest.TestCase):
    """Integrační test TCP protokolu služby PILOT-MARATON."""

    @classmethod
    def setUpClass(cls):
        cls.test_port = 19105
        cls.service = MaratonPilotService()
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
        self.assertEqual(resp, "PONG PILOT_MARATON")

    def test_status(self):
        resp = self._send_cmd("STATUS")
        data = json.loads(resp)
        self.assertIn("state", data)
        self.assertIn(data["state"], ["IDLE", "STOPPED", "RUNNING", "PAUSED", "FINISHED"])
        self.assertIn("wp_index", data)
        self.assertIn("wp_total", data)
        self.assertIn("distance_to_goal_m", data)
        self.assertIn("speed_actual", data)
        self.assertIn("speed_target", data)
        self.assertIn("obstacle_distance_cm", data)
        self.assertIn("h_acc_mm", data)
        self.assertIsInstance(data["h_acc_mm"], int)
        self.assertIn("source", data)
        self.assertIn("info", data)

    def test_start_with_maps_json(self):
        maps_json = json.dumps({
            "metadata": {"area_name": "UnitTest"},
            "nodes": [
                {"id": "n1", "lat": 49.5541, "lon": 12.7411},
                {"id": "n2", "lat": 49.5545, "lon": 12.7415}
            ],
            "edges": []
        })
        resp = self._send_cmd(f"START {maps_json}")
        self.assertEqual(resp, "OK")
        
        status_resp = self._send_cmd("STATUS")
        st = json.loads(status_resp)
        self.assertEqual(st["state"], "RUNNING")
        self.assertEqual(st["wp_total"], 2)

    def test_start_without_route_fails(self):
        resp = self._send_cmd("START")
        self.assertTrue(resp.startswith("ERR"), f"Očekávána ERR odpověď, získáno: {resp}")

    def test_start_with_invalid_json_fails(self):
        resp = self._send_cmd("START {invalid_json")
        self.assertTrue(resp.startswith("ERR"), f"Očekávána ERR odpověď, získáno: {resp}")

    def test_start_with_insufficient_points_fails(self):
        bad_route = json.dumps({"nodes": [{"id": "n1", "lat": 49.5541, "lon": 12.7411}]})
        resp = self._send_cmd(f"START {bad_route}")
        self.assertTrue(resp.startswith("ERR"), f"Očekávána ERR odpověď, získáno: {resp}")

    def test_stop(self):
        resp = self._send_cmd("STOP")
        self.assertEqual(resp, "OK")


if __name__ == "__main__":
    unittest.main()
