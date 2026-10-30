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

from pilot_maraton.geo_utils import deg2rad, rad2deg, lla_to_ecef, ecef_to_lla, rrp_to_nose, displace_wgs84
from pilot_maraton.near_waypoint import NearWaypoint
from pilot_maraton.path_tracker import PathTracker
from pilot_maraton.corridor_planner import (
    RouteCorridor,
    ObstacleAvoidanceController,
    CORRIDOR_LEFT_M,
    CORRIDOR_RIGHT_M,
)
from pilot_maraton.service import MaratonPilotService
from pilot_maraton.main import handle_client, TCP_STREAM_LIMIT


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
        cls.service.drive = MagicMock()
        cls.service.set_oow_tcp_ok = MagicMock()
        cls.server_ready = threading.Event()

        def run_server():
            async def amain():
                cls.stop_event = asyncio.Event()
                cls.server = await asyncio.start_server(
                    lambda r, w: handle_client(r, w, cls.service),
                    '127.0.0.1', cls.test_port,
                    limit=TCP_STREAM_LIMIT
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
        try:
            self.assertEqual(resp, "OK")
            status_resp = self._send_cmd("STATUS")
            st = json.loads(status_resp)
            self.assertEqual(st["state"], "RUNNING")
            self.assertEqual(st["wp_total"], 2)
        finally:
            self._send_cmd("STOP")

    def test_start_with_large_5km_maps_json_over_64kb(self):
        """Regresní test: trasa > 5.5 km (> 120 KB JSON na jednom řádku) nesmí narazit na 64 KiB StreamReader limit."""
        nodes = []
        edges = []
        base_lat, base_lon = 50.10455, 14.43031
        for i in range(600):
            nid = f"stromovka_node_{i:04d}"
            nodes.append({
                "id": nid,
                "lat": round(base_lat + i * 0.00009, 7),  # ~10 m krok -> ~6.0 km trasa
                "lon": round(base_lon + (i % 5) * 0.00001, 7),
                "surface": "asphalt_promenade",
                "description": "Královská obora Stromovka - pěší promenáda maraton",
            })
            if i > 0:
                edges.append({
                    "from": f"stromovka_node_{i-1:04d}",
                    "to": nid,
                    "length_m": 10.02,
                    "width_m": 3.5,
                })
        large_json = json.dumps({
            "metadata": {"area_name": "Královská obora (Stromovka v5)", "route_length_m": 5990.0},
            "nodes": nodes,
            "edges": edges,
        }, ensure_ascii=False)
        self.assertGreater(len(large_json.encode("utf-8")), 90000)

        resp = self._send_cmd(f"START {large_json}")
        try:
            self.assertEqual(resp, "OK")
            st = json.loads(self._send_cmd("STATUS"))
            self.assertEqual(st["state"], "RUNNING")
            self.assertEqual(st["wp_total"], 600)

            # Ověření rychlosti projekce na 6 km koridoru (~30 000 vzorků)
            t0 = time.perf_counter()
            for _ in range(36):
                self.service.corridor_planner.corridor.project_enu(2.0, 55.0, hint_seg_idx=5)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self.assertLess(elapsed_ms, 25.0)
        finally:
            self._send_cmd("STOP")

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


class TestCorridorAndObstacleAvoidance(unittest.TestCase):
    """Testy koridoru [-2 m, +1 m], vyčkávání 5->60 s, zametené oblasti, předjíždění chodců a stavu NO_PATH."""

    def setUp(self):
        # Přímá severní trasa délky 40 m + odbočka 90° na východ délky 20 m
        self.lat0, self.lon0 = 50.000000, 14.000000
        lat1, lon1 = displace_wgs84(self.lat0, self.lon0, d_north_m=40.0, d_east_m=0.0)
        lat2, lon2 = displace_wgs84(lat1, lon1, d_north_m=0.0, d_east_m=20.0)
        self.route = {
            "nodes": [
                {"id": "w0", "lat": self.lat0, "lon": self.lon0},
                {"id": "w1", "lat": lat1, "lon": lon1},
                {"id": "w2", "lat": lat2, "lon": lon2},
            ]
        }
        self.tracker = PathTracker(self.route, L_near_m=2.0)
        self.corridor = RouteCorridor(self.tracker.waypoints)
        self.ctrl = ObstacleAvoidanceController(self.corridor)

    @staticmethod
    def _empty_collisions() -> dict:
        return {
            "distances_cm": [2000 if (b < 18 or b >= 54) else -1 for b in range(72)],
            "delta_d_cms": [0] * 72,
            "min_ttc_s": 999.0,
            "min_ttc_angle_deg": -1.0,
        }

    @classmethod
    def _collisions_with_point(cls, x_fwd_m: float, y_cw_m: float) -> dict:
        col = cls._empty_collisions()
        dist_cm = math.hypot(x_fwd_m, y_cw_m) * 100.0
        ang_deg = math.degrees(math.atan2(y_cw_m, x_fwd_m)) % 360.0
        sec = int(ang_deg // 5.0) % 72
        col["distances_cm"][sec] = round(dist_cm, 1)
        return col

    def test_corridor_bounds_and_filleted_intersection(self):
        """Ověření mezí koridoru [-2.0 m vlevo, +1.0 m vpravo] a zaoblení rohu + přepnutí bisektory."""
        self.assertAlmostEqual(CORRIDOR_LEFT_M, 2.0)
        self.assertAlmostEqual(CORRIDOR_RIGHT_M, 1.0)
        self.assertAlmostEqual(min(self.ctrl.lat_offsets), -2.0)
        self.assertAlmostEqual(max(self.ctrl.lat_offsets), 1.0)

        # Těsně před rohem w1 (ve vzdálenosti s = 39.2 m) už zaoblená tečna stáčí doprava (te > 0)
        smp_turn = self.corridor.sample_at_s(39.2)
        self.assertGreater(smp_turn.te, 0.15)

        # Přepnutí vnitřního rohu v PathTrackeru při bočním posunu (např. +0.8 m vpravo u pravotočivé odbočky)
        lat_inside, lon_inside = displace_wgs84(self.lat0, self.lon0, d_north_m=39.3, d_east_m=0.85)
        st = self.tracker.update(lat_inside, lon_inside)
        self.assertIsNotNone(st)
        self.assertEqual(self.tracker.current_wp_index, 1)

    def test_rough_terrain_avoids_on_the_fly_without_stopping(self):
        """Hrubý terén / obrubník v LIDAR-TRAVERSABILITY bez 3D překážky -> plynulé objetí v CRUISING bez zastavení."""
        def trav_cost(x_fwd: float, y_cw: float) -> int:
            # Přímo před robotem (y kolem 0) je rozbitý povrch / obrubník (cost 85..100), vlevo (y < -0.55) hladký asfalt (cost 3)
            if 0.8 <= x_fwd <= 3.5 and -0.55 <= y_cw <= 0.50:
                return 90
            return 3

        dec = self.ctrl.step(
            lat_rrp=self.lat0,
            lon_rrp=self.lon0,
            heading_deg=0.0,
            current_speed_cms=150.0,
            max_speed_cms=180.0,
            current_wp_index=0,
            default_target_heading_deg=0.0,
            collision_data=self._empty_collisions(),
            collisions_active=True,
            traversability_active=True,
            get_traversability_cost_fn=trav_cost,
            now=1000.0,
        )
        self.assertEqual(dec.nav_state, "CRUISING")
        self.assertAlmostEqual(dec.speed_cap_cms, 180.0)
        self.assertLess(dec.lateral_offset_m, -0.5)
        # Azimut míří doleva (západně od severu -> 270..359°)
        self.assertGreater(dec.target_heading_deg, 270.0)

    def test_static_3d_obstacle_stops_waits_5s_early_clear_and_detour(self):
        """Statická 3D překážka na 1.40 m -> zastaví, čeká 5 s, poté objede vlevo max rychlostí 1.0 m/s."""
        col_obs = self._collisions_with_point(x_fwd_m=1.40, y_cw_m=0.0)

        # t = 100.0 s: zastavení u překážky -> WAITING_CLEARANCE
        dec0 = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            col_obs, True, False, lambda x, y: 0, now=100.0,
        )
        self.assertEqual(dec0.nav_state, "WAITING_CLEARANCE")
        self.assertEqual(dec0.speed_cap_cms, 0.0)
        self.assertIn("statická", dec0.nav_info)

        # t = 104.5 s (4.5 s čekání): stále čeká
        dec4 = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            col_obs, True, False, lambda x, y: 0, now=104.5,
        )
        self.assertEqual(dec4.nav_state, "WAITING_CLEARANCE")
        self.assertEqual(dec4.speed_cap_cms, 0.0)

        # t = 105.1 s (> 5.0 s): přechází do AVOIDING, rychlost omezena na 100 cm/s (1.0 m/s), objíždí vlevo (< 0)
        dec5 = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            col_obs, True, False, lambda x, y: 0, now=105.1,
        )
        self.assertEqual(dec5.nav_state, "AVOIDING")
        self.assertAlmostEqual(dec5.speed_cap_cms, 100.0)
        self.assertLess(dec5.lateral_offset_m, -0.5)
        self.assertIn("vlevo", dec5.nav_info)

        # Test dřívějšího uvolnění během čekání: nová instance, překážka po 1.5 s zmizí
        ctrl2 = ObstacleAvoidanceController(self.corridor)
        ctrl2.step(self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0, col_obs, True, False, lambda x, y: 0, now=200.0)
        self.assertEqual(ctrl2.nav_state, "WAITING_CLEARANCE")
        ctrl2.step(self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0, self._empty_collisions(), True, False, lambda x, y: 0, now=201.0)
        dec_clear = ctrl2.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            self._empty_collisions(), True, False, lambda x, y: 0, now=202.1,
        )
        self.assertEqual(dec_clear.nav_state, "CRUISING")
        self.assertAlmostEqual(dec_clear.speed_cap_cms, 180.0)

    def test_dynamic_obstacle_extends_wait_up_to_60s_and_swept_zone(self):
        """Pohybující se překážka (>= 20 cm posuny) prodlužuje čekání až na 60 s a vytváří zametenou oblast."""
        t0 = 500.0
        # Start u překážky na y = +0.2 m
        dec = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            self._collisions_with_point(1.40, 0.20), True, False, lambda x, y: 0, now=t0,
        )
        self.assertEqual(dec.nav_state, "WAITING_CLEARANCE")

        # Každé 4 s se překážka posune o >= 25 cm mezi y = -0.35 m a y = +0.25 m -> prodlužuje až do t0 + 60 s
        positions_y = [-0.10, -0.35, -0.05, 0.25, -0.10, -0.35, -0.05, 0.25, -0.10, -0.35, -0.05, 0.25, -0.10, 0.20]
        t_cur = t0
        for idx, py in enumerate(positions_y):
            t_cur = t0 + (idx + 1) * 4.0  # 4, 8, ..., 56 s
            dec = self.ctrl.step(
                self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
                self._collisions_with_point(1.40, py), True, False, lambda x, y: 0, now=t_cur,
            )
            self.assertEqual(dec.nav_state, "WAITING_CLEARANCE", f"Měl stále čekat v čase +{t_cur - t0}s")
            self.assertIn("pohybující se", dec.nav_info)

        # V čase t0 + 60.1 s (1 minuta) vyprší strop, i když se překážka naposledy pohnula v 56 s!
        # Překážka nyní stojí vpravo (y = +0.25 m), zametená zóna pokrývá [-0.85..+0.75 m] -> objede čistě vlevo (< -1.0 m)
        dec_after_60 = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            self._collisions_with_point(1.45, 0.25), True, False, lambda x, y: 0, now=t0 + 60.1,
        )
        self.assertEqual(dec_after_60.nav_state, "AVOIDING")
        self.assertLessEqual(dec_after_60.lateral_offset_m, -1.0)

        # Dvoustupňový fallback: i kdyby historická zametená oblast pokryla celou šířku koridoru [-2..+1 m],
        # ale překážka nyní stojí jen vpravo (+0.6 m), vlevo je zametená oblast průjezdná s penalizací 92 (< 100)!
        ctrl_full_sweep = ObstacleAvoidanceController(self.corridor)
        ctrl_full_sweep.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            self._collisions_with_point(1.40, 0.0), True, False, lambda x, y: 0, now=800.0,
        )
        for sweep_y in (-1.9, -1.4, -0.9, -0.4, 0.1, 0.6):
            ctrl_full_sweep._record_swept_footprint([(sweep_y, 1.45)], now=802.0)
        # Překážka se v t=802.0 přesune na pravou stranu stopy (+0.45 m -> stále blokuje střed, prodlouží o +5 s do 807.0 s) a zůstane stát
        ctrl_full_sweep.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            self._collisions_with_point(1.45, 0.45), True, False, lambda x, y: 0, now=802.0,
        )
        dec_fallback = ctrl_full_sweep.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0,
            self._collisions_with_point(1.45, 0.45), True, False, lambda x, y: 0, now=807.2,
        )
        self.assertTrue(dec_fallback.path_found)
        self.assertEqual(dec_fallback.nav_state, "AVOIDING")
        self.assertLessEqual(dec_fallback.lateral_offset_m, -0.6)

    def test_same_direction_pedestrian_overtake_threshold_06ms_and_blind_spot(self):
        """
        Chodec ve stejném směru:
        1) Rychlý chodec (1.35 m/s při max 1.80 m/s -> rozdíl 0.45 < 0.60 m/s) -> robot jen plynule NÁSLEDUJE (FOLLOWING).
        2) Pomalejší chodec (1.00 m/s při max 1.80 m/s -> rozdíl 0.80 >= 0.60 m/s) -> po 2.5 s PŘEDJÍŽDÍ plnou rychlostí
           a drží levý pruh, dokud nemá náskok >= 2.2 m před predikovanou polohou chodce ve slepém úhlu!
        """
        # 1) Rychlý chodec 1.35 m/s (rozdíl 0.45 m/s < 0.6 m/s)
        ctrl_fast = ObstacleAvoidanceController(self.corridor)
        t = 1000.0
        r_north = 0.0
        ped_north = 2.2
        for step_i in range(40):  # 4.0 s po 0.1 s
            dt = 0.1
            t += dt
            r_north += 1.35 * dt
            ped_north += 1.35 * dt
            rlat, rlon = displace_wgs84(self.lat0, self.lon0, d_north_m=r_north, d_east_m=0.0)
            col = self._collisions_with_point(ped_north - r_north, 0.0)
            dec = ctrl_fast.step(rlat, rlon, 0.0, 135.0, 180.0, 0, 0.0, col, True, False, lambda x, y: 0, now=t)
        self.assertEqual(dec.nav_state, "FOLLOWING")

        # 2) Pomalejší chodec 1.00 m/s (rozdíl 0.80 m/s >= 0.60 m/s)
        ctrl_slow = ObstacleAvoidanceController(self.corridor)
        t = 2000.0
        r_north = 0.0
        ped_north = 2.2
        dec = None
        for step_i in range(32):  # 3.2 s > 2.5 s pozorování
            dt = 0.1
            t += dt
            r_north += 1.00 * dt
            ped_north += 1.00 * dt
            rlat, rlon = displace_wgs84(self.lat0, self.lon0, d_north_m=r_north, d_east_m=0.0)
            col = self._collisions_with_point(ped_north - r_north, 0.0)
            dec = ctrl_slow.step(rlat, rlon, 0.0, 100.0, 180.0, 0, 0.0, col, True, False, lambda x, y: 0, now=t)

        self.assertEqual(dec.nav_state, "OVERTAKING")
        self.assertAlmostEqual(dec.speed_cap_cms, 180.0)  # Plná rychlost při předjíždění!
        self.assertLessEqual(dec.lateral_offset_m, -0.9)

        # Nyní robot v levém pruhu (-1.2 m) mine chodce o pouhý 1.0 m (chodec je už za LiDARem -> prázdné kolize!):
        # Predikce slepého úhlu (náskok 1.0 m < 2.2 m) musí robota stále držet v OVERTAKING vlevo!
        t += 1.0
        ped_north += 1.0 * 1.0
        r_north = ped_north + 1.0  # Jen 1.0 m před chodcem (< 2.2 m)
        rlat, rlon = displace_wgs84(self.lat0, self.lon0, d_north_m=r_north, d_east_m=-1.2)
        dec_blind = ctrl_slow.step(
            rlat, rlon, 0.0, 180.0, 180.0, 0, 0.0,
            self._empty_collisions(), True, False, lambda x, y: 0, now=t,
        )
        self.assertEqual(dec_blind.nav_state, "OVERTAKING")
        self.assertLess(dec_blind.lateral_offset_m, -0.8)

        # Jakmile má robot náskok >= 2.5 m před predikovanou polohou chodce a vrátí se ke stopě -> CRUISING
        t += 2.5
        ped_north += 1.0 * 2.5
        r_north = ped_north + 2.8
        rlat, rlon = displace_wgs84(self.lat0, self.lon0, d_north_m=r_north, d_east_m=-0.1)
        dec_done = ctrl_slow.step(
            rlat, rlon, 0.0, 180.0, 180.0, 0, 0.0,
            self._empty_collisions(), True, False, lambda x, y: 0, now=t,
        )
        self.assertEqual(dec_done.nav_state, "CRUISING")

    def test_no_path_blocked_corridor_recovery_and_goal_occupied(self):
        """Zablokovaný celý koridor -> NO_PATH (v=0), po uvolnění pokračuje; překážka v cíli -> FINISHED."""
        # Zeď 3D překážek přes celý koridor ve vzdálenosti 1.40 m
        col_wall = self._empty_collisions()
        for b in list(range(0, 13)) + list(range(59, 72)):
            col_wall["distances_cm"][b] = 140.0

        self.ctrl.step(self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0, col_wall, True, False, lambda x, y: 0, now=3000.0)
        dec_blocked = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0, col_wall, True, False, lambda x, y: 0, now=3005.2,
        )
        self.assertEqual(dec_blocked.nav_state, "NO_PATH")
        self.assertFalse(dec_blocked.path_found)
        self.assertEqual(dec_blocked.speed_cap_cms, 0.0)

        # Cesta se úplně uvolní na >= 1.0 s -> přejde zpět do jízdy
        self.ctrl.step(self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0, self._empty_collisions(), True, False, lambda x, y: 0, now=3006.0)
        dec_rec = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 180.0, 0, 0.0, self._empty_collisions(), True, False, lambda x, y: 0, now=3007.2,
        )
        self.assertIn(dec_rec.nav_state, ("CRUISING", "AVOIDING"))
        self.assertGreater(dec_rec.speed_cap_cms, 0.0)

        # Statická překážka přímo na posledním waypointu cíle (<= 2.0 m od konce trasy) -> goal_reached_near_obstacle
        lat_end_prev, lon_end_prev = displace_wgs84(self.lat0, self.lon0, d_north_m=40.0, d_east_m=18.5)
        ctrl_goal = ObstacleAvoidanceController(self.corridor)
        col_goal = self._collisions_with_point(1.40, 0.0)  # Robot míří na východ (90°), překážka 1.4 m před ním na cíli
        ctrl_goal.step(lat_end_prev, lon_end_prev, 90.0, 0.0, 180.0, 1, 90.0, col_goal, True, False, lambda x, y: 0, now=4000.0)
        dec_goal = ctrl_goal.step(
            lat_end_prev, lon_end_prev, 90.0, 0.0, 180.0, 1, 90.0, col_goal, True, False, lambda x, y: 0, now=4005.2,
        )
        self.assertTrue(dec_goal.goal_reached_near_obstacle)

    def test_unmeasured_traversability_grid_without_3d_obstacle_allows_cruising(self):
        """Nezměřená/neviděná mřížka Traversability (cost 100) bez 3D překážky nezamrzne v NO_PATH ani v nárazníku 70 cm."""
        # 1. Plánovač koridoru: všechny buňky mají cost 100, ale v dráze není 3D překážka
        dec = self.ctrl.step(
            self.lat0, self.lon0, 0.0, 0.0, 170.0, 0, 0.0,
            self._empty_collisions(), True, True, lambda x, y: 100, now=5000.0,
        )
        self.assertTrue(dec.path_found)
        self.assertEqual(dec.nav_state, "CRUISING")
        self.assertGreater(dec.speed_cap_cms, 0.0)

        # 2. Služba a nárazník: traversability clearance <= 70 cm nesmí vyvolat nouzový stop target_v = 0
        service = MaratonPilotService()
        service.max_speed = 170.0
        service.state = "RUNNING"
        service.collision_data = self._empty_collisions()
        service.last_collision_time = 5000.0
        service.traversability_data = {"min_corridor_clearance_cm": 50.0}
        service.last_traversability_time = 5000.0
        service.traversability_corridor_cm = 50.0

        dir_d = service._get_directional_obstacle_distance_cm(0.0, now_ts=5000.0)
        self.assertGreater(dir_d, 70.0)  # Traversability je omezena na > 70 cm

        target_v, _, stop_reason = service._evaluate_collision_limits(170.0, dir_d, now_ts=5000.0)
        self.assertGreater(target_v, 0.0)
        self.assertEqual(stop_reason, "")


if __name__ == "__main__":
    unittest.main()
