# pilot_robotour/test_pilot_robotour.py
import asyncio
import math
import os
import socket
import sys
import threading
import time
import unittest

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from pilot_robotour.geo_utils import deg2rad, rad2deg, lla_to_ecef, ecef_to_lla, rrp_to_nose
from pilot_robotour.near_waypoint import NearWaypoint
from pilot_robotour.path_tracker import PathTracker
from pilot_robotour.service import RobotourPilotService
from pilot_robotour.main import handle_client


class TestPilotRobotourLogic(unittest.TestCase):
    """Jednotkové testy logiky sledování trasy a stavového automatu."""

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
        self.service = RobotourPilotService()

    def tearDown(self):
        self.service.shutdown()

    def test_path_tracker_loaded(self):
        tracker = PathTracker(self.route_data, L_near_m=2.0)
        self.assertEqual(len(tracker.waypoints), 3)
        first_wp = tracker.waypoints[0]
        self.assertAlmostEqual(first_wp.lat, 49.5541)
        self.assertAlmostEqual(first_wp.lon, 12.7411)

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
        import json
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
        from unittest.mock import MagicMock
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


class TestPilotRobotourTCP(unittest.TestCase):
    """Integrační test TCP protokolu služby PILOT-ROBOTOUR."""

    @classmethod
    def setUpClass(cls):
        cls.test_port = 19104
        cls.service = RobotourPilotService()
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
        self.assertEqual(resp, "PONG PILOT_ROBOTOUR")

    def test_status(self):
        import json
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
        import json
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
        import json
        bad_route = json.dumps({"nodes": [{"id": "n1", "lat": 49.5541, "lon": 12.7411}]})
        resp = self._send_cmd(f"START {bad_route}")
        self.assertTrue(resp.startswith("ERR"), f"Očekávána ERR odpověď, získáno: {resp}")

    def test_stop(self):
        resp = self._send_cmd("STOP")
        self.assertEqual(resp, "OK")


if __name__ == "__main__":
    unittest.main()
