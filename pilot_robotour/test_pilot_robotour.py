# pilot_robotour/test_pilot_robotour.py
import asyncio
import os
import socket
import sys
import threading
import time
import unittest

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from pilot_robotour.geo_utils import deg2rad, rad2deg, lla_to_ecef, ecef_to_lla
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
