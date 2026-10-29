import asyncio
import json
import socket
import threading
import unittest

from lidar_client import LidarClient

class MockLidarServiceServer:
    def __init__(self, port: int, service_name: str, status_payload: str = "RUNNING"):
        self.port = port
        self.service_name = service_name
        self.status_payload = status_payload
        self.received_cmds = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", self.port))
        self.sock.listen(5)
        self.sock.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                conn, _ = self.sock.accept()
                with conn:
                    conn.settimeout(1.0)
                    f = conn.makefile("rwb", buffering=0)
                    while True:
                        line = f.readline()
                        if not line:
                            break
                        cmd = line.decode("utf-8").strip()
                        self.received_cmds.append(cmd)
                        if cmd == "PING":
                            conn.sendall(f"PONG {self.service_name}\n".encode("utf-8"))
                        elif cmd == "START":
                            conn.sendall(b"OK STARTED\n")
                        elif cmd == "STOP":
                            conn.sendall(b"OK STOPPED\n")
                        elif cmd == "STATUS":
                            conn.sendall(f"{self.status_payload}\n".encode("utf-8"))
                        else:
                            conn.sendall(b"ERR UNKNOWN\n")
            except socket.timeout:
                continue
            except Exception:
                break

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except Exception:
            pass
        self.thread.join(timeout=1.0)

class TestLidarClient(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.port_lidar = 39002
        self.port_col = 39222
        self.port_trav = 39223

        self.server_lidar = MockLidarServiceServer(self.port_lidar, "LIDAR", status_payload="RUNNING 1.45")
        self.server_col = MockLidarServiceServer(
            self.port_col,
            "LIDAR-COLLISIONS",
            status_payload='{"service": "LIDAR-COLLISIONS", "evaluating": true}'
        )
        self.server_trav = MockLidarServiceServer(
            self.port_trav,
            "LIDAR-TRAVERSABILITY",
            status_payload='{"service": "LIDAR-TRAVERSABILITY", "evaluating": true}'
        )

        self.client = LidarClient(
            host="127.0.0.1",
            port=self.port_lidar,
            collisions_port=self.port_col,
            traversability_port=self.port_trav
        )

    async def asyncTearDown(self):
        self.server_lidar.stop()
        self.server_col.stop()
        self.server_trav.stop()

    async def test_lidar_on_starts_all_three_services(self):
        res = await self.client.handle_command("LIDAR_ON")
        self.assertIsNotNone(res)
        data = json.loads(res)

        self.assertEqual(data["lidar_status"], "ON")
        self.assertEqual(data["LIDAR"], "OK STARTED")
        self.assertEqual(data["LIDAR-COLLISIONS"], "OK STARTED")
        self.assertEqual(data["LIDAR-TRAVERSABILITY"], "OK STARTED")

        self.assertIn("START", self.server_lidar.received_cmds)
        self.assertIn("START", self.server_col.received_cmds)
        self.assertIn("START", self.server_trav.received_cmds)

    async def test_lidar_off_stops_all_three_services(self):
        res = await self.client.handle_command("LIDAR_OFF")
        self.assertIsNotNone(res)
        data = json.loads(res)

        self.assertEqual(data["lidar_status"], "OFF")
        self.assertEqual(data["LIDAR"], "OK STOPPED")
        self.assertEqual(data["LIDAR-COLLISIONS"], "OK STOPPED")
        self.assertEqual(data["LIDAR-TRAVERSABILITY"], "OK STOPPED")

        self.assertIn("STOP", self.server_col.received_cmds)
        self.assertIn("STOP", self.server_trav.received_cmds)
        self.assertIn("STOP", self.server_lidar.received_cmds)

    async def test_lidar_status_returns_all_three_statuses(self):
        res = await self.client.handle_command("LIDAR_STATUS")
        self.assertIsNotNone(res)
        data = json.loads(res)

        self.assertEqual(data["lidar_status"], "ON")
        self.assertEqual(data["LIDAR"], "RUNNING 1.45")
        self.assertIsInstance(data["LIDAR-COLLISIONS"], dict)
        self.assertEqual(data["LIDAR-COLLISIONS"]["service"], "LIDAR-COLLISIONS")
        self.assertEqual(data["LIDAR-COLLISIONS"]["evaluating"], True)
        self.assertIsInstance(data["LIDAR-TRAVERSABILITY"], dict)
        self.assertEqual(data["LIDAR-TRAVERSABILITY"]["service"], "LIDAR-TRAVERSABILITY")
        self.assertEqual(data["LIDAR-TRAVERSABILITY"]["evaluating"], True)

    async def test_partial_failure_tolerance(self):
        # Zastavíme server pro kolize
        self.server_col.stop()

        res = await self.client.handle_command("LIDAR_ON")
        self.assertIsNotNone(res)
        data = json.loads(res)

        # Hlavní lidar i traversability se spustí, kolize mají ERR
        self.assertEqual(data["lidar_status"], "ON")
        self.assertEqual(data["LIDAR"], "OK STARTED")
        self.assertTrue(data["LIDAR-COLLISIONS"].startswith("ERR:"))
        self.assertEqual(data["LIDAR-TRAVERSABILITY"], "OK STARTED")

    async def test_unknown_command(self):
        res = await self.client.handle_command("SOME_OTHER_CMD")
        self.assertIsNone(res)

if __name__ == "__main__":
    unittest.main()

