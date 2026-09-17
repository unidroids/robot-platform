# oow-bridge/test_fusion_client.py
import asyncio
import json
import socket
import threading
import unittest

from fusion_client import FusionClient


class MockServiceServer:
    def __init__(self, port: int, pong_resp: str):
        self.port = port
        self.pong_resp = pong_resp
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
                            conn.sendall(f"{self.pong_resp}\n".encode("utf-8"))
                        elif cmd in ("START", "STOP"):
                            conn.sendall(b"OK\n")
                        elif cmd == "STATUS":
                            conn.sendall(b"RUNNING\n")
                        else:
                            conn.sendall(b"ERR\n")
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


class TestFusionClient(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.client = FusionClient(host="127.0.0.1")
        # Dočasně přesměrujeme porty na testovací rozsahy
        self.test_ports = {
            "GNSS-GPS": 39004,
            "GNSS-DUAL": 39006,
            "DRIVE": 39003,
            "GNSS-IMU": 39016,
            "FUSION": 39009
        }
        for name, p in self.test_ports.items():
            self.client.services[name]["port"] = p

        self.servers = {
            name: MockServiceServer(self.client.services[name]["port"], self.client.services[name]["pong"])
            for name in self.client.services
        }

    async def asyncTearDown(self):
        for s in self.servers.values():
            s.stop()

    async def test_fusion_on_and_off_include_fusion(self):
        # 1. Test FUSION_ON
        res_on = await self.client.handle_command("FUSION_ON")
        self.assertIsNotNone(res_on)
        data_on = json.loads(res_on)
        self.assertIn("GNSS-IMU", data_on)
        self.assertIn("FUSION", data_on)
        self.assertEqual(data_on["FUSION"], "OK")
        self.assertEqual(data_on["GNSS-IMU"], "OK")

        # Ověření, že FUSION i GNSS-IMU obdržely START
        self.assertIn("START", self.servers["FUSION"].received_cmds)
        self.assertIn("START", self.servers["GNSS-IMU"].received_cmds)

        # 2. Test FUSION_OFF
        res_off = await self.client.handle_command("FUSION_OFF")
        self.assertIsNotNone(res_off)
        data_off = json.loads(res_off)
        self.assertIn("FUSION", data_off)
        self.assertIn("GNSS-IMU", data_off)
        self.assertEqual(data_off["FUSION"], "OK")
        self.assertEqual(data_off["GNSS-IMU"], "OK")

        # Ověření, že FUSION i GNSS-IMU obdržely STOP
        self.assertIn("STOP", self.servers["FUSION"].received_cmds)
        self.assertIn("STOP", self.servers["GNSS-IMU"].received_cmds)

    async def test_fusion_status(self):
        res_status = await self.client.handle_command("FUSION_STATUS")
        self.assertEqual(res_status, "RUNNING")
        self.assertIn("STATUS", self.servers["FUSION"].received_cmds)


if __name__ == "__main__":
    unittest.main()
