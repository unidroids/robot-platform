import asyncio
import socket
import threading
import unittest

from mission_robotour_client import MissionRobotourClient

class MockMissionRobotourServer:
    def __init__(self, port: int):
        self.port = port
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
                        if cmd == "START":
                            conn.sendall(b"OK\n")
                        elif cmd == "STOP":
                            conn.sendall(b"OK\n")
                        elif cmd == "STATUS":
                            conn.sendall(b'{"service": "MISSION-ROBOTOUR", "running": true}\n')
                        else:
                            conn.sendall(b"ERR Unknown cmd\n")
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

class TestMissionRobotourClient(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.port = 39031
        self.server = MockMissionRobotourServer(self.port)
        self.client = MissionRobotourClient(host="127.0.0.1", port=self.port)

    async def asyncTearDown(self):
        self.server.stop()

    async def test_commands(self):
        # 1. MISSION_ROBOTOUR_ON -> START
        res_on = await self.client.handle_command("MISSION_ROBOTOUR_ON")
        self.assertEqual(res_on, "OK")
        self.assertIn("START", self.server.received_cmds)

        # 2. MISSION_ROBOTOUR_STATUS -> STATUS
        res_status = await self.client.handle_command("MISSION_ROBOTOUR_STATUS")
        self.assertEqual(res_status, '{"service": "MISSION-ROBOTOUR", "running": true}')
        self.assertIn("STATUS", self.server.received_cmds)

        # 3. MISSION_ROBOTOUR_OFF -> STOP
        res_off = await self.client.handle_command("MISSION_ROBOTOUR_OFF")
        self.assertEqual(res_off, "OK")
        self.assertIn("STOP", self.server.received_cmds)

        # 4. Neznámý příkaz -> None
        res_unknown = await self.client.handle_command("OTHER_COMMAND")
        self.assertIsNone(res_unknown)

if __name__ == "__main__":
    unittest.main()
