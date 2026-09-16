# fusion/test_tcp.py
import json
import socket
import threading
import time
import unittest

from service import FusionService
from client import client_thread

TEST_PORT = 19009

class TestFusionTcp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.service = FusionService()
        cls.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cls.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cls.server_sock.bind(('127.0.0.1', TEST_PORT))
        cls.server_sock.listen(5)

        cls.running = True
        cls.server_thread = threading.Thread(target=cls._accept_loop, daemon=True)
        cls.server_thread.start()
        time.sleep(0.1)

    @classmethod
    def _accept_loop(cls):
        while cls.running:
            try:
                sock, addr = cls.server_sock.accept()
                threading.Thread(target=client_thread, args=(sock, addr, cls.service), daemon=True).start()
            except Exception:
                break

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        cls.server_sock.close()
        cls.service._stop()

    def _send_cmd(self, cmd: str) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', TEST_PORT))
        f = s.makefile('rwb', buffering=0)
        f.write((cmd + "\n").encode('utf-8'))
        resp = f.readline().decode('utf-8').strip()
        s.close()
        return resp

    def test_ping(self):
        resp = self._send_cmd("PING")
        self.assertEqual(resp, "PONG FUSION")

    def test_status_enriched(self):
        resp = self._send_cmd("STATUS")
        mode, rest = resp.split(" ", 1)
        self.assertIn(mode, ["WAITING", "READY", "IDLE"])

        decoder = json.JSONDecoder()
        state_data, idx = decoder.raw_decode(rest)
        sol_data, _ = decoder.raw_decode(rest[idx:].strip())
        self.assertIn("diagnostics", state_data)
        diag = state_data["diagnostics"]
        self.assertIn("constellation", diag)
        self.assertIn("antennas", diag)
        self.assertIn("heading_hold", diag)
        self.assertIn("odometry", diag)
        self.assertIn("attitude", diag)
        self.assertEqual(diag["odometry"]["scale_factor"], 1.05)

        self.assertIn("pitch", sol_data)
        self.assertIn("roll", sol_data)
        self.assertIn("heading_source", sol_data)
        self.assertIn("antenna_status", sol_data)


if __name__ == '__main__':
    unittest.main()
