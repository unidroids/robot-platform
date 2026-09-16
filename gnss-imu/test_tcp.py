# test_tcp.py
import socket
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

from service import GnssImuService
from client_handler import client_thread

class TestTcpClientHandler(unittest.TestCase):

    def setUp(self):
        self.service = GnssImuService()
        # Mock serial to avoid needing physical device
        self.service.gnss_serial = MagicMock()
        self.service.light_fusion = MagicMock()
        self.service.light_fusion.finish_calibration.return_value = (True, "OK")
        self.service.light_fusion.pop_20hz_increment.return_value = (time.monotonic(), 0.0, 0.0, 0)
        self.service.light_fusion.get_stats.return_value = {"bias_z": 0.0, "latest_wz": 0.0}

        # Create server socket on loopback high port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('127.0.0.1', 0))
        self.port = self.server_sock.getsockname()[1]
        self.server_sock.listen(1)

        self.stop_event = threading.Event()
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()

    def tearDown(self):
        self.stop_event.set()
        try:
            self.server_sock.close()
        except Exception:
            pass

    def _run_server(self):
        while not self.stop_event.is_set():
            try:
                client_s, addr = self.server_sock.accept()
                threading.Thread(target=client_thread, args=(client_s, addr, self.service), daemon=True).start()
            except Exception:
                break

    def _send_cmd(self, cmd: str) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', self.port))
        s.sendall((cmd + '\n').encode('utf-8'))
        f = s.makefile('rb')
        resp = f.readline().decode('utf-8').strip()
        s.close()
        return resp

    def test_ping(self):
        resp = self._send_cmd("PING")
        self.assertEqual(resp, "PONG GNSS-IMU")

    def test_status_idle(self):
        resp = self._send_cmd("STATUS")
        self.assertEqual(resp, "IDLE")

    def test_unknown_command(self):
        resp = self._send_cmd("FOOBAR")
        self.assertEqual(resp, "ERR UNKNOWN COMMAND")

if __name__ == '__main__':
    unittest.main()
