# maps/main.py
"""
Vstupní bod síťové služby MAPS na TCP portu 9040.
Spuštění:
  python maps/main.py [--port 9040] [--map path/to/map.json]
"""
from __future__ import annotations
import argparse
import os
import signal
import socket
import sys
import threading

try:
    from .service import MapService
    from .client import client_thread
except (ImportError, ValueError):
    from service import MapService
    from client import client_thread

DEFAULT_SERVICE_PORT = 9040


def main():
    parser = argparse.ArgumentParser(description="Unidroids Robot Platform - MAPS Service")
    parser.add_argument("--port", type=int, default=DEFAULT_SERVICE_PORT, help=f"TCP port (default: {DEFAULT_SERVICE_PORT})")
    parser.add_argument("--map", type=str, default=None, help="Cesta k souboru mapy (výchozí: defaut_map.json)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Listen host (default: 127.0.0.1)")
    args = parser.parse_args()

    service = MapService(map_file_path=args.map)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind((args.host, args.port))
    except Exception as e:
        print(f"[MAPS SERVER ERROR] Cannot bind to {args.host}:{args.port} - {e}")
        sys.exit(1)

    sock.listen(5)
    print(f"[MAPS SERVER] Service MAPS listening on {args.host}:{args.port}")

    def handle_sigint(signum, frame):
        print("\n[MAPS SERVER] Stopping MAPS service...")
        service._stop()
        try:
            sock.close()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    while True:
        try:
            client_sock, addr = sock.accept()
            threading.Thread(
                target=client_thread,
                args=(client_sock, addr, service),
                daemon=True
            ).start()
        except (KeyboardInterrupt, SystemExit):
            break
        except Exception as e:
            print(f"[MAPS SERVER] Accept error: {e}")
            break


if __name__ == "__main__":
    main()
