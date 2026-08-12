import socket
import signal
import threading
import sys

from service import GnssDualService
from client_handler import client_thread

SERVICE_PORT = 9006

def main():
    service = GnssDualService()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', SERVICE_PORT))
    sock.listen(1)
    print(f"[SERVER] GNSS-DUAL Service listening on port {SERVICE_PORT}")

    shutdown_event = threading.Event()

    def handle_sigint(signum, frame):
        print("\n[SERVER] Signal received. Initiating shutdown...")
        shutdown_event.set()
        try:
            sock.close()
        except Exception:
            pass

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    while not shutdown_event.is_set():
        try:
            client_sock, addr = sock.accept()
            threading.Thread(target=client_thread, args=(client_sock, addr, service), daemon=True).start()
        except KeyboardInterrupt:
            shutdown_event.set()
            break
        except OSError:
            # Expected if sock.close() is called during accept()
            break
        except Exception as e:
            if not shutdown_event.is_set():
                print(f"[SERVER] Accept error: {e}")

    print("[SERVER] Stopping GNSS-DUAL service threads...")
    service.stop()
    print("[SERVER] GNSS-DUAL service shutdown confirmed.")
    sys.exit(0)

if __name__ == '__main__':
    main()
