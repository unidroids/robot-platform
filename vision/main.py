# main.py – robot-vision server (Robotour 2025)
import socket
import threading
import signal
from client import handle_client
from vision_service import service

shutdown_event = threading.Event()

HOST = "127.0.0.1"
PORT = 9011

client_threads = []
client_threads_lock = threading.Lock()

def sigint_handler(signum, frame):
    print("\n🧯 SIGINT zachycen, ukončuji server a vision službu...")
    service.stop()
    shutdown_event.set()

def start_server():
    signal.signal(signal.SIGINT, sigint_handler)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen()
    print(f"👁️ robot-vision server naslouchá na {HOST}:{PORT}")

    try:
        while not shutdown_event.is_set():
            server.settimeout(1.0)
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            print(f"📡 Klient připojen: {addr}")
            t = threading.Thread(target=handle_client, args=(conn, addr, shutdown_event), daemon=True)
            t.start()
            with client_threads_lock:
                client_threads.append(t)
    except Exception as e:
        print(f"❌ Chyba serveru: {e}")
    finally:
        try:
            server.close()
        except:
            pass
        with client_threads_lock:
            for t in client_threads:
                t.join(timeout=1.0)
        print("🛑 Server ukončen.")

if __name__ == "__main__":
    start_server()
