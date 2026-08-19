import socket
import threading
import signal
import sys
from client import handle_client
from vision_qrcode_service import service

shutdown_event = threading.Event()

HOST = "127.0.0.1"
PORT = 9201

client_threads = []
client_threads_lock = threading.Lock()

def sigint_handler(signum, frame):
    print("\n🧯 SIGINT zachycen, ukončuji server a vision-qrcode službu...")
    service.stop()
    shutdown_event.set()

def start_server():
    signal.signal(signal.SIGINT, sigint_handler)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((HOST, PORT))
    except Exception as e:
        print(f"❌ Nelze nabindovat port {PORT}: {e}")
        sys.exit(1)
        
    server.listen()
    print(f"👁️ robot-vision-qrcode server naslouchá na {HOST}:{PORT}")

    try:
        while not shutdown_event.is_set():
            server.settimeout(1.0)
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except Exception as e:
                if not shutdown_event.is_set():
                    print(f"❌ Chyba serveru při accept: {e}")
                continue
                
            print(f"📡 Klient připojen: {addr}")
            t = threading.Thread(target=handle_client, args=(conn, addr, shutdown_event), daemon=True)
            t.start()
            with client_threads_lock:
                client_threads.append(t)
    except Exception as e:
        print(f"❌ Hlavní smyčka: {e}")
    finally:
        try:
            server.close()
        except:
            pass
        service.stop()
        with client_threads_lock:
            for t in client_threads:
                t.join(timeout=1.0)
        print("🛑 Server ukončen.")

if __name__ == "__main__":
    start_server()
