import socket
import threading
import sys
from client import handle_client

HOST = "127.0.0.1"
PORT = 9102

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((HOST, PORT))
    except Exception as e:
        print(f"❌ Nelze nabindovat port {PORT}: {e}")
        sys.exit(1)

    server.listen(5)
    server.settimeout(1.0)
    print(f"👁️ robot-pilot-vision server naslouchá na {HOST}:{PORT}")

    shutdown_event = threading.Event()
    threads = []

    try:
        while not shutdown_event.is_set():
            try:
                conn, addr = server.accept()
                print(f"📡 Klient připojen: {addr}")
                t = threading.Thread(target=handle_client, args=(conn, addr, shutdown_event), daemon=True)
                t.start()
                threads.append(t)
            except TimeoutError:
                continue
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ Chyba serveru: {e}")
                
    except KeyboardInterrupt:
        print("\n🧯 Detekováno Ctrl+C, zahajuji čisté vypnutí...")
        shutdown_event.set()

    finally:
        for t in threads:
            t.join(timeout=1.0)
        server.close()
        print("🛑 Server ukončen.")

if __name__ == "__main__":
    main()
