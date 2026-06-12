import socket
import threading
import sys
import signal
from client import handle_client
from logger_service import service

HOST = "127.0.0.1"
PORT = 9012

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
    print(f"👁️ robot-logger server naslouchá na {HOST}:{PORT}")

    shutdown_event = threading.Event()
    threads = []
    
    # Zachycení signálů pro korektní ukončení (uzavření souboru)
    def signal_handler(sig, frame):
        print("\n🧯 Detekován signál k ukončení, zahajuji čisté vypnutí...")
        service.stop()
        service.wait_for_stop()
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

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
                if not shutdown_event.is_set():
                    print(f"❌ Chyba serveru: {e}")
                
    except Exception as e:
        print(f"❌ Hlavní smyčka: {e}")

    finally:
        for t in threads:
            t.join(timeout=1.0)
        server.close()
        # Bezpečnostní pojistka pro zavření souboru
        service.stop()
        service.wait_for_stop()
        print("🛑 Server ukončen.")

if __name__ == "__main__":
    main()
