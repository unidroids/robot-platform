import socket
import threading
import traceback
from worker import RtkWorker

class RtkServer:
    def __init__(self, host: str, port: int, worker: RtkWorker):
        self.host = host
        self.port = port
        self.worker = worker
        self.server_socket = None
        self._stop_event = threading.Event()
        self.client_threads = []
        self._lock = threading.Lock()

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        self._stop_event.clear()
        
        print(f"[SERVER] Naslouchám na {self.host}:{self.port}")
        
        while not self._stop_event.is_set():
            self.server_socket.settimeout(1.0)
            try:
                conn, addr = self.server_socket.accept()
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[SERVER] Chyba při accept: {e}")
                break
                
            print(f"[SERVER] Klient připojen z {addr}")
            t = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
            t.start()
            with self._lock:
                self.client_threads.append(t)

    def stop(self):
        self._stop_event.set()
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        with self._lock:
            for t in self.client_threads:
                t.join(timeout=1.0)
        print("[SERVER] TCP Server zastaven.")

    def _handle_client(self, conn, addr):
        try:
            with conn:
                buf = b""
                while not self._stop_event.is_set():
                    try:
                        conn.settimeout(1.0)
                        data = conn.recv(1024)
                        if not data:
                            break
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            cmd = line.decode('utf-8', errors='ignore').strip()
                            if not cmd:
                                continue
                                
                            print(f"[SERVER] Přijat příkaz: '{cmd}' od klienta {addr}")
                            
                            if cmd == "PING":
                                conn.sendall(b"PONG RTK\n")
                            elif cmd == "START":
                                self.worker.start()
                                conn.sendall(b"OK\n")
                            elif cmd == "STOP":
                                self.worker.stop()
                                conn.sendall(b"OK\n")
                            elif cmd == "STATUS":
                                status = self.worker.get_status()
                                conn.sendall(f"{status}\n".encode('utf-8'))
                            elif cmd == "EXIT":
                                conn.sendall(b"BYE\n")
                                return
                            elif cmd == "SHUTDOWN":
                                conn.sendall(b"SHUTDOWN_OK\n")
                                self._stop_event.set()  # Zastaví přijímací smyčku serveru, což povede k ukončení
                                return
                            else:
                                conn.sendall(b"ERR Unknown cmd\n")
                    except socket.timeout:
                        continue
        except Exception as e:
            print(f"[SERVER] Chyba klienta {addr}: {e}")
            traceback.print_exc()
        finally:
            print(f"[SERVER] Klient odpojen: {addr}")
