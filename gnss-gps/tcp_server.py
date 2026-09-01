import socket
import threading
import traceback

class GpsTcpServer:
    def __init__(self, port: int, serial_io):
        self.port = port
        self.serial_io = serial_io
        self.server_socket = None
        self._stop_event = threading.Event()
        self.client_threads = []
        self._lock = threading.Lock()
        self._thread = None

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        
    def _run_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen()
            
            print(f"[GpsTcpServer] Naslouchám na portu {self.port}")
            
            while not self._stop_event.is_set():
                self.server_socket.settimeout(1.0)
                try:
                    conn, addr = self.server_socket.accept()
                except socket.timeout:
                    continue
                except Exception as e:
                    if not self._stop_event.is_set():
                        print(f"[GpsTcpServer] Chyba při accept: {e}")
                    break
                    
                print(f"[GpsTcpServer] Klient připojen z {addr}")
                t = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
                t.start()
                with self._lock:
                    self.client_threads.append(t)
        except Exception as e:
            print(f"[GpsTcpServer] Fatální chyba serveru: {e}")
            traceback.print_exc()

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
                
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
            
        print("[GpsTcpServer] Server zastaven.")

    def _handle_client(self, conn, addr):
        try:
            with conn:
                while not self._stop_event.is_set():
                    conn.settimeout(1.0)
                    try:
                        data = conn.recv(4096)
                        if not data:
                            break
                        
                        # Předat přijatá data rovnou na sériovou linku
                        if self.serial_io:
                            self.serial_io.send_data(data)
                            
                        # Odeslat odpověď klientovi
                        conn.sendall(b"OK\n")
                    except socket.timeout:
                        continue
        except Exception as e:
            print(f"[GpsTcpServer] Chyba klienta {addr}: {e}")
            traceback.print_exc()
        finally:
            print(f"[GpsTcpServer] Klient odpojen: {addr}")
