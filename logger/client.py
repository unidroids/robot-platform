import sys
import os
import traceback
from logger_service import service

def read_line(conn) -> str:
    buffer = b""
    while not buffer.endswith(b"\n"):
        chunk = conn.recv(1)
        if not chunk:
            break
        buffer += chunk
    return buffer.decode("utf-8").strip().upper()

def handle_client(conn, addr, shutdown_event):
    try:
        conn.settimeout(2.0)
        with conn:
            while not shutdown_event.is_set():
                try:
                    line = read_line(conn)
                except TimeoutError:
                    continue
                except Exception:
                    continue

                if not line:
                    break

                parts = line.split()
                cmd = parts[0]
                
                print(f"📥 Příkaz od {addr}: {line}")

                if cmd == "PING":
                    conn.sendall(b"PONG LOGGER\n")

                elif cmd == "START":
                    if service.start():
                        conn.sendall(b"OK STARTED\n")
                    else:
                        conn.sendall(b"OK ALREADY RUNNING\n")

                elif cmd == "STOP":
                    if service.stop():
                        # Počkáme na dokončení vlákna a zavření souboru
                        service.wait_for_stop()
                        conn.sendall(b"OK STOPPED\n")
                    else:
                        conn.sendall(b"OK ALREADY STOPPED\n")

                elif cmd == "STATUS":
                    status = service.get_status()
                    conn.sendall(f"{status}\n".encode())

                elif cmd == "EXIT":
                    conn.sendall(b"BYE\n")
                    return

                elif cmd == "SHUTDOWN":
                    service.stop()
                    service.wait_for_stop()
                    shutdown_event.set()
                    conn.sendall(b"SHUTTING DOWN\n")
                    return

                else:
                    conn.sendall(b"ERR Unknown cmd\n")

    except Exception as e:
        print(f"❌ Chyba klienta {addr}: {e}")
    finally:
        print(f"🔌 Odpojeno: {addr}")
