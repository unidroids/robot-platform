# client.py – obsluha klienta (Robotour 2025 cameras)
import traceback
import time
import socket
from camera_service import service

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
                    cmd = read_line(conn)
                except socket.timeout:
                    continue
                if not cmd:
                    break

                print(f"📥 Příkaz od {addr}: {cmd}")

                # --- základní příkazy ---
                if cmd == "PING":
                    conn.sendall(b"PONG CAMERA\n")

                elif cmd in ("START"):
                    if service.start():
                        conn.sendall(b"STARTED\n")
                    else:
                        conn.sendall(b"ALREADY RUNNING\n")

                elif cmd == "STOP":
                    if service.stop():
                        conn.sendall(b"STOPPED\n")
                    else:
                        conn.sendall(b"NOT RUNNING\n")

                elif cmd == "STATUS":
                    status = service.get_status()
                    conn.sendall(f"{status}\n".encode())

                elif cmd == "EXIT":
                    conn.sendall(b"BYE\n")
                    return

                elif cmd == "SHUTDOWN":
                    service.stop()
                    shutdown_event.set()
                    conn.sendall(b"SHUTTING DOWN\n")
                    return

                else:
                    conn.sendall(b"ERR Unknown cmd\n")

    except Exception as e:
        print(f"❌ Chyba klienta {addr}: {e}")
        traceback.print_exc()
    finally:
        print(f"🔌 Odpojeno: {addr}")
