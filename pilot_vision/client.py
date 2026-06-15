import sys
import os
import traceback
from pilot_vision_service import service

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
                args = parts[1:]

                print(f"📥 Příkaz od {addr}: {line}")

                if cmd == "PING":
                    conn.sendall(b"PONG PILOT_VISION\n")

                elif cmd == "START":
                    tok = args[0] if len(args) > 0 else None
                    max_speed = 150
                    max_pwm = 150
                    
                    if len(args) > 1:
                        try:
                            max_speed = int(args[1])
                            max_speed = max(50, min(200, max_speed))
                        except ValueError:
                            pass
                    
                    if len(args) > 2:
                        try:
                            max_pwm = int(args[2])
                        except ValueError:
                            pass

                    if not tok:
                        conn.sendall(b"ERR MISSING TOKEN\n")
                    else:
                        if service.start(token=tok, max_speed=max_speed, max_pwm=max_pwm):
                            conn.sendall(b"OK STARTED\n")
                        else:
                            conn.sendall(b"OK RESTARTED WITH NEW TOKEN\n")

                elif cmd == "STOP":
                    if service.stop():
                        conn.sendall(b"OK STOPPED\n")
                    else:
                        conn.sendall(b"OK ALREADY STOPPED\n")

                elif cmd == "STATUS":
                    status = service.get_status()
                    conn.sendall(f"{status}\n".encode())

                elif cmd == "PAUSE":
                    if service.pause():
                        conn.sendall(b"OK PAUSED\n")
                    else:
                        conn.sendall(b"OK ALREADY PAUSED OR STOPPED\n")

                elif cmd == "RESUME":
                    if service.resume():
                        conn.sendall(b"OK RESUMED\n")
                    else:
                        conn.sendall(b"OK ALREADY RUNNING OR STOPPED\n")

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
    finally:
        print(f"🔌 Odpojeno: {addr}")
