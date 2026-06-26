from __future__ import annotations

import traceback
import json
import socket
from typing import Tuple

from service import DriveService

__all__ = ["client_thread"]


def client_thread(conn: socket.socket, addr: Tuple[str, int], svc: DriveService) -> None:
    print(f"[DRIVE-CRAWLER] Client connected from {addr}")
    conn.settimeout(0.5)
    verified = False
    buf = bytearray()
    try:
        while True:
            line = _recv_line(conn, buf)
            if line is None:
                continue
            if line == "":
                break

            cmdline = line.strip()
            if not cmdline:
                continue

            tokens = cmdline.split()
            cmd = tokens[0].upper()
            args = tokens[1:]

            if cmd in ("START", "STOP", "POWER_OFF", "ON", "OFF", "HALT", "BREAK"):
                print(f"[DRIVE-CRAWLER] Command from {addr}: {cmdline}")

            try:
                if cmd == "PING":
                    _send_line(conn, svc.ping())

                elif cmd == "START":
                    tok = args[0] if len(args) > 0 else None
                    state = svc.start(token=tok)
                    _send_line(conn, f"OK {state}")

                elif cmd == "TOKEN":
                    if len(args) != 1:
                        _send_line(conn, "ERROR BAD_ARGS use: TOKEN <token>")
                    else:
                        verified = svc.check_token(args[0])
                        _send_line(conn, "OK VERIFIED" if verified else "ERROR INVALID_TOKEN")

                elif cmd == "STOP":
                    state = svc.stop()
                    _send_line(conn, f"OK {state}")

                elif cmd == "STATE":
                    st = svc.get_state()
                    _send_line(conn, json.dumps(st, separators=(",",":")))

                elif cmd == "EXIT":
                    _send_line(conn, "BYE")
                    break

                elif cmd == "POWER_OFF":
                    ok = svc.power_off()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "OFF":
                    ok = svc.motors_off()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "ON":
                    if svc.is_token_required() and not verified:
                        _send_line(conn, "ERROR UNAUTHORIZED token not verified")
                    else:
                        ok = svc.motors_on()
                        _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "HALT":
                    ok = svc.halt()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "BREAK":
                    ok = svc.brake()
                    _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "DRIVE":
                    if svc.is_token_required() and not verified:
                        _send_line(conn, "ERROR UNAUTHORIZED token not verified")
                    elif len(args) != 3:
                        _send_line(conn, "ERROR BAD_ARGS use: DRIVE <max_pwm> <vL> <vR>")
                    else:
                        max_pwm = int(args[0])
                        vL = int(args[1])
                        vR = int(args[2])
                        ok = svc.drive(max_pwm, vL, vR)
                        _send_line(conn, "OK" if ok else "ERROR")

                elif cmd == "PWM":
                    if svc.is_token_required() and not verified:
                        _send_line(conn, "ERROR UNAUTHORIZED token not verified")
                    elif len(args) != 2:
                        _send_line(conn, "ERROR BAD_ARGS use: PWM <pwmL> <pwmR>")
                    else:
                        pwmL = int(args[0])
                        pwmR = int(args[1])
                        ok = svc.pwm(pwmL, pwmR)
                        _send_line(conn, "OK" if ok else "ERROR")

                else:
                    _send_line(conn, "ERROR UNKNOWN_CMD")

            except ValueError as e:
                _send_line(conn, f"ERROR {e}")
            except Exception as e:
                _send_line(conn, f"ERROR {type(e).__name__}: {e}")
                _send_line(conn, traceback.format_exc())
    except Exception:
        pass
    finally:
        print(f"[DRIVE-CRAWLER] Client disconnected: {addr}")
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass


def _send_line(conn: socket.socket, s: str) -> None:
    try:
        conn.sendall((s + "\n").encode("utf-8", errors="replace"))
    except Exception:
        raise

def _recv_line(conn: socket.socket, buf: bytearray) -> str | None:
    try:
        chunk = conn.recv(4096)
        if not chunk:
            return ""
        buf.extend(chunk)
    except socket.timeout:
        return None

    nl = buf.find(b"\n")
    if nl < 0:
        return None
    line = bytes(buf[:nl])
    del buf[: nl + 1]
    if line.endswith(b"\r"):
        line = line[:-1]
    try:
        return line.decode("utf-8", errors="replace")
    except Exception:
        return line.decode("latin1", errors="replace")
