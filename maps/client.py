# maps/client.py
"""
Obsluha TCP klientů pro mapovou službu (port 9040):
- PING -> PONG MAPS
- START -> OK
- STOP -> OK
- RESTART -> OK
- STATUS -> READY {json}
- EXIT -> OK-MAPS-BYE
- SHUTDOWN -> OK SHUTDOWN
- FIND_ROUTE <start_lat>, <start_lon>, <cil_lat>, <cil_lon> -> JSON řádek s výsledkem
"""
from __future__ import annotations
import json
import re
import socket
import sys
import traceback
from typing import Optional, List

try:
    from .service import MapService
except (ImportError, ValueError):
    from service import MapService


def parse_find_route_args(args_str: str) -> Optional[List[float]]:
    """
    Flexibilní parsování argumentů FIND_ROUTE.
    Podporuje jak čárkami, tak mezerami oddělená čísla:
    'FIND_ROUTE 49.5541, 12.7411, 49.5545, 12.7424'
    'FIND_ROUTE 49.5541 12.7411 49.5545 12.7424'
    'FIND_ROUTE 49.5541,12.7411,49.5545,12.7424'
    """
    # Nahradíme čárky mezerami a rozdělíme na tokeny
    tokens = re.split(r"[\s,]+", args_str.strip())
    tokens = [t for t in tokens if t]
    if len(tokens) != 4:
        return None
    try:
        return [float(t) for t in tokens]
    except ValueError:
        return None


def client_thread(sock: socket.socket, addr, service: MapService):
    """Vlákno pro obsluhu jednoho připojeného TCP klienta."""
    f = sock.makefile("rwb", buffering=0)
    print(f"[MAPS SERVER] Client connected: {addr}")
    try:
        while True:
            try:
                line_bytes = f.readline()
            except (ConnectionResetError, ConnectionAbortedError):
                break

            if not line_bytes:
                break

            line = line_bytes.decode("utf-8").strip()
            if not line:
                continue

            try:
                # --- Standardní řídicí příkazy ---
                if line == "PING":
                    f.write(b"PONG MAPS\n")

                elif line == "START":
                    res = service._start()
                    f.write((res + "\n").encode("utf-8"))

                elif line == "STOP":
                    res = service._stop()
                    f.write((res + "\n").encode("utf-8"))

                elif line == "RESTART":
                    res = service.restart()
                    f.write((res + "\n").encode("utf-8"))

                elif line == "STATUS":
                    res = service.get_state()
                    f.write((res + "\n").encode("utf-8"))

                elif line == "EXIT":
                    f.write(b"OK-MAPS-BYE\n")
                    f.flush()
                    break

                elif line == "SHUTDOWN":
                    f.write(b"OK SHUTDOWN\n")
                    f.flush()
                    service._stop()
                    sys.exit(0)

                # --- Byznys příkaz: FIND_ROUTE ---
                elif line.startswith("FIND_ROUTE"):
                    args_part = line[len("FIND_ROUTE"):].strip()
                    coords = parse_find_route_args(args_part)
                    if coords is None:
                        err_resp = {
                            "metadata": {
                                "search_result": "cesta nenalezena",
                                "reason": "Neplatný formát příkazu. Použijte: FIND_ROUTE <start_lat>, <start_lon>, <cil_lat>, <cil_lon>",
                                "route_length_m": 0.0
                            },
                            "nodes": [],
                            "edges": []
                        }
                        f.write((json.dumps(err_resp) + "\n").encode("utf-8"))
                    else:
                        start_lat, start_lon, goal_lat, goal_lon = coords
                        route_data = service.find_route(start_lat, start_lon, goal_lat, goal_lon)
                        payload = json.dumps(route_data, ensure_ascii=False)
                        f.write((payload + "\n").encode("utf-8"))

                # --- Neznámý příkaz ---
                else:
                    f.write(b"ERR UNKNOWN COMMAND\n")

            except Exception as e:
                print(f"[MAPS CLIENT ERROR] {e}")
                traceback.print_exc()
                try:
                    f.write(f"ERROR: {e}\n".encode("utf-8"))
                    f.flush()
                except Exception:
                    pass
                break

    except Exception as e:
        print(f"[MAPS SERVER] Client error: {e}")
        traceback.print_exc()
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print(f"[MAPS SERVER] Client disconnected: {addr}")
