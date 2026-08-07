# client.py
import sys
import json
import traceback
import socket

from service import FusionService


def ensure_running(f, fusion):
    if not fusion.running:
        f.write(b"ERR: PILOT not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock:socket.socket, addr, fusion : FusionService):
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            try:
                line = f.readline()
            except ConnectionResetError as e:
                break
            if not line:
                break
            line = line.decode('utf-8').strip()
            try:
                # --- output fusion data ---

                if line == "DATA": # data 
                    if not ensure_running(f, fusion): continue
                    sol = fusion.get_latest()
                    if not sol: 
                        f.write(b'\n')
                        continue
                    payload = sol.to_json()
                    f.write(f"{payload}\n".encode("utf-8"))

                # --- standard API ---

                elif line == "PING":
                    f.write(b'PONG FUSION\n')

                elif line == "RESTART":
                    res = fusion.restart()
                    f.write((res+'\n').encode('utf-8'))

                elif line == "STATUS":
                    res = fusion.get_state()
                    f.write((res+'\n').encode('utf-8'))

                elif line == "SHUTDOWN":
                    f.write(b'OK SHUTDOWN\n')
                    f.flush()
                    # graceful shutdown via service
                    fusion._stop()
                    sys.exit(0)

                elif line == "EXIT":
                    f.write(b'OK-FUSION-BYE\n')
                    f.flush()
                    break
                
                # --- default response ---
                else:
                    f.write(b'ERR UNKNOWN COMMAND\n')
                    f.flush()
            except Exception as e:
                print(f"[CLIENT ERROR] {e}")
                traceback.print_exc()
                try: 
                    f.write(f"ERROR: {e}\n".encode())
                    f.flush()
                except Exception:
                    pass
                finally:
                    break
                
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
        traceback.print_exc()
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
