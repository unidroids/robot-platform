# client_handler.py
import sys
import socket
import traceback

from service import CompassService

def ensure_compass(f, service: CompassService):
    if not service.running:
        f.write(b"ERR: COMPASS service not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock, addr, service: CompassService):
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8', errors='ignore').strip()
            
            try:
                if line == "PING":
                    f.write(b'PONG COMPASS\n')
                elif line == "START":
                    res = service.start()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "STOP":
                    res = service.stop()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "STATUS":
                    res = service.get_status()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "EXIT":
                    f.write(b'COMPASS-BYE\n')
                    f.flush()
                    break
                elif line == "SHUTDOWN":
                    f.write(b'COMPASS-SHUTDOWN\n')
                    f.flush()
                    import os, signal
                    os.kill(os.getpid(), signal.SIGINT)
                    break
                elif line == "SETUP_UNIT":
                    res = service.setup_unit()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "CALIBRATE_COMPASS":
                    res = service.calibrate_compass()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "CALIBRATE_ACC":
                    res = service.calibrate_acc()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "SAVE_CALIBRATION":
                    res = service.save_calibration()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "CANCEL":
                    res = service.cancel()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "FACTORY_RESET":
                    res = service.factory_reset()
                    f.write((res+'\n').encode('utf-8'))
                else:
                    f.write(b'ERR UNKNOWN COMMAND\n')
                f.flush()
            except Exception as e:
                print(f"[CLIENT ERROR] {e}")
                print(traceback.format_exc()) 
                f.write(f"ERROR: {e}\n".encode())
                f.flush()
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
        print(traceback.format_exc()) 
    finally:
        try:
            sock.close()
        except:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
