import socket
import threading

import asyncio
import cv2

import os
from datetime import datetime
import time

from pyzbar import pyzbar
import traceback

from collections import deque
from threading import Condition

import struct

# maximální počet uložených snímků; 30 ≈ 3 s při 10 fps
BUFFER_SIZE = 3

# vlákna klintů
shutdown_flag = False

# zakladní vlákna : ctení kamer, logování
state_lock = threading.Lock()
loop_running = False
log_running = False
loop_thread = None
log_thread = None

# kruhové buffery pro levý/pravý obraz
left_buf  = deque(maxlen=BUFFER_SIZE)
right_buf = deque(maxlen=BUFFER_SIZE)

# Sekvenční číslo posledního uloženého snímku
frame_seq = 0 

# Condition (obsahuje interní Lock)
frame_cond = Condition()

#promenne ke QR kodu
qr_running = False
qr_thread = None
qr_lock = threading.Lock()
qr_result = None
qr_ready = threading.Event()


HOST = '127.0.0.1'   # Lokální přístup
PORT = 9001          # Port pro pravou kameru (můžeme později rozšířit)

def read_line(conn):
    buffer = b""
    while not buffer.endswith(b"\n"):
        chunk = conn.recv(1)
        if not chunk:
            break
        buffer += chunk
    return buffer.decode("utf-8").strip().upper()

def handle_client(conn, addr):
    print(f"📡 Klient připojen: {addr}")
    global loop_running, loop_thread
    global log_running, log_thread
    global shutdown_flag
    global qr_running, qr_thread, qr_lock, qr_result, qr_ready
    try:
        conn.settimeout(2.0)
        with conn:
            while True:
                try:
                    cmd = read_line(conn)
                    if not cmd:
                        break
                except socket.timeout:
                    if (shutdown_flag):
                        conn.sendall(b"SERVER SHUTDOWN\n")
                        conn.sendall(b'')
                        conn.shutdown(socket.SHUT_WR)
                        conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,struct.pack('ii', 1, 0))
                        conn.close()
                        break
                    else:
                        continue  # jinak jen čekáme dál


                print(f"📥 Příkaz: '{cmd}'")

                if cmd == "PING": # PING - communication test
                    conn.sendall(b"PONG CAMERA\n")

                elif cmd == "HI": # PING - communication test
                    conn.sendall(b"HI\n")

                elif (cmd == "RUN" or cmd == "START"): # RUN - start internal loop
                    with state_lock:
                        if not loop_running:
                            loop_running = True
                            loop_thread = threading.Thread(target=camera_loop_thread)
                            loop_thread.start()
                            conn.sendall(b"LOOP OK\n")
                        else:
                            conn.sendall(b"LOOP ALREADY\n")

                        if not log_running:
                            log_running = True
                            log_thread = threading.Thread(target=log_loop_thread)
                            log_thread.start()
                            conn.sendall(b"LOG OK\n")
                        else:
                            conn.sendall(b"LOG ALREADY\n")

                elif cmd == "STOP": # STOP - stops internal loop
                    with state_lock:
                        if loop_running:
                            loop_running = False
                            conn.sendall(b"LOOP STOP\n")
                        else:
                            conn.sendall(b"LOOP NOTRUN\n")

                        if log_running:
                            log_running = False
                            conn.sendall(b"LOG STOP\n")
                        else:
                            conn.sendall(b"LOG NOTRUN\n")

                elif cmd == "EXIT": # ukončí while smyčku a spojení
                    conn.sendall(b"BYE\n")
                    conn.sendall(b'')
                    conn.shutdown(socket.SHUT_WR)
                    conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,struct.pack('ii', 1, 0))                    
                    conn.close()
                    break  

                elif cmd == "SHUTDOWN": # ukončí while smyčku a spojení
                    shutdown_flag = True

                elif cmd == "QR":
                    if not loop_running:
                        print(f"QR:LOOP DID NOT STARTED. CALL 'RUN' BEFORE 'QR' COMMAND")
                        conn.sendall(f"QR:LOOP DID NOT STARTED. CALL 'RUN' BEFORE 'QR' COMMAND\n".encode())
                        conn.sendall(f"\n".encode())
                        continue

                    with qr_lock:
                        if not qr_running:
                            qr_running = True
                            qr_thread = threading.Thread(target=qr_worker, daemon=True)
                            qr_thread.start()

                    print(f"🧾 QR STARTED")

                    # počkej na výsledek nebo timeout
                    deadline = time.time() + 120
                    while (time.time() < deadline and not shutdown_flag ):                    
                        if qr_ready.wait(timeout=2):
                            if qr_result:
                                conn.sendall(f"QR:{qr_result}\n".encode())
                                print(f"🧾 QR FOUND:{qr_result}\n")
                                break
                        
                    if (time.time() < deadline and qr_result is None):
                        conn.sendall("QR:NONE\n".encode())
                        print(f"🧾 QR TIMEOUT\n")

                    conn.shutdown(socket.SHUT_WR)
                    #conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,struct.pack('ii', 1, 0))
                    conn.close()
                    break               

                elif cmd == "LCAM":
                    conn.sendall(b"OK\n")
                elif cmd == "RCAM":
                    conn.sendall(b"OK\n")
                else:
                    conn.sendall(b"ERR\n")
    except Exception as e:
        print(f"❌ Chyba: {e}\n📍 Stack:\n{traceback.format_exc()}")
    finally:
        try:
            #conn.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,struct.pack('ii', 1, 0))                    
            conn.shutdown(socket.SHUT_WR)
            conn.close()
        except:
            pass
        finally:
            pass
        print(f"🔌 Odpojeno: {addr}")

def qr_worker():
    global shutdown_flag, qr_result, qr_ready, qr_lock, qr_running
    global frame_cond, right_buf, frame_seq
    global loop_running

    deadline = time.time() + 120
    qr_result = None
    last_seq = 0

    while time.time() < deadline and not shutdown_flag and loop_running:

        with frame_cond:
            frame_cond.wait_for(lambda: frame_seq > last_seq or not shutdown_flag, timeout=2)
            if frame_seq == last_seq:
                continue
            last_seq = frame_seq
            latest = right_buf[-1]

        codes = pyzbar.decode(cv2.cvtColor(latest, cv2.COLOR_BGR2GRAY))
        print(f"🧾 QR data ... {len(codes)}")

        for code in codes:
            data = code.data.decode("utf-8")
            if data.startswith("geo:"):  # nebo jiný filtr
                qr_result = data
                break

        if qr_result:
            qr_ready.set()
            break

    with qr_lock:
        qr_running = False


def start_server():
    global loop_running, log_running
    global shutdown_flag

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"📷 robot-cameras server naslouchá na {HOST}:{PORT}")

    try:
        while not shutdown_flag:
            server.settimeout(2.0)  # umožní kontrolu shutdown_flag
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            thread = threading.Thread(target=handle_client, args=(conn, addr))
            thread.start()
    except KeyboardInterrupt:
        shutdown_flag = True
        print("\n🧯 Ctrl+C – ukončuji server")
    finally:
        server.close()
        log_running=False
        loop_running=False
        time.sleep(0.1)
        print("🛑 Port uvolněn, server ukončen")


def gst_pipeline(sensor_id: int, w: int = 1640, h: int = 1232, fps: int = 10) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )

def log_loop_thread():
    global log_running, frame_cond, left_buf, right_buf, frame_seq
    global shutdown_flag
    print("📝 Logovací vlákno spuštěno")
    path = "/robot/data/logs/camera"
    os.makedirs(path, exist_ok=True)
    last_seq = 0

    while log_running and not shutdown_flag:
        with frame_cond:
            frame_cond.wait_for(lambda: frame_seq > last_seq or not log_running, timeout=2)

            if frame_seq == last_seq: # timeout bez nového snímku
                continue           
            last_seq = frame_seq # zaznamenat, co jsme už zvedli                             

            left  = left_buf[-1]  # vždy vezmeme NEJNOVĚJŠÍ snímek
            right = right_buf[-1]

        if left is not None and right is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            combined = cv2.hconcat([left, right])
            cv2.imwrite(f"{path}/stereo_{ts}.jpg", combined, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            print(f"💾 Uloženo: stereo_{ts}.jpg")

        time.sleep(3)  # zapisuj foto každých x sekund

    print("🛑 Logovací vlákno ukončeno")


def camera_loop_thread():
    global loop_running, shutdown_flag #, latest_left, latest_right
    global left_buf, right_buf, frame_cond, frame_seq
    prev_id = None

    print("📷 Smyčka kamer spuštěna (2 MP GStreamer)")
    try:
        capL = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)
        capR = cv2.VideoCapture(gst_pipeline(1), cv2.CAP_GSTREAMER)

        if not capL.isOpened() or not capR.isOpened():
            print("❌ Nelze otevřít kamery")
            loop_running = False
            return

        while loop_running and not shutdown_flag:
            t0 = time.time()
            retL, frameL = capL.read()
            t1 = time.time()
            retR, frameR = capR.read()
            t2 = time.time()

            dt_left = (t1 - t0) * 1000  # ms
            dt_right = (t2 - t1) * 1000  # ms
            dt_total = (t2 - t0) * 1000

            if retL and retR:
                frameL = cv2.rotate(frameL, cv2.ROTATE_90_CLOCKWISE)
                frameL = frameL[150:-165, :]
                frameR = cv2.rotate(frameR, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frameR = frameR[150:-165, :]

                same = (id(frameL) == prev_id)
                prev_id = id(frameL)

                print(f"⏱ Kamera L: {dt_left:.1f} ms, R: {dt_right:.1f} ms, Δ celkem: {dt_total:.1f} ms,  🔢 id = {id(frameL):#x}   stejné_jako_předchozí? {same}")

                with frame_cond:                # získá zámek ↓
                    left_buf.append(frameL)     # uloží do bufferu
                    right_buf.append(frameR)
                    frame_seq += 1              # nový snímek → posuň čítač
                    frame_cond.notify_all()     # probudí všechna čekající vlákna

            time.sleep(1.0) #pauza mezi snímky


    except Exception as e:
        print(f"❌ Kamera loop chyba: {e}")
    finally:
        capL.release()
        capR.release()
        print("🛑 Smyčka kamer ukončena")

if __name__ == "__main__":
    start_server()
