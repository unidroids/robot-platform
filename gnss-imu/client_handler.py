# client_handler.py
import socket
import os
import signal

def client_thread(sock: socket.socket, addr, service):
    """
    Obsluha jednoho TCP klienta připojeného na řídicí port služby (9016).
    Podporované příkazy: PING, START, STOP, STATUS, EXIT, SHUTDOWN.
    """
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8', errors='ignore').strip()
            print(f"[SERVER] Client {addr} command: {line}")

            try:
                if line == "PING":
                    f.write(b'PONG GNSS-IMU\n')
                elif line == "START":
                    res = service.start()
                    f.write((res + '\n').encode('utf-8'))
                elif line == "STOP":
                    res = service.stop()
                    f.write((res + '\n').encode('utf-8'))
                elif line == "STATUS":
                    res = service.get_status()
                    f.write((res + '\n').encode('utf-8'))
                elif line == "EXIT":
                    f.write(b'IMU-BYE\n')
                    f.flush()
                    break
                elif line == "SHUTDOWN":
                    f.write(b'IMU-SHUTDOWN\n')
                    f.flush()
                    os.kill(os.getpid(), signal.SIGINT)
                    break
                else:
                    f.write(b'ERR UNKNOWN COMMAND\n')
                f.flush()
            except Exception as e:
                print(f"[CLIENT ERROR] {e}")
                f.write(f"ERROR: {e}\n".encode('utf-8'))
                f.flush()
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
