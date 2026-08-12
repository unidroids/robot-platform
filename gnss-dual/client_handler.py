import socket

def client_thread(sock, addr, service):
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
                    f.write(b'PONG GNSS-DUAL\n')
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
                    f.write(b'GNSS-DUAL-BYE\n')
                    f.flush()
                    break
                elif line == "SHUTDOWN":
                    f.write(b'GNSS-DUAL-SHUTDOWN\n')
                    f.flush()
                    import os, signal
                    os.kill(os.getpid(), signal.SIGINT)
                    break
                else:
                    f.write(b'ERR UNKNOWN COMMAND\n')
                f.flush()
            except Exception as e:
                print(f"[CLIENT ERROR] {e}")
                f.write(f"ERROR: {e}\n".encode())
                f.flush()
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
    finally:
        try:
            sock.close()
        except:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
