import multiprocessing
import socket
import struct
import time
import os
import zmq

# KONFIGURACE TESTU
DURATION = 3.0          # Jak dlouho se budou chrlit data (sekundy)
MSG_SIZE = 1024         # Velikost jedné zprávy v bajtech (1 KB)
PAYLOAD = b"x" * MSG_SIZE

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 9999
ZMQ_ADDR = "ipc:///tmp/zmq_perf_test"

# --- UDP MULTICAST SEKCÉ ---
def udp_receiver(pipe):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((MCAST_GRP, MCAST_PORT))
    
    mreq = struct.pack("4s4s", socket.inet_aton(MCAST_GRP), socket.inet_aton("127.0.0.1"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(1.0) # Timeout pro ukončení testu
    
    pipe.send("READY") # Signalizace odesílači
    
    count = 0
    start_time = None
    
    try:
        while True:
            data, _ = sock.recvfrom(MSG_SIZE + 100)
            if not start_time:
                start_time = time.time()
            count += 1
    except socket.timeout:
        pass # Test skončil, odesílač přestal posílat
    
    end_time = time.time() - 1.0 # Odečteme sekundu timeoutu
    actual_duration = end_time - start_time if start_time else 0
    pipe.send((count, actual_duration))

def udp_sender(pipe):
    pipe.recv() # Počkáme, až přijímač nastartuje
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
    
    count = 0
    end_time = time.time() + DURATION
    while time.time() < end_time:
        sock.sendto(PAYLOAD, (MCAST_GRP, MCAST_PORT))
        count += 1
    pipe.send(count)

# --- ZERO MQ SEKCÉ ---
def zmq_receiver(pipe):
    context = zmq.Context()
    sock = context.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.setsockopt(zmq.RCVTIMEO, 1000) # Timeout 1 sekunda
    sock.connect(ZMQ_ADDR)
    
    pipe.send("READY")
    
    count = 0
    start_time = None
    
    try:
        while True:
            data = sock.recv()
            if not start_time:
                start_time = time.time()
            count += 1
    except zmq.error.Again:
        pass # Timeout
        
    end_time = time.time() - 1.0
    actual_duration = end_time - start_time if start_time else 0
    pipe.send((count, actual_duration))

def zmq_sender(pipe):
    context = zmq.Context()
    sock = context.socket(zmq.PUB)
    sock.bind(ZMQ_ADDR)
    
    pipe.recv() # Počkáme na přijímač
    time.sleep(0.2) # ZMQ slow-joiner ochrana
    
    count = 0
    end_time = time.time() + DURATION
    while time.time() < end_time:
        sock.send(PAYLOAD)
        count += 1
    pipe.send(count)

# --- MAIN RUNNER ---
if __name__ == "__main__":
    print(f"=== Spouštím benchmark na jednom hostu ===")
    print(f"Velikost zprávy: {MSG_SIZE} bajtů | Doba generování: {DURATION}s\n")
    
    # 1. TEST: UDP Multicast
    p1, c1 = multiprocessing.Pipe()
    p2, c2 = multiprocessing.Pipe()
    
    r_proc = multiprocessing.Process(target=udp_receiver, args=(c1,))
    s_proc = multiprocessing.Process(target=udp_sender, args=(c2,))
    
    r_proc.start()
    s_proc.start()
    
    sent_udp = p2.recv()
    recv_udp, dur_udp = p1.recv()
    
    r_proc.join()
    s_proc.join()
    
    # 2. TEST: ZeroMQ IPC
    p3, c3 = multiprocessing.Pipe()
    p4, c4 = multiprocessing.Pipe()
    
    r_proc_zmq = multiprocessing.Process(target=zmq_receiver, args=(c3,))
    s_proc_zmq = multiprocessing.Process(target=zmq_sender, args=(c4,))
    
    r_proc_zmq.start()
    s_proc_zmq.start()
    
    sent_zmq = p4.recv()
    recv_zmq, dur_zmq = p3.recv()
    
    r_proc_zmq.join()
    s_proc_zmq.join()
    
    # Úklid IPC souboru
    if os.path.exists("/tmp/zmq_perf_test"):
        os.remove("/tmp/zmq_perf_test")

    # VÝSLEDKY
    print(f"📊 VÝSLEDKY UDP MULTICAST (Loopback):")
    print(f"  Odesláno zpráv: {sent_udp:,}")
    print(f"  Přijato zpráv:  {recv_udp:,} (Ztrátovost: {round((1 - recv_udp/sent_udp)*100, 2)} %)")
    if dur_udp > 0:
        print(f"  Rychlost:       {round(recv_udp / dur_udp):,} msg/s")
        print(f"  Datový tok:     {round((recv_udp * MSG_SIZE) / dur_udp / 1024 / 1024, 2)} MB/s")
        
    print(f"\n📊 VÝSLEDKY ZEROMQ (IPC):")
    print(f"  Odesláno zpráv: {sent_zmq:,}")
    print(f"  Přijato zpráv:  {recv_zmq:,} (Ztrátovost: {round((1 - recv_zmq/sent_zmq)*100, 2)} %)")
    if dur_zmq > 0:
        print(f"  Rychlost:       {round(recv_zmq / dur_zmq):,} msg/s")
        print(f"  Datový tok:     {round((recv_zmq * MSG_SIZE) / dur_zmq / 1024 / 1024, 2)} MB/s")