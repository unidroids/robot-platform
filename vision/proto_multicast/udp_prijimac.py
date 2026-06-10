import socket
import struct
import time

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 9999

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((MCAST_GRP, MCAST_PORT))
    
    # Přihlášení k multicastu na 127.0.0.1
    mreq = struct.pack("4s4s", socket.inet_aton(MCAST_GRP), socket.inet_aton("127.0.0.1"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print("UDP Multicast Přijímač běží. Čekám na první data...")
    
    count = 0
    start_time = None

    try:
        while True:
            _ = sock.recv(2048)
            
            if not start_time:
                start_time = time.time()
                
            count += 1
            
            if count % 100000 == 0:
                elapsed = time.time() - start_time
                speed = count / elapsed
                print(f"📥 Přijato: {count:,} zpráv | Rychlost: {speed:,.0f} msg/s ({speed * 1024 / 1024 / 1024:.2f} GB/s)")
    except KeyboardInterrupt:
        print("\nUkončeno uživatelem.")

if __name__ == "__main__":
    main()