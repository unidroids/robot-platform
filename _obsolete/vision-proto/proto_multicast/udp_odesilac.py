import socket
import time

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 9999

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))

    payload = b"x" * 1024  # 1 KB dat
    print("UDP Multicast Odesílač nastartován. Začínám chrlit data... (Ukonči přes Ctrl+C)")
    
    count = 0
    start_time = time.time()

    try:
        while True:
            sock.sendto(payload, (MCAST_GRP, MCAST_PORT))
            count += 1
            
            if count % 100000 == 0:
                elapsed = time.time() - start_time
                speed = count / elapsed
                print(f"🚀 Odesláno: {count:,} zpráv | Rychlost: {speed:,.0f} msg/s")
    except KeyboardInterrupt:
        print("\nUkončeno uživatelem.")

if __name__ == "__main__":
    main()