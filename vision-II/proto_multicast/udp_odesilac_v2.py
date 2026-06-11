import socket
import time

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 9999

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))

    print("Čekám 2 sekundy na připojení přijímačů...")
    time.sleep(2)

    payload = b"x" * 1024  # 1 KB dat
    # TADY JE OPRAVA: Definujeme ukončovací zprávu
    payload_end = b"END" + b"x" * 1021  
    
    TOTAL_MSGS = 20_000
    TARGET_RATE = 1_000  # Cíl: 10 000 zpráv za sekundu
    
    print(f"🚀 UDP Odesílač: Začínám posílat {TOTAL_MSGS:,} zpráv rychlostí {TARGET_RATE:,} msg/s...")
    
    start_time = time.time()

    for i in range(TOTAL_MSGS):
        sock.sendto(payload, (MCAST_GRP, MCAST_PORT))
        
        # Precizní hlídání rychlosti
        expected_elapsed = (i + 1) / TARGET_RATE
        while (time.time() - start_time) < expected_elapsed:
            pass  # Počkáme zlomek mikrosekundy, než nastane správný čas pro další zprávu

    # Ukončovací zpráva (poslaná 5x, kdyby náhodou OS nějakou zahodil)
    for _ in range(5):
        sock.sendto(payload_end, (MCAST_GRP, MCAST_PORT))
    
    elapsed = time.time() - start_time
    print(f"🏁 Odesláno za {elapsed:.2f} s | Výsledná rychlost: {TOTAL_MSGS / elapsed:,.0f} msg/s")

if __name__ == "__main__":
    main()