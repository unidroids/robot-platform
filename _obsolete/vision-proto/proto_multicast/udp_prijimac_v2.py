import socket
import struct
import time

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 9999

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((MCAST_GRP, MCAST_PORT))
    

    mreq = struct.pack("4s4s", socket.inet_aton(MCAST_GRP), socket.inet_aton("127.0.0.1"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    # Timeout 2 sekundy: Pokud odesílač utichne a nepřijde END, test ukončíme
    #sock.settimeout(2.0)

    print("📥 UDP Přijímač připraven a čeká...")
    
    count = 0
    start_time = None
    
    # KLÍČOVÉ: Pre-alokovaný buffer (Python zero-copy ekvivalent)
    buf = bytearray(1024)
    view = memoryview(buf)

    try:
        while True:
            # Čteme data přímo do existujícího bufferu v paměti
            _ = sock.recv_into(view)
            
            if not start_time:
                start_time = time.time()
            
            # Kontrola začátku zprávy v bufferu
            if view[:3] == b"END":
                break
                
            count += 1
            
    except socket.timeout:
        print("\n⚠️ Test ukončen timeoutem (všechny END pakety byly pravděpodobně zahozeny OS bufferem kvůli přetížení).")

    elapsed = time.time() - start_time if start_time else 0
    if elapsed > 0:
        print(f"🏁 Přijato: {count:,} zpráv za {elapsed:.2f} s | Výsledná rychlost: {count / elapsed:,.0f} msg/s")
        lost = 20_000 - count
        print(f"📉 Ztracených zpráv: {max(0, lost):,} ({max(0, lost)/20_000*100:.2f} %)")

if __name__ == "__main__":
    main()