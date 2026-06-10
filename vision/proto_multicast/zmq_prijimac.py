import zmq
import time

def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    #socket.setsockopt(zmq.RCVHWM, 50000)
    socket.connect("ipc:///tmp/zmq_vlastni_test")
    
    # Důležité pro masivní tok dat: Nastavíme "non-blocking" režim
    # To znamená, že když je fronta plná, recv() ihned vrátí chybu (try/except)
    # Místo toho, aby se zasekl a čekal na data, která už možná nikdo nechce
    #socket.set_nonblocking(True)
    #socket.setsockopt(zmq.RCVTIMEO, 500)  # Timeout 500ms, aby se test nezasekl donekonečna

    print("📥 ZMQ Přijímač připraven a čeká...")
    
    count = 0
    start_time = None

    while True:
        # KLÍČOVÉ: copy=False zajistí Zero-Copy (extrémní zrychlení v Pythonu)
        msg = socket.recv(copy=False)
        
        # Kontrola konce (pokud zpráva nemá 1024 bajtů, zkontrolujeme, zda je to END)
        if len(msg) != 1024:
            if msg.bytes == b"END":
                break
            
        if not start_time:
            start_time = time.time()
            
        count += 1

    elapsed = time.time() - start_time
    print(f"🏁 Přijato: {count:,} zpráv za {elapsed:.2f} s | Výsledná rychlost: {count / elapsed:,.0f} msg/s")

if __name__ == "__main__":
    main()