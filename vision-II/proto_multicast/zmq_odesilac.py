import zmq
import time

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    #socket.setsockopt(zmq.SNDHWM, 50000)
    socket.bind("ipc:///tmp/zmq_vlastni_test")

    print("Čekám 2 sekundy na připojení přijímačů...")
    time.sleep(2)

    payload = b"x" * 1024  # 1 KB dat
    TOTAL_MSGS = 20_000
    TARGET_RATE = 1_000  # Cíl: 100 000 zpráv za sekundu
    
    print(f"🚀 ZMQ Odesílač: Začínám posílat {TOTAL_MSGS:,} zpráv rychlostí {TARGET_RATE:,} msg/s...")
    
    start_time = time.time()

    for i in range(TOTAL_MSGS):
        socket.send(payload)
        
        # Precizní hlídání rychlosti
        expected_elapsed = (i + 1) / TARGET_RATE
        while (time.time() - start_time) < expected_elapsed:
            pass  # Počkáme zlomek mikrosekundy, než nastane správný čas pro další zprávu

    # Ukončovací zpráva
    socket.send(b"END")
    
    elapsed = time.time() - start_time
    print(f"🏁 Odesláno za {elapsed:.2f} s | Výsledná rychlost: {TOTAL_MSGS / elapsed:,.0f} msg/s")

if __name__ == "__main__":
    main()