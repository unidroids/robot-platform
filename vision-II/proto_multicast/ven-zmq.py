import zmq
import time
import random

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    
    # Vydavatel se "binduje" (otevírá komunikační kanál)
    socket.bind("ipc:///tmp/robot_vysilac")
    
    print("Odesílač běží. Posílám data každou sekundu...")
    
    try:
        while True:
            # simulace dat z robota
            x = round(random.uniform(-1.0, 1.0), 2)
            y = round(random.uniform(-1.0, 1.0), 2)
            stav_baterie = random.randint(80, 100)
            
            # 1. Zpráva s tématem "telemetrie"
            zprava_telemetrie = f"telemetrie X:{x} Y:{y}"
            socket.send_string(zprava_telemetrie)
            print(f"Odesláno: {zprava_telemetrie}")
            
            # 2. Zpráva s tématem "system" (přijímač ji ignoruje, pokud neodebírá vše)
            zprava_system = f"system Baterie je na {stav_baterie}%"
            socket.send_string(zprava_system)
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nOdesílač ukončen.")

if __name__ == "__main__":
    main()