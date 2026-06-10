import zmq

def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    
    # Připojíme se k vysílači
    socket.connect("ipc:///tmp/robot_vysilac")
    
    # PŘIHLÁŠENÍ K ODBĚRU:
    # Chceme odebírat pouze zprávy, které začínají slovem "telemetrie"
    socket.setsockopt_string(zmq.SUBSCRIBE, "telemetrie")
    
    # Pokud bys chtěl odebírat ÚPLNĚ VŠE, odkomentuj tento řádek:
    # socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print("Přijímač běží a čeká na data z tématu 'telemetrie'...")
    
    try:
        while True:
            # ZMQ přijme zprávu jako string
            zprava = socket.recv_string()
            print(f"Přijato: {zprava}")
    except KeyboardInterrupt:
        print("\nPřijímač ukončen.")

if __name__ == "__main__":
    main()