import socket

def start_ping_server():
    # Nastavení přesně na localhost
    HOST = "127.0.0.1"
    PORT = 9000

    # Vytvoření klasického TCP socketu
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Povolíme okamžité znovupoužití portu, kdybychom skript restartovali
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"Jetson naslouchá na {HOST}:{PORT} ... Čekám na zprávy z telefonu.")

        while True:
            # Čekáme, dokud se klient (telefon) nepřipojí
            conn, addr = s.accept()
            with conn:
                # Přečteme si data (max 1024 bytů naráz) a dekódujeme je na text
                data = conn.recv(1024).decode('utf-8').strip()
                
                if data:
                    print(f"Přijato z Androidu: '{data}'")
                    
                    if data == "PING":
                        # Spojení je obousměrné, rovnou do něj zapíšeme odpověď
                        response = "PONG HMI\n"
                        conn.sendall(response.encode('utf-8'))
                        print("-> Odesláno: PONG HMI")
                    elif data.startswith("GET") == True:
                        # Spojení je obousměrné, rovnou do něj zapíšeme odpověď
                        response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<h1>Ahoj z robotu!</h1>"
                        conn.sendall(response.encode('utf-8'))
                        print("-> Odesláno: HTTP 200")
                    else:
                        response = "Neznamy prikaz\n"
                        conn.sendall(response.encode('utf-8'))
                        print("-> Odesláno: Neznamy prikaz")

if __name__ == "__main__":
    start_ping_server()