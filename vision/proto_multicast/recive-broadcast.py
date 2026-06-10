import socket

PORT = 5008

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Povolíme sdílení portu pro více programů
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bindujeme na '' (což je 0.0.0.0 - poslouchej na všech rozhraních včetně loopbacku)
sock.bind(('', PORT))

print(f"Přijímač běží na portu {PORT} a čeká na broadcast...")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Přijato od {addr}: {data.decode('utf-8')}")