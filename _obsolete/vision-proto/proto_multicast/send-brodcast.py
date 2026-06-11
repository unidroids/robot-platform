import socket

BROADCAST_ADDR = '127.255.255.255'
PORT = 5008

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# KLÍČOVÉ: Musíme OS říct, že máme povoleno vysílat broadcast
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

sock.sendto(b"Ahoj vsichni na tomto PC!", (BROADCAST_ADDR, PORT))
print("Broadcast odeslán.")