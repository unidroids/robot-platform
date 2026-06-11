import socket
import struct

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 5007

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
# Klíčové: Dovolí více programům sdílet tento port
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

sock.bind((MCAST_GRP, MCAST_PORT))

# Přihlášení k multicast skupině na loopback rozhraní (127.0.0.1)
mreq = struct.pack("4s4s", socket.inet_aton(MCAST_GRP), socket.inet_aton("127.0.0.1"))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print("Čekám na data...")
while True:
    print(sock.recv(1024).decode('utf-8'))