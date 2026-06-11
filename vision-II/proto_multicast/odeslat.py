import socket

MCAST_GRP = '239.0.0.1'
MCAST_PORT = 5007

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

# Klíčové: Povolí loopback pro multicast (aby to slyšel i tvůj vlastní PC)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
# Nastavíme odchozí rozhraní na loopback
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))

sock.sendto(b"Ahoj vsem prijimacum!", (MCAST_GRP, MCAST_PORT))
print("Zpráva odeslána.")