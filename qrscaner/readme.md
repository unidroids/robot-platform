
QRScaner port 9022

služba běží na HMI clinet app, tento adresář je jen pro referenci.

PING vrací PONG QRSCANER (pro ověření správné služby)
START - spuštění skenování
STOP - ukončení scaneru
QRCODE - poslední načtený QR Code

Feedback nalezených QR kódů přes ZMQ message "ipc:///tmp/robot-qrscaner" PUB/SUB
frame 1 - message typ, frame 2 - celá naskenovaná message