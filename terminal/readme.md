
Terminal port 9021

služba běží na HMI clinet app, tento adresář je jen pro referenci.

PING vrací PONG TERMINAL (pro ověření správné služby)
BLINK <color #rgb> <frekvence Hz> <duration ms> - spuštění vizuální upozornění
SOUND <name> - přehraje zvuk (barking, notification, game-over) list je v /opt/projects/robotour/hmi-bridge/sounds
MESSAGE <json>
sample_msg_A = '{"header":"Varování","text":"Překážka","buttons":[{"id":"btn_1","text":"OK"}]}'

sample_msg_B = '{"header":"Robotour","text":"Jdeme na to!","buttons":[{"id":"scan_qrcode","text":"Scan QR Code"}]}'

sample_msg_C = '{"header":"Robotour - Potvrzení destinace","text":"Cílové souřadnice jsou geo:12.233,45.345. Vzdušná vzdálenost do cíle je 300m.","buttons":[{"id":"repeat_qrscan","text":"Opakovat QR Scan"}, {"id":"destination_ok","text":"Ano, jedeme"},{"id":"cancel","text":"Ne, zrušit"}]}'


Feedback tlačítek přichází přes ZMQ kanál "ipc:///tmp/robot-terminal" PUB/SUB 
frame 1 "button", frame 2 <button_id podle definice ze message>

Ukázka "ipc:///tmp/robot-terminal"
['button', 'destination_ok']
['button', 'cancel']
['button', 'repeat_qrscan']
