Pojďme napsat novou TCP službu "mission-robotour". Služba by měla mít stejné logické členění jako má oow-bridge. 
Nicméně služba misson-robotour je z principu věci stavovým automatem, který postupně spouští a zastavuje microslužby robota pro dosažení mise. 

Robotour se dá rozdělit do tří částí. 

První je naskenování QR kódu, druhá nalezení cesty pomocí OSM a třetí je spuštění pilota, který s robotem jede k cíli po cestě dané waypointy. 

Služba je TCP na portu 9031. Příkazy:
PING odpověď PONG 
START zahájí kroky mise OK/ERROR <reason>
STOP ukončí misi, zastavení spuštěných microslužeb OK/ERROR <reason>
STATUS aktuální step a jeho stav/progress
EXIT ukončí spojení s klientem
SHUTDOWN ukončí program služby (vysláním signálu)

Služba ošetří systémové signály a řádné zastavení spuštěných microslužeb


Microslužby: 
QRSCANER 9021
TERMINAL 9022
LIDAR 9002
DRIVE 9003
GNSS-DUAL 9006
RTK 9015
COMPASS 9014
LOGGER 9012
FUSION 9009 
OOW-BRIDGE 9030


Workflow:
0. Kontrola potřebných microslužeb. ( PING na službu vrací PONG <nazev služby> )
0a1 - Chyba spojení/názvu - zobrazení zprávy na terminálu Nadpis: Chyba komunikace, Text: popis která služby nejsou dostupné nebo mají chybný název, buttons: Zkusit znovu [try_again].
0a2 - zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'try_again'] - přechod na 0.
0b - vše OK přechod na krok 1.
1. Start služeb "LOGGER", "GNSS-DUAL", "COMPASS", "RTK"
1a1. Nějaká služba neodpoví "OK ..." zobrazit MESSAGE přes (Terminál) Nadpis: Chyba při startu služby, Text: služby které neodpověděy OK .. a výpis odpovědí, buttons: Zkusit znovu [try_again]. 
1a2. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'try_again'] - přechod na 1 pro nenastarvované služby.

2. úvodní dialogue (MESSAGE). Nadpis: Robotour, Text: Jdeme na to!, button: Scan QR Code [scan_qrcode]
3. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'scan_qrcode']
4. Spustění QR Scaneru. QRSCANER (9021) START
5. Čeká až 120 s na qrcode s gps souřadnicemi přes zmq "ipc:///tmp/robot-qrcode" ['geo:', 'geo:<lat>,<lon>'] 
6. Přijetí zprávy do 120 s - ne - zavolání QRSCANER STOP - zobrazení zprávy Nadpis: Robotour - Nenačteny cílové souřadnice, Text: Během psledních dvou minut nebyl zaznamenán QR Code., buttons: Rescan QR Code [rescan_qrcode]
6a. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'rescan_qrcode'] - přechod na 4. (Spuštění QR Scaneru)
7. Přijetí zprávy do 120 s - ano - ověření QR Kódu - geo:<lat>,<lon> 
8. Formát not OK - přechod na 5.
9. Formát OK - zavolání QRSCANER STOP 
10. Zavolání služby FUSION příkaz DATA 
11. Vyhodnocení připravenosti GPS Data (gpsSol != None a hAcc < 2m)
12. Vyhodnocení připravenosti GPS Dat - NE - zobrazení zprávy Title: Robotour - Čekání na polohu, Text: Poloha robotba nebyla vyhodnocena. Aktuální stav řešení polohy je <gpsSol>, přesost polohy je <hAcc>. Gps poloha je <lat>, <lon>. Button: Zrušit misi [cancel_mission] - wait 1s - přechod na 10.
13. Vyhodnocení připravenosti GPS Dat - ANO - Výpočet vzdálenosti vzdušnou čarou k cíli (souřadnice QR code a GPS poloha)
14. Vzdálenost k cíli je rovna nebo větší než 3km - Zobrazit zprávu Title: Robotour - Cíl je příliš daleko, Title: Cílové souřadnice jsou <geo: ...>. Vzdušná vzdálenost k cíli je <vzdálenost> m a je mimo parametry soutěže Robotour. Button: Re-Scan QR Code [rescan_qrcode] - čeká na zmq zprávu "ipc:///tmp/robot-terminal" ['button', 'rescan_qrcode'] - přechod na 4. (Spuštění QR Scaneru)
15. Vzdálenost k cíli je menší než 3km - Zobrazit zprávu Title: Robotour - Potvrzení cíle, Title: Cílové souřadnice jsou <geo: ...>. Vzdušná vzdálenost k cíli je <vzdálenost> m. Button: Re-Scan QR Code [rescan_qrcode], Zrušit misi [cancel_mission], Clí je OK [destination_ok]
16. čeká na zmq zprávu "ipc:///tmp/robot-terminal" :
- ['button', 'rescan_qrcode'] - přechod na 4. (Spuštění QR Scaneru) 
- ['button', 'cancel_mission'] - přechod na 2. (Úvodní obrazovka) 
- ['button', 'destination_ok'] - přechod na 17. (Nalezení cesty pomocí OSM)
17. TBD: workflow - je předmětem další specifikace

