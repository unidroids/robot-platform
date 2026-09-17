Pojďme napsat novou TCP službu "mission-robotour". 
Nicméně služba misson-robotour je z principu věci stavovým automatem, který postupně spouští a zastavuje microslužby robota pro dosažení mise. 

Robotour se dá rozdělit do tří částí. 

První je naskenování QR kódu,  
druhá nalezení cesty pomocí MAPS a  
třetí je spuštění pilota, který s robotem jede k cíli po cestě dané waypointy. 

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
DRIVE 9003
GNSS-DUAL 9006
GNSS-GPS 9004
RTK 9015
GNSS-IMU 9016
LOGGER 9012
FUSION 9009
MAPS 9040
LIDAR 9002
OOW-BRIDGE 9030
PILOT-ROBOTOUR 9104



Workflow:
0. Kontrola potřebných microslužeb. ( PING na službu vrací PONG <nazev služby> )
0a1 - Chyba spojení/názvu - zobrazení zprávy na terminálu Nadpis: Chyba komunikace, Text: popis která služby nejsou dostupné nebo mají chybný název, buttons: Zkusit znovu [try_again].
0a2 - zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'try_again'] - přechod na 0.
0b - vše OK přechod na krok 1.
1. Start služeb "LOGGER", "DRIVE", "GNSS-DUAL", "GNSS-GPS", "GNSS-IMU", "RTK"
1a1. Nějaká služba neodpoví "OK ..." zobrazit MESSAGE přes (Terminál) Nadpis: Chyba při startu služby, Text: služby které neodpověděy OK .. a výpis odpovědí, buttons: Zkusit znovu [try_again]. 
1a2. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'try_again'] - přechod na 1 pro nenastarvované služby.

2. úvodní dialogue (MESSAGE). Nadpis: Robotour, Text: Jdeme na to!, button: Scan QR Code [scan_qrcode]
3. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'scan_qrcode']
4. Spustění QR Scaneru. QRSCANER (9021) START
5. Čeká až 120 s na qrcode s gps souřadnicemi přes zmq "ipc:///tmp/robot-qrscaner" ['geo:', 'geo:<lat>,<lon>'] 
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

16.1 Start služby MAPS (9040), LIDAR (9002) příkaz START
16.1.1 Nějaká služba neodpoví OK .. zobrazení zprávy Nadpis: Robotour - Chyba při startu služby, Text: Některá služba neodpověděl OK .. a výpis odpovědí, buttons: Zkusit znovu [try_again], Zrušit misi [cancel_mission]
16.1.2 zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'try_again'] - přechod na 16.1 pro nenastarvované služby.
16.1.3 zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'cancel_mission'] - přechod na 2. (Úvodní obrazovka)

17. Zobrazit zprávu Nadpis: Robotour, Text: Hledáme cestu k cíli., buttons: Zrušit misi [cancel_mission]
18. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'cancel_mission'] - přechod na 2. (Úvodní obrazovka)
17.1. Volání služby MAPS s příkazem: "FIND_ROUTE <start_lat>, <start_lon>, <cil_lat>, <cil_lon>" (    Flexibilní parsování argumentů FIND_ROUTE.
    Podporuje jak čárkami, tak mezerami oddělená čísla:
    'FIND_ROUTE 49.5541, 12.7411, 49.5545, 12.7424'
    'FIND_ROUTE 49.5541 12.7411 49.5545 12.7424'
    'FIND_ROUTE 49.5541,12.7411,49.5545,12.7424'
    """)
(Formát odpovědi: viz `maps\readme.md`)

17.2. MAPS odpoví cesta nenalezena "cesta nenalezena" - Zobrazit zprávu Nadpis: Robotour - Cesta nebyla nalezena, Text: <metadata reason> buttons: Re-Scan QR Code [rescan_qrcode], Zrušit misi[cancel_mission]
17.2a
    Re-Scan QR Code [rescan_qrcode] - přechod na 4. (Spuštění QR Scaneru)
    Cancel [cancel_mission],  - přechod na 2. (Úvodní obrazovka)
17.3. MAPS odpoví "cesta nalezena" - Zobrazit zprávu Nadpis: Robotour - Cesta nalezena, Text: Vzdálenost k cíli <viz metadata> , buttons: Re-Scan QR Code [rescan_qrcode], Zrušit misi[cancel_mission], Vydat se na cestu [mission_go]
17.3a. zmq zpráva "ipc:///tmp/robot-terminal" :
- ['button', 'rescan_qrcode'] - přechod na 4. (Spuštění QR Scaneru)
- ['button', 'cancel_mission'] - přechod na 2. (Úvodní obrazovka)
- ['button', 'mission_go'] - přechod na 18. (Spuštění robota)
18. Spuštění služby PILOT-ROBOTOUR příkaz START {json} formát odpovídá specifikaci služby MAPS.
19. Volání služby STATUS služby PILOT-ROBOTOUR pro zjištění stavu robota 
20. PILOT-ROBOTOUR odpoví STOPPED - zobrazení zprávy Nadpis: Robotour - Ukončení jízdy, Text: <reason>, buttons: Rozumím [acknowledge],
20a. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'acknowledge'] - přechod na 2. (Úvodní obrazovka)
20.1 PILOT-ROBOTOUR odpoví FINISHED - zobrazení zprávy Nadpis: Robotour - Jsme v cíli!, Text: Gratukujeme, dokončili jste misi!, buttons: Rozumím [acknowledge],
20.1a. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'acknowledge'] - přechod na 2. (Úvodní obrazovka)

21. PILOT-ROBOTOUR neodpoví STOPPED Zobrazení zprávy Nadpis: Robotour - Jízda, Text: <status>, buttons: Pause [pause_mission],  Stop [stop_mission]
21a. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'stop_mission'] - ukončení pilot-robotour (STOP) a přechod na 2. (Úvodní obrazovka)
21b. zmq zpráva "ipc:///tmp/robot-terminal" ['button', 'pause_mission'] - pozastavení robota příkazem PAUSE pro pilot-robotour. Přechod na 22.
22. Zobrazení zprávy Nadpis: Robotour - Pozastaveno, Text: Robot je pozastaven. Čekáme na vstup uživatele. buttons: Pokračovat [resume_mission], Zrušit misi [cancel_mission]
22a. zmq zpráva "ipc:///tmp/robot-terminal" :
- ['button', 'resume_mission'] - pokračování jízdy (RESUME pro pilot-robotour). Přechod na 19.
- ['button', 'cancel_mission'] - ukončení pilot-robotour (STOP) a přechod na 2. (Úvodní obrazovka)
23. počkat 2s a přechod na 19 (zavolat status).




