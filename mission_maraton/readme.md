# Orchestrátor mise Maraton – Služba MISSION-MARATON (Port 9033)

Služba **`mission_maraton`** je centrální stavový automat robota Unidroids pro soutěž Maraton (cíl ujet 5 km za méně než hodinu).
Řídí kompletní workflow od prvotní kontroly mikroslužeb, přes naskenování QR kódu s cílovými souřadnicemi, výpočet trasy v mapovém podkladu (MAPS), až po autonomní jízdu s pilotem (PILOT-MARATON, port 9105) a reakci na překážky a operátora.

---

## 🔌 Síťové TCP rozhraní (Port 9033)

Služba naslouchá na TCP portu `9033` (výchozí bind `127.0.0.1:9033`).

### Standardní příkazy:
| Příkaz | Odpověď | Popis |
| :--- | :--- | :--- |
| `PING` | `PONG MISSION_MARATON` | Test dostupnosti a správného názvu služby |
| `START` | `OK` / `ERROR <reason>` | Zahájení workflow mise |
| `STOP` | `OK` / `ERROR <reason>` | Okamžité zastavení mise a všech spuštěných služeb |
| `STATUS` | `{"service": "...", "step": ..., "state": ..., ...}` | Aktuální stav mise, krok workflow a stav pilota |
| `EXIT` | `BYE` | Uzavření klientského TCP spojení |
| `SHUTDOWN` | `OK SHUTDOWN` | Ukončení celého procesu služby včetně zastavení mikroslužeb |

---

## 🧭 Seznam spravovaných mikroslužeb:
| Služba | Port | Účel | Kontrola (Krok 0) |
| :--- | :---: | :--- | :--- |
| **`QRSCANER`** | 9021 | Kamera pro snímání QR kódů | `PONG QRSCANER` |
| **`TERMINAL`** | 9022 | HMI displej na telefonu (dialogy, tlačítka, zvuky, blikání) | `PONG TERMINAL` |
| **`DRIVE`** | 9003 | Pohon podvozku (příkazy ON / OFF motorů) | `PONG DRIVE` |
| **`GNSS-DUAL`**| 9006 | Duální RTK GNSS jednotka | `PONG GNSS-DUAL` |
| **`GNSS-GPS`** | 9004 | Základní GPS přijímač | `PONG GNSS-GPS` |
| **`RTK`** | 9015 | RTK korekční data (PointPerfect) | `PONG GNSS-RTK` / `PONG RTK` |
| **`GNSS-IMU`** | 9016 | Inerciální měřicí jednotka (IMU) | `PONG GNSS-IMU` |
| **`LOGGER`** | 9012 | Centrální datalogger telemetrie robota | `PONG LOGGER` |
| **`FUSION`** | 9009 | Fúze polohy a orientace (příkaz DATA) | `PONG FUSION` |
| **`MAPS`** | 9040 | Vyhledávání tras v grafu cest (FIND_ROUTE) | `PONG MAPS` |
| **`CAMERA-STEREO`** | 9001 | Stereo kamera (spouštěná čistě pro záznam obrazu a logování) | `PONG CAMERA_STEREO` |
| **`LIDAR`** | 9002 | Nízkoúrovňový ovladač laserového scanneru | `PONG LIDAR` |
| **`LIDAR-COLLISIONS`** | 9222 | Kolizní kurzy a minimální vzdálenosti překážek | `PONG LIDAR-COLLISIONS` |
| **`LIDAR-TRAVERSABILITY`** | 9223 | Potenciálová mapa sjízdnosti terénu (8×8 m) | `PONG LIDAR-TRAVERSABILITY` |
| **`OOW-BRIDGE`**| 9030 | Bezpečnostní dohled OOW | `PONG OOW` |
| **`PILOT-MARATON`**| 9105 | Autonomní pilot robota pro maraton | `PONG PILOT_MARATON` |

---

## 📋 Přehled fází stavového automatu (Workflow)

1. **Krok 0**: Striktní PING/PONG ověření dostupnosti všech 16 mikroslužeb.
2. **Krok 1**: Spuštění polohových a senzorických služeb (`LOGGER`, `DRIVE`, `GNSS-DUAL`, `GNSS-GPS`, `GNSS-IMU`, `RTK`, `FUSION`).
3. **Krok 2 & 3**: Úvodní obrazovka (*"Maraton - Jdeme na to!"*), otevření nového logu mise do `/data/robot/mission_maraton/<yyyy-mm-dd>/mission-<HH-MM-SS>.dat`.
4. **Krok 4 až 9**: Spuštění QR scanneru (`START`), příjem souřadnic přes ZMQ `ipc:///tmp/robot-qrscaner` (`geo:<lat>,<lon>`), timeout 120s, přehrání zvuku `notification` a vypnutí scanneru (`STOP`).
5. **Krok 10 až 12**: Dotaz na `FUSION DATA`, ověření stavu fixu (`gpsSol != NONE`) a přesnosti (`hAcc <= 10000 mm`).
6. **Krok 13 až 16**: Výpočet vzdušné vzdálenosti k cíli, potvrzení cíle uživatelem a spuštění `MAPS START`.
7. **Krok 17 až 17.3**: Volání `MAPS FIND_ROUTE <start> <cíl>`, kontrola napojení na komunikaci, zobrazení délky trasy.
8. **Krok 18**: Spuštění LiDARu a jeho percepčního profilu (`LIDAR START`, `LIDAR-COLLISIONS START`, služba `LIDAR-TRAVERSABILITY` se pro maraton nezapíná), spuštění záznamu ze stereo kamer (`CAMERA-STEREO START` – čistě pro účely logování, neblokující při selhání `START`), zapnutí motorů `DRIVE ON`, předání JSON trasy do `PILOT-MARATON START <json>` (port 9105), zvuk `barking` + oranžové blikání displeje.
9. **Kroky 19 až 23**: Monitorovací smyčka jízdy (perioda 0.5s + okamžité ZMQ notifikace z `ipc:///tmp/robot-pilot-maraton` topic `STATE`):
    - **`RUNNING`**: Zobrazení aktuálního waypointu, vzdálenosti, rychlosti a autonomního podstavu pilota (`nav_state`) na terminálu (vždy se zachováním tlačítek *Pause* a *Stop*):
      - `CRUISING` – *Maraton - Jízda*
      - `OBSTACLE` – *Maraton - Překážka!* (zpomalování / zastavování před 3D překážkou)
      - `FOLLOWING` – *Maraton - Sledování chodce* (plynulé srovnání rychlosti za chodcem)
      - `WAITING_CLEARANCE` – *Maraton - Čeká na uvolnění* (čekání 5 s až 60 s u statické/pohybující se překážky, doprovázené tichým upozorněním `SOUND notification` + oranžové `BLINK`)
      - `AVOIDING` – *Maraton - Objíždění překážky* (včetně směru a velikosti vybočení v koridoru)
      - `OVERTAKING` – *Maraton - Předjíždění* (předjíždění pomalejšího chodce)
      - `NO_PATH` – *Maraton - Není průjezdná cesta!* (čekání na uvolnění zablokovaného koridoru, klidné opakování zvukového upozornění `SOUND notification` jednou za `60 s`)
    - **`PAUSED`**: Pozastavení pilota, tlačítka *Pokračovat* (`RESUME`) a *Stop*.
    - **`STOPPED`**: Zvuk `game-over` + červené blikání, hlášení důvodu, tlačítko *Rozumím* (`acknowledge`).
    - **`FINISHED`**: Zvuk `meow` + zelené blikání (*"Jsme v cíli!"*), tlačítko *Rozumím* (`acknowledge`).
10. **Ukončení jízdy**:
    - Ihned po `STOPPED`/`FINISHED` (a rovněž při `acknowledge` nebo zrušení mise): zastaví se jízdní a záznamové služby etapy (`PILOT-MARATON STOP`, `LIDAR-COLLISIONS STOP`, `LIDAR STOP`, `CAMERA-STEREO STOP`, `MAPS STOP` a po potvrzení `DRIVE OFF`), uzavře se log a systém se vrátí do Kroku 2.
    - Při celkovém zastavení služby (`STOP` / `SHUTDOWN` na portu 9033): zastaví se všechny spuštěné mikroslužby.

---

## 🚀 Spuštění a testy

### Spuštění služby:
```bash
python mission_maraton/main.py [--port 9033]
```

### Spuštění testů:
```bash
python -m unittest mission_maraton.test_mission_maraton
```
