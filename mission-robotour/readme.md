# Orchestrátor mise Robotour – Služba MISSION-ROBOTOUR (Port 9031)

Služba **`mission-robotour`** je centrální stavový automat robota Unidroids pro autonomní soutěž Robotour 2025/2026.
Řídí kompletní workflow od prvotní kontroly mikroslužeb, přes naskenování QR kódu s cílovými souřadnicemi, výpočet trasy v mapovém podkladu (MAPS), až po autonomní jízdu s pilotem (PILOT-ROBOTOUR) a reakci na překážky a operátora.

---

## 🔌 Síťové TCP rozhraní (Port 9031)

Služba naslouchá na TCP portu `9031` (výchozí bind `127.0.0.1:9031`).

### Standardní příkazy:
| Příkaz | Odpověď | Popis |
| :--- | :--- | :--- |
| `PING` | `PONG MISSION_ROBOTOUR` | Test dostupnosti a správného názvu služby |
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
| **`LIDAR`** | 9002 | Antikolizní laserový scanner | `PONG LIDAR` |
| **`OOW-BRIDGE`**| 9030 | Bezpečnostní dohled OOW | `PONG OOW` |
| **`PILOT-ROBOTOUR`**| 9104 | Autonomní trasový pilot robota | `PONG PILOT_ROBOTOUR` |

---

## 📋 Přehled fází stavového automatu (Workflow)

1. **Krok 0**: Striktní PING/PONG ověření dostupnosti všech 13 mikroslužeb.
2. **Krok 1**: Spuštění polohových a senzorických služeb (`LOGGER`, `DRIVE`, `GNSS-DUAL`, `GNSS-GPS`, `GNSS-IMU`, `RTK`, `FUSION`).
3. **Krok 2 & 3**: Úvodní obrazovka (*"Robotour - Jdeme na to!"*), otevření nového logu mise do `/data/robot/mission-robotour/<yyyy-mm-dd>/mission-<HH-MM-SS>.dat`.
4. **Krok 4 až 9**: Spuštění QR scanneru (`START`), příjem souřadnic přes ZMQ `ipc:///tmp/robot-qrscaner` (`geo:<lat>,<lon>`), timeout 120s, přehrání zvuku `notification` a vypnutí scanneru (`STOP`).
5. **Krok 10 až 12**: Dotaz na `FUSION DATA`, ověření stavu fixu (`gpsSol != NONE`) a přesnosti (`hAcc < 2000 mm`).
6. **Krok 13 až 16**: Výpočet vzdušné vzdálenosti k cíli. Validace limitu ($< 3\,\text{km}$) a potvrzení cíle uživatelem.
7. **Krok 16.1**: Spuštění podpůrných jízdních služeb `MAPS START` a `LIDAR START`.
8. **Krok 17 až 17.3**: Volání `MAPS FIND_ROUTE <start> <cíl>`, kontrola napojení na komunikaci ($\le 5\,\text{m}$), zobrazení délky trasy.
9. **Krok 18**: Zapnutí motorů `DRIVE ON`, předání JSON trasy do `PILOT-ROBOTOUR START <json>`, zvuk `barking` + oranžové blikání displeje.
10. **Kroky 19 až 23**: Monitorovací smyčka jízdy (perioda 1s):
    - **`RUNNING`**: Zobrazení aktuálního waypointu, vzdálenosti a rychlosti na terminálu, tlačítka *Pause* a *Stop*.
    - **`PAUSED`**: Pozastavení pilota, tlačítka *Pokračovat* (`RESUME`) a *Zrušit misi*.
    - **`STOPPED`**: Zvuk `game-over` + červené blikání, hlášení důvodu, tlačítko *Rozumím* (`acknowledge`).
    - **`FINISHED`**: Zvuk `meow` + zelené blikání (*"Jsme v cíli!"*), tlačítko *Rozumím* (`acknowledge`).
11. **Ukončení jízdy**:
    - Při `acknowledge` nebo zrušení mise: zastaví se jízdní služby (`DRIVE OFF`, `PILOT-ROBOTOUR STOP`, `MAPS STOP`, `LIDAR STOP`), uzavře se log a systém se vrátí do Kroku 2.
    - Při celkovém zastavení služby (`STOP` / `SHUTDOWN` na portu 9031): zastaví se všech 13 spuštěných mikroslužeb.

---

## 🚀 Spuštění a testy

### Spuštění služby:
```bash
python mission-robotour/main.py [--port 9031]
```

### Spuštění testů:
```bash
python -m unittest mission-robotour.test_mission_robotour
```
