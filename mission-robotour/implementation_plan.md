# Implementační plán: Služba MISSION-ROBOTOUR (Port 9031)

Tento plán detailně popisuje implementaci orchestrátoru **`mission-robotour`**, který řídí autonomní misi soutěže Robotour 2025/2026. Součástí je také úprava služeb **`FUSION`** (přidání standardních příkazů `START`/`STOP`) a **`PILOT-ROBOTOUR`** (přijetí JSON trasy z MAPS, ukládání do `/data/robot/pilot_robotour/<date>/<time>/route.json` a flat JSON výstup pro `STATUS`).

---

## 1. Související úpravy existujících služeb

### A. Služba FUSION ([fusion/](file:///e:/robotour/robot-platform/fusion))
- **`fusion/client.py` & `fusion/service.py`**:
  - Doplnit standardní příkazy `START` a `STOP`.
  - Příkaz `START`: spustí fúzi (pokud neběží), nebo provede reset/restart interního stavu (reset starých dat) a vrátí `OK`.
  - Příkaz `STOP`: zastaví zpracování a vrátí `OK`.
  - Zachovat příkaz `DATA`, který vrací aktuální `NavFusionData` v JSON formátu (`hAcc` v mm).

### B. Služba PILOT-ROBOTOUR ([pilot_robotour/](file:///e:/robotour/robot-platform/pilot_robotour))
- **`pilot_robotour/main.py` & `pilot_robotour/service.py`**:
  - **Příkaz `START`**:
    - Umožnit spuštění buď ve tvaru `START <json_trasy>`, nebo klasicky `START [speed] [pwm]`.
    - Při předání JSONu trasy (výstup z `MAPS FIND_ROUTE`):
      1. Uložit soubor trasy do `/data/robot/pilot_robotour/<yyyy-mm-dd>/<HH-MM-SS>/route.json` (podle aktuálního času spuštění).
      2. Načíst uzly trasy (`nodes` s `lat`, `lon`), inicializovat `PathTracker` přímo s těmito body.
  - **Příkaz `STATUS`**:
    - Nahradit textový výstup s mezerami za čistý flat JSON string:
      ```json
      {
        "state": "RUNNING",
        "source": "PILOT",
        "info": "",
        "wp_index": 2,
        "wp_total": 24,
        "distance_to_goal_m": 125.4,
        "lat": 49.55412,
        "lon": 12.74115,
        "heading": 134.2,
        "speed": 0.8,
        "gps_sol": "FIX",
        "h_acc_mm": 180
      }
      ```
      Pro stavy `IDLE`, `PAUSED`, `STOPPED`, `FINISHED`:
      ```json
      {
        "state": "FINISHED",
        "source": "PILOT",
        "info": "Goal reached",
        "wp_index": 24,
        "wp_total": 24,
        "distance_to_goal_m": 0.0
      }
      ```

---

## 2. Architektura služby MISSION-ROBOTOUR ([mission-robotour/](file:///e:/robotour/robot-platform/mission-robotour))

Služba poběží na **TCP portu 9031** a bude řízena jako asynchronní stavový automat.

```
e:\robotour\robot-platform\mission-robotour\
├── main.py                     # Vstupní bod, TCP server (port 9031), obsluha signálů SIGINT/SIGTERM
├── service.py                  # Stavový automat mise Robotour (kroky 0 až 23)
├── terminal_client.py          # Odesílání MESSAGE, SOUND, BLINK na TERMINAL (port 9022)
├── microservices.py            # TCP klient pro mikroslužby (PING, START, STOP, ON, OFF, STATUS)
├── test_mission_robotour.py    # Unit a integrační testy stavového automatu a komunikace
└── readme.md                   # Dokumentace protokolu a kroků workflow
```

### Seznam mikroslužeb a portů:
- `QRSCANER`: `9021` (`START`, `STOP`, `PING`)
- `TERMINAL`: `9022` (`MESSAGE`, `SOUND`, `BLINK`, `PING`)
- `DRIVE`: `9003` (`ON`, `OFF`, `STATUS`, `PING`)
- `GNSS-DUAL`: `9006` (`START`, `STOP`, `PING`)
- `GNSS-GPS`: `9004` (`START`, `STOP`, `PING`)
- `RTK`: `9015` (`START`, `STOP`, `PING`)
- `GNSS-IMU`: `9016` (`START`, `STOP`, `PING`)
- `LOGGER`: `9012` (`START`, `STOP`, `PING`)
- `FUSION`: `9009` (`START`, `STOP`, `DATA`, `PING`)
- `MAPS`: `9040` (`START`, `STOP`, `FIND_ROUTE`, `PING`)
- `LIDAR`: `9002` (`START`, `STOP`, `PING`)
- `OOW-BRIDGE`: `9030` (`PING` - běží trvale)
- `PILOT-ROBOTOUR`: `9104` (`START <json>`, `STOP`, `PAUSE`, `RESUME`, `STATUS`, `PING`)

---

## 3. Detailní implementace kroků Workflow

1. **Krok 0: Kontrola mikroslužeb**
   - Pošle `PING` na všech 13 mikroslužeb.
   - Pokud některá neodpoví `PONG <nazev>`: Zobrazí na terminálu chybovou zprávu `"Chyba komunikace"`, tlačítko `try_again`. Po kliknutí opakuje krok 0.
2. **Krok 1: Start polohových a senzorických služeb**
   - Pošle `START` pro `LOGGER`, `GNSS-DUAL`, `GNSS-GPS`, `GNSS-IMU`, `RTK`, `FUSION`.
   - Pokud některá neodpoví `OK`: Zobrazí `"Chyba při startu služby"`, tlačítko `try_again`.
3. **Krok 2 & 3: Úvodní obrazovka**
   - Zobrazí dialog: Header: `"Robotour"`, Text: `"Jdeme na to!"`, Tlačítko: `scan_qrcode` (`"Scan QR Code"`).
   - Čeká na ZMQ `ipc:///tmp/robot-terminal` `['button', 'scan_qrcode']`.
4. **Krok 4 až 9: Skenování QR kódu**
   - Spustí `QRSCANER START` (port 9021).
   - Čeká až 120s na ZMQ `ipc:///tmp/robot-qrscaner` `['geo:', 'geo:<lat>,<lon>']`.
   - Při timeoutu: `QRSCANER STOP`, zobrazí `"Nenačteny cílové souřadnice"`, tlačítko `rescan_qrcode`.
   - Při načtení a validaci: `QRSCANER STOP`, přehraje `SOUND notification`.
5. **Krok 10 až 12: Ověření GPS fixu a přesnosti**
   - Zavolá `FUSION DATA`.
   - Kontroluje `gpsSol is not None` a `gpsSol != 'NONE'` a `hAcc < 2000` (mm).
   - Pokud NE: Zobrazí `"Čekání na polohu"` s aktuálním `gpsSol`, `hAcc` a tlačítkem `cancel_mission`. Čeká 1s a opakuje dotaz.
6. **Krok 13 až 16: Výpočet vzdálenosti k cíli**
   - Spočte vzdušnou vzdálenost mezi polohou robota a cílem z QR kódu.
   - Pokud $\ge 3000\,\text{m}$: Zobrazí `"Cíl je příliš daleko"`, tlačítko `rescan_qrcode`.
   - Pokud $< 3000\,\text{m}$: Zobrazí `"Potvrzení cíle"` s uvedením vzdálenosti v metrech a tlačítky `rescan_qrcode`, `cancel_mission`, `destination_ok`.
7. **Krok 16.1: Start jízdních podpůrných služeb**
   - Po kliknutí na `destination_ok`: Spustí `MAPS START` a `LIDAR START`.
8. **Krok 17 až 17.3: Hledání trasy v MAPS**
   - Zobrazí `"Hledáme cestu k cíli."` s tlačítkem `cancel_mission`.
   - Pošle do MAPS: `FIND_ROUTE <start_lat>, <start_lon>, <cil_lat>, <cil_lon>`.
   - Pokud `search_result != "found"`: Zobrazí `"Cesta nebyla nalezena"` (s textem reason z metadat), tlačítka `rescan_qrcode`, `cancel_mission`.
   - Pokud nalezena: Zobrazí `"Cesta nalezena"` (s délkou z metadat), tlačítka `rescan_qrcode`, `cancel_mission`, `mission_go`.
9. **Krok 18: Zahájení jízdy**
   - Po kliknutí na `mission_go`:
     - Spustí `DRIVE ON` (zapnutí napájení motorů na portu 9003).
     - Spustí `PILOT-ROBOTOUR START <json_z_maps>` (port 9104).
     - Signalizace: `SOUND barking` + `BLINK #FFA500 2 3000` (oranžová, 2Hz, 3s).
10. **Kroky 19 až 23: Monitorovací smyčka jízdy (perioda 1s)**
    - Dotazuje se na `PILOT-ROBOTOUR STATUS`:
      - **`RUNNING`**: Zobrazuje na terminálu `"Robotour - Jízda"` s aktuálním waypointem, vzdáleností a přesností, tlačítka `pause_mission`, `stop_mission`.
      - **`pause_mission`**: Zavolá `PILOT-ROBOTOUR PAUSE`. Zobrazí `"Robotour - Pozastaveno"`, tlačítka `resume_mission`, `cancel_mission`.
      - **`resume_mission`**: Zavolá `PILOT-ROBOTOUR RESUME`, pokračuje v jízdě.
      - **`STOPPED`**: Signalizace `SOUND game-over` + `BLINK #FF0000 2 3000`. Zobrazí `"Robotour - Ukončení jízdy"` s důvodem, tlačítko `acknowledge`.
      - **`FINISHED`**: Signalizace `SOUND meow` + `BLINK #00FF00 2 2000`. Zobrazí `"Robotour - Jsme v cíli!"`, tlačítko `acknowledge`.
11. **Ukončení jízdy vs ukončení služby (dva stavy):**
    - **Po kliknutí na `acknowledge` nebo při zrušení mise (`cancel_mission` / `stop_mission`):**
      - Zastaví se pouze služby jízdy: `DRIVE OFF`, `PILOT-ROBOTOUR STOP`, `MAPS STOP`, `LIDAR STOP`.
      - Polohové služby (`GNSS-*`, `RTK`, `FUSION`, `DRIVE` proces, `LOGGER`) zůstávají běžet!
      - Návrat na úvodní obrazovku (krok 2).
    - **Při ukončení celé služby MISSION-ROBOTOUR (příkaz `STOP` / `SHUTDOWN` na portu 9031 nebo signál procesu):**
      - Zastaví se VŠECHNY spuštěné služby: `PILOT-ROBOTOUR`, `MAPS`, `LIDAR`, `LOGGER`, `FUSION`, `RTK`, `GNSS-*`, `DRIVE` (`OFF` i `STOP`), `QRSCANER`.

---

## 4. Instalační a systémové skripty

- **`install/mission_robotour_register.sh`**:
  - Vytvoří logovací složku `/data/logs/mission_robotour/` (`user:user`).
  - Vytvoří systemd službu `robot-mission-robotour.service` na portu 9031.
  - Nastaví git execute bit (`100755`) a LF zakončení řádků.
- **`install/mission_robotour_unregister.sh`**:
  - Bezpečné zastavení a odregistrování služby ze systemd.
- **`install/readme.md`**:
  - Doplnění příkazů pro kontrolu a sledování logů služby `mission-robotour`.

---

## 5. Verifikační plán

### Automatizované testy
- `python -m unittest fusion.test_fusion` (ověření rozšířeného START/STOP v FUSION).
- `python -m unittest pilot_robotour.test_pilot_robotour` (ověření START s JSON trasou a nového flat JSON STATUS).
- `python -m unittest mission-robotour.test_mission_robotour`:
  - Test kroků 0-1 (PING a START mikroslužeb).
  - Test příjmu QR kódu a validace `geo:<lat>,<lon>`.
  - Test kontroly GPS polohy a vzdálenosti.
  - Test komunikace s MAPS a předání trasy do PILOT-ROBOTOUR.
  - Test stavového automatu během jízdy (PAUSE, RESUME, STOPPED, FINISHED, acknowledge).
  - Test odesílání SOUND a BLINK na TERMINAL.
  - Test bezpečného zastavení a obsluhy signálů.

### Manuální / systémové ověření
- Kontrola syntaxe skriptů bash (`bash -n`).
- Kontrola funkčnosti TCP příkazů `PING`, `START`, `STOP`, `STATUS`, `EXIT`, `SHUTDOWN` na portu 9031.
