# Služba PILOT-MARATON (Port 9105)

Služba `pilot_maraton` zajišťuje autonomní řízení robota podle navigačních bodů (waypoints) pro soutěžní misi Maraton (cíl ujet 5 km za méně než hodinu), s budoucím zapojením antikolizního algoritmu kolizních kurzů z LiDARu a detekce krajnic z kamer.

V této fázi vychází ze služby `pilot_robotour`, používá osvědčené řídicí jádro a slouží jako základ pro misi `mission_maraton`.

## Architektura a komunikace

### 1. ZMQ Odběry (Subscriptions)
* **`ipc:///tmp/robot-fusion`** (Topic: `SOLUTION`) – fúzovaná poloha a azimut.
* **`ipc:///tmp/robot-lidar`** (Topic: `DISTANCE`) – antikolizní data z LiDARu (< 70 cm zastavení).
* **`ipc:///tmp/robot-oow`** (Topics: `STATUS`, `CMD`) – bezpečnostní dohled OOW.

### 2. Odchozí TCP komunikace
* **DRIVE (`127.0.0.1:9003`)** – posílání povelů motorům (`DRIVE pwm ...`, `STOP`, `BREAK`).
* **OOW Poller (`127.0.0.1:9030`)** – periodický heartbeat dotaz `OOW\n`.

### 3. Příchozí TCP komunikace (Port 9105)
Služba naslouchá na portu **`9105`**:
* `START [speed] [pwm] <json_trasa>` – spuštění autonomní jízdy po waypointové trase předané v JSON payloadu.
* `STOP` – zastavení řízení (`STOPPED`).
* `PAUSE` – dočasné pozastavení (`PAUSED`).
* `RESUME` – obnovení jízdy (`RUNNING`).
* `STATUS` – aktuální stav (waypoint index, souřadnice, azimut, kvalita fixu, překážky).
* `PING` – vrací `PONG PILOT_MARATON`.
* `EXIT` – uzavření klientského spojení (`BYE`).
* `SHUTDOWN` – ukončení procesu služby.

---

## Spuštění a správa

### Ruční spuštění:
```bash
python3 pilot_maraton/main.py
```

### Systemd služba:
* Služba: `robot-pilot-maraton.service`
* Registrace: `install/pilot_maraton_register.sh`
* Odregistrace: `install/pilot_maraton_unregister.sh`
* Logy: `/data/logs/pilot_maraton/pilot_maraton.log`
* Telemetrie a trasy: `/data/robot/pilot_maraton/<yyyy-mm-dd>/<HH-MM-SS>/`
