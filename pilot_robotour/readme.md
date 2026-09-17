# Služba PILOT-ROBOTOUR (Port 9104)

Služba `pilot_robotour` zajišťuje autonomní řízení robota podle navigačních bodů (waypoints) pro soutěžní misi Robotour s budoucím zapojením kamery a LiDARu.

V této fázi vychází ze služby `pilot_waypoints`, používá identické řídicí jádro a slouží jako základ pro budoucí misi `mission-robotour`.

## Architektura a komunikace

### 1. ZMQ Odběry (Subscriptions)
* **`ipc:///tmp/robot-fusion`** (Topic: `SOLUTION`) – fúzovaná poloha a azimut.
* **`ipc:///tmp/robot-lidar`** (Topic: `DISTANCE`) – antikolizní data z LiDARu (< 70 cm zastavení).
* **`ipc:///tmp/robot-oow`** (Topics: `STATUS`, `CMD`) – bezpečnostní dohled OOW.

### 2. Odchozí TCP komunikace
* **DRIVE (`127.0.0.1:9003`)** – posílání povelů motorům (`DRIVE pwm ...`, `STOP`, `BREAK`).
* **OOW Poller (`127.0.0.1:9013`)** – periodický heartbeat dotaz `OOW\n`.

### 3. Příchozí TCP komunikace (Port 9104)
Služba naslouchá na portu **`9104`**:
* `START [speed] [pwm]` – spuštění autonomní jízdy po waypointové trase.
* `STOP` – zastavení řízení (`STOPPED`).
* `PAUSE` – dočasné pozastavení (`PAUSED`).
* `RESUME` – obnovení jízdy (`RUNNING`).
* `STATUS` – aktuální stav (waypoint index, souřadnice, azimut, kvalita fixu).
* `PING` – vrací `PONG PILOT_ROBOTOUR`.
* `EXIT` – uzavření klientského spojení (`BYE`).
* `SHUTDOWN` – ukončení procesu služby.

---

## Spuštění a správa

### Ruční spuštění:
```bash
python3 pilot_robotour/main.py
```

### Systemd služba:
* Služba: `robot-pilot-robotour.service`
* Registrace: `install/pilot_robotour_register.sh`
* Odregistrace: `install/pilot_robotour_unregister.sh`
* Logy: `/data/logs/pilot_robotour/pilot_robotour.log`
