# Služba PILOT-MARATON (Port 9105)

Služba `pilot_maraton` zajišťuje autonomní řízení robota podle navigačních bodů (waypoints) pro soutěžní misi Maraton (cíl ujet 5 km za méně než hodinu), se zapojením kolizních kurzů ze služby `LIDAR-COLLISIONS`, bezpečnostního dohledu OOW a fúzovaných pozičních dat.

## Architektura a komunikace

### 1. ZMQ Odběry (Subscriptions)
* **`ipc:///tmp/robot-fusion`** (Topic: `SOLUTION`) – fúzovaná poloha a azimut.
* **`ipc:///tmp/robot-lidar-collisions`** (Topic: `COLLISIONS`) – primární antikolizní data (72 sektorů po $5^\circ$ v těle robota `distances_cm`, predikční diferenciál `delta_d_cms`, `min_ttc_s` a `min_ttc_angle_deg`).
* **`ipc:///tmp/robot-lidar-traversability`** (Topic: `TRAVERSABILITY`) – 2D potenciálová mapa sjízdnosti terénu ($80 \times 80$ buněk po $10\text{ cm}$ v `grid_b64`, cena `0..100`, volná vzdálenost předního koridoru `min_corridor_clearance_cm`) jako podklad pro plánovač / Elastic Band (`get_traversability_cost_at(x_fwd_m, y_cw_m)`).
* **`ipc:///tmp/robot-lidar`** (Topic: `DISTANCE`) – záložní antikolizní vzdálenost z LiDARu.
* **`ipc:///tmp/robot-oow`** (Topics: `STATUS`, `CMD`) – bezpečnostní dohled OOW.

### 2. Logika řízení rychlosti a zastavení před překážkou
* **Maximální rychlost:** až $2.0\text{ m/s}$ ($200\text{ cm/s}$, omezeno parametrem `START`) při volném prostoru v předním koridoru $\ge 6.0\text{ m}$ ($600\text{ cm}$).
* **Minimální vzdálenost zastavení:** $0.7\text{ m}$ ($70\text{ cm}$) před překážkou.
* **Pravidlo $\frac{2}{5}\,\text{TTC}$:** Robot omezuje rychlost tak, aby dokázal zastavit za $\frac{2}{5}\cdot\text{min\_ttc\_s}$ při predikovatelném brzdném zpomalení $2.0\text{ m/s}^2$ (a v nouzi dynamicky zvyšuje brzdné zpomalení až na $0.4g \approx 4.0\text{ m/s}^2$).

### 3. Odchozí TCP komunikace
* **DRIVE (`127.0.0.1:9003`)** – posílání povelů motorům (`DRIVE pwm ...`, `STOP`, `BREAK`).
* **OOW Poller (`127.0.0.1:9030`)** – periodický heartbeat dotaz `OOW\n`.

### 4. Příchozí TCP komunikace (Port 9105)
Služba naslouchá na portu **`9105`**:
* `START [speed] [pwm] <json_trasa>` – spuštění autonomní jízdy po waypointové trase předané v JSON payloadu.
* `STOP` – zastavení řízení (`STOPPED`).
* `PAUSE` – dočasné pozastavení (`PAUSED`).
* `RESUME` – obnovení jízdy (`RUNNING`).
* `STATUS` – aktuální stav (waypoint index, souřadnice, azimut, kvalita fixu, `obstacle_distance_cm`, `min_ttc_s`, `min_ttc_angle_deg`).
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
