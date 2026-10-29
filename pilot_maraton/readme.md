# Služba PILOT-MARATON (Port 9105)

Služba `pilot_maraton` zajišťuje autonomní řízení robota podle navigačních bodů (waypoints) pro soutěžní misi Maraton (cíl ujet 5 km za méně než hodinu), se zapojením kolizních kurzů ze služby `LIDAR-COLLISIONS`, bezpečnostního dohledu OOW a fúzovaných pozičních dat.

## Architektura a komunikace

### 1. ZMQ Komunikace (Odběry a Publikace)
* **Odběr `ipc:///tmp/robot-fusion`** (Topic: `SOLUTION`) – fúzovaná poloha a azimut.
* **Odběr `ipc:///tmp/robot-lidar-collisions`** (Topic: `COLLISIONS`) – primární antikolizní data (72 sektorů po $5^\circ$ v těle robota `distances_cm`, predikční diferenciál `delta_d_cms`, `min_ttc_s` a `min_ttc_angle_deg`).
* **Odběr `ipc:///tmp/robot-lidar-traversability`** (Topic: `TRAVERSABILITY`) – 2D potenciálová mapa sjízdnosti terénu ($80 \times 80$ buněk po $10\text{ cm}$ v `grid_b64`, cena `0..100`, volná vzdálenost předního koridoru `min_corridor_clearance_cm`) jako podklad pro koridorový plánovač (`get_traversability_cost_at(x_fwd_m, y_cw_m)`).
* **Odběr `ipc:///tmp/robot-lidar`** (Topic: `DISTANCE`) – záložní antikolizní vzdálenost z LiDARu.
* **Odběr `ipc:///tmp/robot-oow`** (Topics: `STATUS`, `CMD`) – bezpečnostní dohled OOW.
* **Publikace `ipc:///tmp/robot-pilot-maraton`** (Topic: `STATE`) – okamžité notifikace o změně stavu (`state`, `nav_state`, `nav_info`, `lateral_offset_m`, indexy waypointů a rychlosti) pro `mission_maraton`.

### 2. Koridor trasy a stavový automat vyhýbání překážkám (`nav_state`)
* **Asymetrický koridor jízdy:** Kolem ideální trasy z `MAPS` (posunuté vpravo od osy cesty) má střed otáčení robota povolené pásmo **`1.0 m` vpravo (`+1.0 m`) a `2.0 m` vlevo (`-2.0 m`)**. Vnitřní rohy trasy jsou zaobleny kvadratickou Bézierovou křivkou a přepínány přes úhlovou osu rohu (bisektoru). Fyzické okraje cesty, obrubníky a příkopy ořezává `LIDAR-TRAVERSABILITY` (`cost == 100`).
* **Hrubý terén vs. 3D překážka:**
  * Nerovnosti a nízké terénní vady (`LIDAR-TRAVERSABILITY` bez 3D zásahu v `LIDAR-COLLISIONS`) robot **plynule objíždí za jízdy** ve stavu `CRUISING` bez zastavení.
  * U **3D překážky** (`LIDAR-COLLISIONS`) na ideální trase robot plynule zpomalí (`OBSTACLE`) a zastaví ve vzdálenosti `~1.45 m` (nouzová hranice `0.70 m` a pravidlo $\frac{2}{5}\,\text{TTC}$ zůstávají vždy aktivní).
* **Vyčkávání a dynamické překážky (`WAITING_CLEARANCE`):**
  * Po zastavení u 3D překážky běží základní čekání **`5.0 s`** (pokud se trasa uvolní souvisle na `>= 1.0 s`, ihned pokračuje v `CRUISING`).
  * Změní-li překážka polohu v ENU o **`>= 20 cm`**, vyhodnotí se jako **pohybující se**, její nafouknutá stopa se zaznamená do zametené oblasti (`swept_cells`) a čekání se prodlužuje vždy o **`+5.0 s` od posledního pohybu až do maxima `60.0 s` (1 minuta)**.
  * Při následném hledání objížďky (`AVOIDING`, omezení rychlosti na `1.0 m/s`) má aktuální poloha překážky neprůchodnou cenu a historicky zametená oblast vysokou penalizaci (`cost 92`), takže ji plánovač pokud možno celou mine, ale v nouzi projede uvolněnou částí s bezpečným odstupem od živé překážky.
* **Sledování a předjíždění chodce ve stejném směru (`FOLLOWING` -> `OVERTAKING`):**
  * Pohybuje-li se překážka vpřed ve směru trasy ($v_{\parallel} \ge 0.30\text{ m/s}$, rozptyl do stran $\le 0.45\text{ m}$), robot nezastavuje na nulu, ale plynule srovná rychlost s odstupem `~2.0 m` na **`2.5 s`**.
  * Je-li rozdíl maximální rychlosti robota a rychlosti chodce **`>= 0.60 m/s`** a v koridoru existuje čistá stopa s bočním odstupem středů **`>= 0.95 m`**, přejde do **`OVERTAKING`** plnou povolenou rychlostí. Kvůli slepému úhlu předního $180^\circ$ LiDARu predikuje polohu chodce a drží levý pruh, dokud nemá náskok **`>= 2.2 m`** před predikovanou polohou chodce.
* **Neprůjezdná cesta (`NO_PATH`) a obsazený cíl:**
  * Není-li v koridoru `[-2.0 m, +1.0 m]` žádná průjezdná cesta, robot stojí ve stavu **`NO_PATH`** (`v = 0`), na `10 Hz` kontroluje uvolnění a po `>= 1.0 s` volné cesty sám pokračuje. Stojí-li překážka po vyčkání přímo v okruhu `<= 2.0 m` od koncového cíle trasy, jízda se čistě ukončí jako `FINISHED`.

### 3. Logika řízení rychlosti a nouzového brzdění
* **Maximální rychlost:** až $2.0\text{ m/s}$ ($200\text{ cm/s}$, omezeno parametrem `START`) při volném prostoru ve směru jízdy $\ge 6.0\text{ m}$ ($600\text{ cm}$).
* **Minimální vzdálenost zastavení:** $0.7\text{ m}$ ($70\text{ cm}$) před překážkou (cílové provozní zastavení před 3D překážkou `~1.45 m`).
* **Pravidlo $\frac{2}{5}\,\text{TTC}$:** Robot omezuje rychlost tak, aby dokázal zastavit za $\frac{2}{5}\cdot\text{min\_ttc\_s}$ při predikovatelném brzdném zpomalení $2.0\text{ m/s}^2$ (a v nouzi dynamicky zvyšuje brzdné zpomalení až na $0.4g \approx 4.0\text{ m/s}^2$).

### 4. Odchozí TCP komunikace
* **DRIVE (`127.0.0.1:9003`)** – posílání povelů motorům (`DRIVE pwm ...`, `STOP`, `BREAK`).
* **OOW Poller (`127.0.0.1:9030`)** – periodický heartbeat dotaz `OOW\n`.

### 5. Příchozí TCP komunikace (Port 9105)
Služba naslouchá na portu **`9105`**:
* `START [speed] [pwm] <json_trasa>` – spuštění autonomní jízdy po waypointové trase předané v JSON payloadu.
* `STOP` – zastavení řízení (`STOPPED`).
* `PAUSE` – dočasné pozastavení (`PAUSED`).
* `RESUME` – obnovení jízdy (`RUNNING`).
* `STATUS` – aktuální stav v JSON (`state`, `nav_state`, `nav_info`, `lateral_offset_m`, waypoint index, souřadnice, azimut, kvalita fixu, `obstacle_distance_cm`, `min_ttc_s`, `min_ttc_angle_deg`).
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
