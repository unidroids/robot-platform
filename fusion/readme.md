# Fusion Service (port 9009)

Služba `fusion` provádí fúzi navigačních dat pro autonomního pilota.

## Architektura a vstupy

1. **GNSS Konstelace 3 antén (Trojúhelník):**
   - **`gnss-gps` (UM980)**: Primární anténa předsazená dopředu `[+32, 0]` cm. Odběr z `ipc:///tmp/robot-gnss-gps` (topic `BESTNAV`).
   - **`gnss-dual` Master (UM982)**: Levá základna `[+25, +24]` cm. Odběr z `ipc:///tmp/robot-gnss-dual` (topic `BESTNAV`).
   - **`gnss-dual` Slave (UM982)**: Pravá základna `[+25, -24]` cm. Odběr z `ipc:///tmp/robot-gnss-dual` (topic `BESTNAVH`).
   - Geometrie: Základna Master-Slave $48\text{ cm}$, ramena GPS-Master a GPS-Slave $25\text{ cm}$.
   - Detekce a vyloučení chybné antény (FDE): Porovnání vzájemných vzdáleností vůči nominální geometrii. Pokud má primární anténa multipath/odlehlost, je automaticky vyloučena a výpočet polohy přechází na záložní antény.
   - Výsledná poloha je kinematicky transformována ze souřadnic antény na střed osy otáčení robota `[0, 0, 0]`.

2. **Držení severu (Heading Hold):**
   - **UNIHEADING (`NARROW_INT`, acc < 1.5°)**: Přímá inicializace, po inicializaci komplementární navazování s koeficientem $\alpha = 0.1$ pro zabránění skokům.
   - **Kinematická kompenzace a 3-anténní konsenzus kurzu (`BESTNAV_CONSENSUS`)**:
     - V zatáčkách má každá anténa vlivem své montážní polohy $[x_i, y_i]$ jiný vektor pohybu. Fúze počítá tečný úhel odchylky:
       $$\beta_i = \arctan2(\omega_z \cdot x_i,\; v + \omega_z \cdot y_i)$$
     - Každý surový kurz $\text{trk\_gnd}_i$ je korigován: $\theta_i = \text{trk\_gnd}_i - \beta_i$.
     - **3-anténní konsenzus (detekce lhaní GNSS)**:
       Při $v \ge 0.3\text{ m/s}$ a mírné rotaci $|\omega_z| \le 10^\circ/\text{s}$ se porovnávají všechny 3 kompenzované úhly:
       - Pokud se všechny 3 shodují v toleranci $\pm 3.5^\circ$ (`3_ANTENNAS_AGREE`), je potvrzeno, že GNSS nelže, a kurz je uzamčen.
       - Pokud 1 anténa ustřelí vlivem multipathu (např. pod stromem), systém ji izoluje jako outlier a uzamkne průměr ze 2 shodujících se antén (`2_AGREE_GPS_OUTLIER` apod.).
       - Pokud se antény neshodnou, GNSS kurz se odmítne a orientaci drží gyroskop.
   - **BESTNAV kurz (`trk_gnd`)**: Sekundární kurzový zámek při přímé jízdě robota: podmínka rychlosti vůči zemi z BESTNAV `hor_spd > 0.3 m/s`, $|\omega_z| < 3^\circ/\text{s}$, a 3 po sobě jdoucí měření se shodují v rozmezí $\pm 3^\circ$.
   - **IMU Dead Reckoning (20 Hz)**: Kontinuální integrace přírůstků úhlu $\Delta \text{yaw}$ ze služby `gnss-imu`.

3. **Odometrie a detekce klidu (ZUPT):**
   - **Korekce rychlosti**: Hoverboard vykazuje systematickou odchylku ~+5 % na rychlosti, naměřená rychlost je dělena faktorem `1.05`:
     $$v_{\text{korig}} = \frac{v_{\text{raw}}}{1.05}$$
   - **Zero Velocity Update (ZUPT)**: Pokud se kroky kol `left_steps` a `right_steps` nemění a robot stojí, přírůstek úhlu $\Delta\text{yaw}$ z gyroskopu se zcela ignoruje (nastaví se na $0.0^\circ$). Tím je eliminován jakýkoliv drift gyra při stání robota na křižovatkách či u QR kódů.

4. **Postoj robota (Attitude):**
   - Služba přebírá dynamický předklon (`pitch`) a boční náklon (`roll`) z komplementárního filtru služby `gnss-imu`.

5. **Odpojení staré služby compass:**
   - ZMQ kanál `ipc:///tmp/robot-compass` byl kompletně odstraněn.

## ZMQ Výstupy

- **`ipc:///tmp/robot-fusion`**:
  - Topic **`SOLUTION`**: JSON objekt `NavFusionData` (rozšířený o `pitch`, `roll`, `heading_source`, `antenna_status`).
  - Topic **`DEBUG_HEADING`**: JSON objekt s podrobnými směry a diagnostikou.

## TCP Příkazy (localhost:9009)

- `PING` -> `PONG FUSION`
- `STATUS` -> `<MODE> <STATE_JSON> <SOLUTION_JSON>`
  - `STATE_JSON` obsahuje klíč `diagnostics` s detailním rozpadem:
    - `constellation`: stav trojúhelníku, aktivní anténa, naměřené vzdálenosti antén.
    - `antennas`: validita, pos_type, hAcc, stáří zpráv pro každou ze 3 antén.
    - `heading_hold`: aktuální zdroj (`UNIHEADING`, `BESTNAV_COURSE`, `GYRO`), úhel, přesnost, délka kurzového bufferu.
    - `odometry`: surová a korigovaná rychlost, korekční faktor 1.05.
    - `attitude`: pitch, roll, gyroZ.
- `RESTART` -> restart služby
- `EXIT` -> ukončení spojení klienta
- `SHUTDOWN` -> bezpečné ukončení celého procesu
