# Kolizní kurzy a prostorové vyhodnocení překážek (LiDAR L2)

Tento dokument specifikuje návrh a implementaci služby kolizních kurzů **`LIDAR-KOLIZE`** (TCP port 9222). Služba běží v rámci jednoho společného procesu s nízkoúrovňovým ovladačem **`LIDAR`** (TCP port 9002), ale disponuje vlastní sadou vláken a plně odděleným řízením, diagnostikou i publikační sběrnicí.

---

## 1. Cíle a klíčové principy

1. **Globální orientace sektorů (North-Up):**
   - Středem soustavy je střed otáčení robota (Center of Rotation – CoR), ale orientace 36 výsečí je stabilizována vůči severu (0° = sever).
   - Výstupní sektory umožňují přímé promítnutí překážek do globální mapy plánovače a predikci budoucích trajektorií překážek nezávisle na manévrech robota.
2. **Výšková stratifikace (12 pater):**
   - Válcovitá oblast kolem robota je rozdělena do 12 pater po 10 cm ($Z \in [10, 130]$ cm od země, kde $Z = 0$ je úroveň terénu).
   - Spodní patro (10–20 cm) nezakryje překážky ve vyšších patrech (např. tráva vs. přibližující se člověk, pes či míč).
3. **Dynamické sledování (Rychlost přibližování a TTC):**
   - V každé buňce běží dvoustavový $\alpha$-$\beta$ filtr sledující vzdálenost a rychlost zkracování vzdálenosti.
   - Znaménková konvence: **$v > 0$ přibližování k robotu**, **$v < 0$ vzdalování od robota**.
   - Sousední patra jsou vertikálně provázána (handover stavu při pohybu míče/překážky mezi patry či při pohupování robota na nerovnostech).
4. **Oddělení geometrie a inflace:**
   - **Výstupní mapa překážek zůstává nezkreslená (bez nafukování)** pro věrnou globální mapu a plánovač.
   - **Nafouknutí (válec těla robota $R = 37.5$ cm) se aplikuje výhradně pro výpočet TTC a kolizního rizika.**
5. **Dva komplementární 36prvkové profily:**
   - **Profil A (Průjezdnost / Clearance):** Minimální reálná vzdálenost napříč všemi patry (robot projede jen tam, kde je volno ve všech patrech).
   - **Profil B (Kritické kolize / TTC):** Minimální Time-To-Collision napříč patry pro záchyt rychle se blížících objektů.

---

## 2. Geometrie a konfigurační konstanty

Všechny geometrické a systémové parametry jsou definovány v konfigurační struktuře:

```cpp
struct CollisionConfig {
    // Geometrie a montáž
    float lidar_offset_x_cm      = 22.0f;   // LiDAR je 22 cm před středem otáčení robota
    float robot_diameter_cm      = 75.0f;   // Válec těla robota (max šířka 60 cm + 7.5 cm rezerva na každé straně)
    float robot_radius_cm        = 37.5f;   // Poloměr pro inflaci při výpočtu TTC

    // Vertikální patra
    float z_min_cm               = 10.0f;   // Odfiltrování země/nerovností pod 10 cm
    float z_max_cm               = 130.0f;  // Horní limit obrysu robota
    float floor_height_cm        = 10.0f;   // Výška jednoho patra (celkem 12 pater)
    size_t num_floors            = 12;

    // Horizontální výseče
    size_t num_sectors           = 36;      // 36 sektorů po 10° (360° / 36)
    float sector_angle_deg       = 10.0f;   // Úhlová šířka sektoru

    // Časování a filtrace
    uint32_t period_ms           = 100;     // Perioda vyhodnocení 100 ms (10 Hz)
    float alpha                  = 0.40f;   // Koeficient polohy alfa-beta filtru
    float beta                   = 0.15f;   // Koeficient rychlosti alfa-beta filtru
    float max_track_jump_cm      = 40.0f;   // Max změna vzdálenosti za 100 ms pro zachování tracku
    size_t min_points_per_cell   = 2;       // Odfiltrování ojedinělých odrazů / šumu
    size_t min_points_per_cycle  = 10000;   // Minimální počet bodů za 100 ms pro platný cyklus
    uint32_t max_lag_ms          = 150;     // Max povolená prodleva mezi cykly
};
```

---

## 3. Architektura dvou služeb v jednom procesu

Aplikace integruje dva nezávislé servery běžící v oddělených vláknech:

```
+---------------------------------------------------------------------------------------+
| PROCES: robot_lidar_tcp                                                               |
|                                                                                       |
|  +-------------------------------------+   +---------------------------------------+  |
|  | Služba LIDAR (TCP 9002)             |   | Služba LIDAR-KOLIZE (TCP 9222)        |  |
|  | - UDP reader (Unitree L2)           |   | - TCP server (příkazy a diagnostika)  |  |
|  | - Kalibrace & raw logger            |   | - ZMQ SUB (kurz z /tmp/robot-fusion)  |  |
|  | - ZMQ PUB (DISTANCE na /tmp/robot-  |   | - 100ms kolizní evaluátor (alfa-beta) |  |
|  |   lidar)                            |   | - ZMQ PUB (COLLISIONS na /tmp/robot-  |  |
|  | - Ring buffer vzorků (65 536 bodů)  |   |   lidar-kolize)                       |  |
|  +------------------+------------------+   +-------------------+-------------------+  |
|                     |                                          |                      |
|                     +======== Sdílený Point Buffer ============+                      |
|                               (Inkrementální čtení)                                   |
+---------------------------------------------------------------------------------------+
```

### Životní cyklus a závislosti

1. **START na portu 9222:**
   - Spustí vyhodnocovací smyčku kolizí.
   - Pokud hardware `LIDAR` (9002) ještě neběží nebo `fusion` nedodává data, přejde do stavu `WAITING_FOR_DATA` (konkrétně loguje `WAITING FOR LIDAR` nebo `WAITING FOR HEADING`).
   - Jakmile začnou proudit body i kurz, automaticky se spustí plné vyhodnocování.
2. **STOP na portu 9222:**
   - Zastaví kolizní smyčku a publikační vlákno. Hardware LiDARu (9002) může dál běžet.
3. **Pravidlo bezpečného SHUTDOWN (na obou portech 9002 i 9222):**
   - Příkaz `SHUTDOWN` zkontroluje, že **obě služby stojí** (`IDLE`).
   - Pokud kterákoli z nich ještě běží (`RUNNING`), příkaz selže s chybou:  
     `ERR CANNOT_SHUTDOWN_WHILE_RUNNING (STOP services first)`.
   - Teprve po zastavení obou je celý proces bezpečně ukončen.
4. **Pravidelný reporting na stdout (1 Hz):**
   - Kolizní služba každou sekundu vypisuje stav: počet zpracovaných cyklů za sekundu, počet bodů, zpoždění a případné výpadky.

---

## 4. Algoritmus krok za krokem

```
                 +----------------------------------------+
                 |  Vstup: ~13 000 bodů za 100 ms         |
                 |  ZMQ Fusion: heading_start/end         |
                 +-------------------+--------------------+
                                     |
                                     v
                 +----------------------------------------+
                 | Krok 1: Kontrola kvality & Inkrement   |
                 | - Inkrementální vyčtení z bufferu      |
                 | - Kontrola: body >= 10 000             |
                 | - Kontrola: prodleva <= 150 ms         |
                 +-------------------+--------------------+
                                     |
                                     v
                 +----------------------------------------+
                 | Krok 2: Transformace do severu         |
                 | - Posun o X_offset = +22 cm            |
                 | - Rotace do severu (+ heading)         |
                 | - Zařazení do buňky [patro][sektor]    |
                 +-------------------+--------------------+
                                     |
                                     v
                 +----------------------------------------+
                 | Krok 3: Filtrace & Detekce d_min       |
                 | - Rank filtr / min 2 body              |
                 | - Matice d_meas[12][36]                |
                 +-------------------+--------------------+
                                     |
                                     v
                 +----------------------------------------+
                 | Krok 4: Dynamika (Alfa-Beta tracker)   |
                 | - Odhad v = -dd/dt (v > 0 přibližování)|
                 | - Vertikální handover mezi patry f±1   |
                 +-------------------+--------------------+
                                     |
                                     v
                 +----------------------------------------+
                 | Krok 5: Inflace (jen pro TTC)          |
                 | - delta_theta = asin(R_safe / d)       |
                 | - Výpočet TTC_inflated v kuželu        |
                 +-------------------+--------------------+
                                     |
                                     v
                 +----------------------------------------+
                 | Krok 6: Syntéza výstupních profilů     |
                 | - Profil A: d_min[36] (průjezdnost)    |
                 | - Profil B: TTC_min[36] (kolize)       |
                 | - ZMQ PUB topic: COLLISIONS            |
                 +-------------------+--------------------+
```

### Krok 1: Inkrementální vyčítání a kontrola kvality dat
- Kolizní vlákno udržuje index a časovou značku posledního zpracovaného vzorku.
- Z ring bufferu vyčte přesně nové body, které přibyly za posledních 100 ms.
- **Validace počtu bodů:** Pokud je počet bodů $< 10\,000$, inkrementuje se čítač `cycles_low_points` a publikace se pozastaví (laser nabíhá nebo je blokován).
- **Validace zpoždění (Lag):** Pokud je prodleva mezi snímky $> 150\text{ ms}$, zaloguje se varování `WARN TIMING_LAG`, inkrementuje se čítač `cycles_lag_dropped` a prediktor se resetuje.
- **Iniciace filtru:** Pro první publikaci jsou vyžadovány alespoň dva za sebou jdoucí platné snímky.

### Krok 2: Transformace bodů a redukce do mřížky 12 × 36
- Odečte se `heading_start`, `heading_end` a `heading_avg` ze zpráv služby `fusion`.
- Pro každý bod $(x_L, y_L, z_L)$:
  1. Kontrola výšky: $z = z_L \in [10, 130]$ cm $\implies$ patro $\text{floor} = \lfloor (z - 10) / 10 \rfloor \in [0, 11]$.
  2. Převod na střed otáčení robota:
     $$x_R = x_L + 22.0\text{ cm},\quad y_R = y_L$$
  3. Vzdálenost od středu otáčení:
     $$d = \sqrt{x_R^2 + y_R^2}$$
  4. Globální azimut bodu:
     $$\theta_{local} = \text{atan2d}(y_R, x_R)$$
     $$\theta_{global} = \text{fmod}(\theta_{local} + \text{heading}_{avg} + 360^\circ, 360^\circ)$$
  5. Sektor: $\text{sector} = \lfloor \theta_{global} / 10^\circ \rfloor \in [0, 35]$.

### Krok 3: Filtrace šumu a minimální naměřené vzdálenosti
- Pro každou buňku $(f, s)$ se určí minimální vzdálenost $d_{meas}(f, s)$.
- Buňka je validní pouze při počtu bodů $\ge 2$ (eliminace prachu, kapek a osamocených paprsků).

### Krok 4: Dynamické sledování ($\alpha$-$\beta$ tracker a vertikální provázanost)
Každá buňka udržuje stav trackeru: $[d_{est}, v_{est}, \text{active}]$.
1. **Predikce:**
   $$d_{pred} = d_{est} - v_{est} \cdot \Delta t \quad (\Delta t = 0.1\text{ s})$$
2. **Párování a mezipatrová vazba:**
   - Pokud v buňce $(f, s)$ naměříme $d_{meas}$ blízké $d_{pred}$ ($|d_{meas} - d_{pred}| < 40\text{ cm}$), track pokračuje.
   - Pokud v buňce $(f, s)$ měření chybí, ale v sousedním patře $(f-1, s)$ nebo $(f+1, s)$ se objeví nové měření se shodnou vzdáleností, **track a jeho odhadnutá rychlost $v_{est}$ se předá do nového patra**.
3. **Korekce ($\alpha$-$\beta$ filtr):**
   $$\text{reziduum } r = d_{pred} - d_{meas}$$
   $$d_{est} = d_{pred} - \alpha \cdot r$$
   $$v_{est} = v_{est} + \frac{\beta}{\Delta t} \cdot r$$
   *(Kladné $v_{est}$ znamená přibližování).*

### Krok 5: Výpočet TTC a inflace překážek
- Reálné polohy překážek $d_{est}(f, s)$ se **nenafukují** (věrný geometrický obrys).
- Pro stanovení kolizního rizika se překážka ve vzdálenosti $d$ nafoukne o $R_{safe} = 37.5\text{ cm}$ do úhlového kuželu:
  $$\Delta \theta = \arcsin\left(\frac{R_{safe}}{d}\right)$$
  Počet zasažených sousedních sektorů: $k = \lceil \Delta \theta / 10^\circ \rceil$.
- Pokud se překážka přibližuje ($v_{est} > 0.05\text{ m/s}$), promítne svůj $TTC = \frac{d_{est}}{v_{est}}$ do všech sektorů v rozsahu $s \pm k$.

### Krok 6: Výstupní profily pro řízení (36 hodnot)
1. **Clearance profil (Průjezdnost):**
   $$d_{min}(s) = \min_{f \in [0, 11]} d_{est}(f, s)$$
   Včetně rychlosti $v_{clearance}(s)$ překážky, která toto minimum způsobila, a indexu kritického patra $f_{crit}$.
2. **Kolizní profil (TTC):**
   $$TTC_{min}(s) = \min_{f \in [0, 11]} TTC_{inflated}(f, s)$$
   Včetně vzdálenosti a rychlosti přibližujícího se tělesa.

---

## 5. TCP Protokol služby LIDAR-KOLIZE (port 9222)

Služba naslouchá na `127.0.0.1:9222` a podporuje následující příkazy:

| Příkaz | Odpověď | Popis |
| :--- | :--- | :--- |
| **`PING`** | `PONG LIDAR-KOLIZE` | Ověření dostupnosti služby |
| **`START`** | `OK STARTED` / `ALREADY RUNNING` | Spuštění vyhodnocovací smyčky |
| **`STOP`** | `OK STOPPED` | Zastavení vyhodnocovací smyčky |
| **`STATUS`** | `<STATE> <JSON>` | Aktuální stav (`IDLE`, `WAITING_FOR_DATA`, `RUNNING`) a podrobné metriky |
| **`COLLISIONS`** | `<JSON>` / `ERR NOT_READY` | Vrátí poslední vygenerovaný JSON kolizní zprávy po TCP |
| **`EXIT`** | `BYE` | Uzavření klientského TCP spojení |
| **`SHUTDOWN`** | `SHUTTING DOWN` / `ERR RUNNING` | Ukončení celého procesu (pouze pokud jsou obě služby `IDLE`) |

### Příklad odpovědi na `STATUS`:
```text
RUNNING {"cycles_total":1240,"cycles_published":1235,"cycles_low_points":3,"cycles_lag_dropped":2,"points_last_cycle":12840,"min_clearance_cm":185.4,"min_ttc_s":2.45,"heading_avg":45.2,"eval_duration_ms":0.62}
```

---

## 6. ZMQ Rozhraní

### Vstupní odběr (SUB)
- **Endpoint:** `ipc:///tmp/robot-fusion`
- **Topic:** `SOLUTION`
- **Sledovaná pole:** `heading` (ve stupních vůči severu), `timestamp`.
- **Chování při výpadku:** Pokud `fusion` neběží nebo data chybí, služba striktně čeká ve stavu `WAITING FOR HEADING` a nepublikuje nezarovnaná data.

### Výstupní publikace (PUB)
- **Endpoint:** `ipc:///tmp/robot-lidar-kolize`
- **Topic:** `COLLISIONS` (perioda 100 ms / 10 Hz)
- **Zpětná kompatibilita:** Původní služba `LIDAR` (9002) nadále samostatně publikuje `DISTANCE` na `ipc:///tmp/robot-lidar`.

### Schéma zprávy JSON (`COLLISIONS`)

```json
{
  "timestamp": 1729312000.123,
  "heading_start": 45.2,
  "heading_end": 45.6,
  "heading_avg": 45.4,
  "points_count": 12840,
  "eval_duration_ms": 0.62,
  "sectors": [
    {
      "sector_idx": 0,
      "azimuth_deg": 0.0,
      "clearance": {
        "distance_cm": 240.5,
        "closing_speed_ms": 0.0,
        "floor_idx": 3
      },
      "collision": {
        "ttc_sec": 2.1,
        "threat_distance_cm": 310.0,
        "closing_speed_ms": 1.48,
        "floor_idx": 5
      }
    }
  ]
}
```

---

## 7. Výpočetní náročnost a latence

- **Krok 1 (Inkrementální načtení ~13k bodů):** < 0.1 ms
- **Krok 2 (Transformace do severu a redukce do mřížky 12 × 36):** ~0.3–0.5 ms
- **Krok 3 a 4 ($\alpha$-$\beta$ tracker a vertikální provázanost nad 432 buňkami):** ~0.05 ms
- **Krok 5 a 6 (Inflace kuželu a syntéza 36 sektorů):** ~0.02 ms
- **Celková doba vyhodnocení:** **< 0.8 ms** na cyklus (< 1 % jednoho CPU jádra).
