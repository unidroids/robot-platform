# Výsledky analýzy a zpětné rekonstrukce rozhodovacího procesu pilota

Byla provedena detailní analýza služby `pilot_robotour`, algoritmu výběru cílových bodů a trajektorie řízení. Na základě této analýzy byl vytvořen nástroj pro zpětnou rekonstrukci a vygenerováno obohacené CSV.

## 1. Analýza služby pilot_robotour

### Jaký bod se pilot snaží dosáhnout?
Služba `pilot_robotour` pracuje se dvěma úrovněmi cílů:
1. **Navigační cíl úseku (Waypoint)**: Robot se pohybuje po segmentech definovaných body z `route.json`. Cílem aktuálního segmentu je waypoint $W_{i+1}$ (`target_wp_lat`, `target_wp_lon`).
2. **Okamžitý dynamický řídicí bod (Lookahead / Near Point)**: V algoritmu `NearWaypoint` (`near_waypoint.py`) se čumák robota předsazený o $0.40\text{ m}$ promítne na trať:
   - Určí se kolmý průmět $P_{closest}$ (`closest_lat`, `closest_lon`) a boční odchylka $d_{perp}$ (`d_perp_m`).
   - Ve vzdálenosti lookahead $L = 2.0\text{ m}$ po směru segmentu se určí bod $P_{near}$ (`near_lat`, `near_lon`).
   - Na tento bod míří požadovaný kurz `target_heading`.

### Po jaké křivce pilot jede?
Robot sleduje **teoretický geometrický kruhový oblouk (Pure Pursuit)**:
- Jde o kružnici procházející čumákem robota a bodem $P_{near}$, která je tečná k aktuálnímu kurzu robota (`heading`).
- S úhlovou odchylkou $\alpha = \text{heading\_error} = \text{target\_heading} - \text{heading}$ a vzdáleností $L$:
  $$\kappa = \frac{2 \sin(\alpha)}{L} \quad [\text{m}^{-1}], \qquad R = \frac{L}{2 |\sin(\alpha)|} \quad [\text{m}]$$
- Směr zatáčení: $\alpha > 0 \implies \text{RIGHT}$, $\alpha < 0 \implies \text{LEFT}$, $\alpha \approx 0 \implies \text{STRAIGHT}$ ($R \to \infty$).

---

## 2. Vytvořené soubory v `pilot_robotour/temp/analysis`

1. **[reconstruct_pilot_log.py](file:///e:/robotour/robot-platform/pilot_robotour/temp/analysis/reconstruct_pilot_log.py)**
   - Samostatný CLI program umožňující zrekonstruovat jakýkoliv CSV log jízdy spolu s `route.json`.
   - Podporuje parametry `--csv`, `--route`, `--output`, `--lookahead`.
2. **[navigate-10-41-07-enriched.csv](file:///e:/robotour/robot-platform/pilot_robotour/temp/analysis/navigate-10-41-07-enriched.csv)**
   - Obohacený CSV soubor se všemi 441 řádky původního záznamu a 11 novými sloupci:
     - `target_wp_index`: index cílového uzlu segmentu
     - `target_wp_lat`, `target_wp_lon`: GPS souřadnice cílového uzlu segmentu
     - `near_lat`, `near_lon`: GPS souřadnice okamžitého lookahead bodu
     - `closest_lat`, `closest_lon`: GPS souřadnice kolmého průmětu na segment
     - `lookahead_dist_m`: vzdálenost k lookahead bodu ($2.0\text{ m}$)
     - `geom_curvature`: křivost $\kappa$ v $\text{m}^{-1}$
     - `geom_radius_m`: poloměr zakřivení $R$ v metrech
     - `geom_turn_direction`: směr zatáčení (`LEFT`, `RIGHT`, `STRAIGHT`)

---

## 3. Výsledky verifikace a přesnost

### Běh 1: 10-41-07 (`temp/strom/10-41-07`)
| Metrika | Naměřená hodnota |
| :--- | :--- |
| Celkový počet zpracovaných řádků | **441** (100% shoda) |
| Aktivní navigační řádky (`source=NAV`) | **208** |
| Neaktivní řádky (výpadek LiDARu, `STOPPED`) | **233** (ponechány prázdné) |
| Přesná shoda `target_heading` (< $0.0001^\circ$) | **208 / 208 (100.0 %)** |
| Maximální numerická odchylka azimutu | **$0.00000002^\circ$** ($2 \cdot 10^{-8}$) |

### Běh 2: 10-35-44 (`temp/strom/10-35-44`)
| Metrika | Naměřená hodnota |
| :--- | :--- |
| Celkový počet zpracovaných řádků | **2 800** (100% shoda) |
| Aktivní navigační řádky (`source=NAV`) | **1 708** |
| Neaktivní řádky (`LIDAR`, `OOW_ZMQ`, `PILOT`) | **1 092** (ponechány prázdné) |
| Projevené segmenty trasy | **Waypoint 2 až 19** |
| Přesná shoda `target_heading` (< $0.0001^\circ$) | **1 708 / 1 708 (100.0 %)** |
| Maximální numerická odchylka azimutu | **$0.00000005^\circ$** ($5 \cdot 10^{-8}$) |

### Běh 3: 10-28-00 (`temp/strom/10-28-00`)
| Metrika | Naměřená hodnota |
| :--- | :--- |
| Celkový počet zpracovaných řádků | **4 397** (100% shoda) |
| Aktivní navigační řádky (`source=NAV`) | **3 908** |
| Neaktivní řádky (`LIDAR`, `OOW_TCP`, `STOPPED`) | **489** (ponechány prázdné) |
| Projevené segmenty trasy | **Waypoint 1 až 41** (celkem 43 waypointů) |
| Přesná shoda `target_heading` (< $0.0001^\circ$) | **3 908 / 3 908 (100.0 %)** |
| Maximální numerická odchylka azimutu | **$0.00000003^\circ$** ($3 \cdot 10^{-8}$) |

### Běh 4: 11-35-55 (`temp/strom/11-35-55`)
| Metrika | Naměřená hodnota |
| :--- | :--- |
| Celkový počet zpracovaných řádků | **628** (100% shoda) |
| Aktivní navigační řádky (`source=NAV`) | **339** |
| Neaktivní řádky (`LIDAR`, `OOW_TCP`, `STOPPED`) | **289** (ponechány prázdné) |
| Projevené segmenty trasy | **Pouze Segment 10** (chybný přeskok na vzdálený segment o 52 m) |
| Přesná shoda `target_heading` (< $0.0001^\circ$) | **339 / 339 (100.0 %)** |
| Maximální numerická odchylka azimutu | **$0.00000001^\circ$** ($1 \cdot 10^{-8}$) |

---

## 4. Oprava v kódu `pilot_robotour`

V souboru [path_tracker.py](file:///e:/robotour/robot-platform/pilot_robotour/path_tracker.py) byla odstraněna problematická inicializační logika hledání „nejbližšího segmentu“:
1. **Odstraněno**:
   - Cyklus hledající `min_dist` napříč všemi segmenty trasy (který kolidoval s nekonečnými přímkami vzdálených úseků).
   - Vytváření umělých segmentů (`artificial_segment`).
2. **Nový stav**:
   - `PathTracker` se inicializuje přímo na Segment 0 (spojnice Waypoint 0 $\to$ Waypoint 1).
   - Kód se zkrátil o cca 40 řádků a je plně deterministický.
3. **Verifikace**:
   - Vytvořen nový unit test `test_path_tracker_always_starts_at_zero` v [test_pilot_robotour.py](file:///e:/robotour/robot-platform/pilot_robotour/test_pilot_robotour.py).
   - Všech 16 unit testů úspěšně prošlo.

Ukázka zrekonstruovaných hodnot v zatáčce z `navigate-10-41-07-enriched.csv`:
```csv
time,heading,target_heading,heading_error,near_lat,near_lon,geom_curvature,geom_radius_m,geom_turn_direction
1789807306.072,70.64,79.37,8.73,50.1039049322,14.4213641823,0.151755,6.59,RIGHT
1789807307.274,68.67,81.42,12.75,50.1039075069,14.4213778602,0.220714,4.53,RIGHT
```
Dokonale ukazuje postupné utahování poloměru oblouku z $6.59\text{ m}$ na $4.53\text{ m}$ při nájezdu do pravotočivé zatáčky.
