# Služba LIDAR-TRAVERSABILITY (Potenciálová mapa sjízdnosti terénu)

Služba `LIDAR-TRAVERSABILITY` běží jako nezávislé vlákno v rámci binárky `robot_lidar_tcp`. Z 3D mračna bodů LiDARu Unitree L2 a plné orientace a polohy robota ze služby `fusion` (`ipc:///tmp/robot-fusion`, topic `SOLUTION`) sestavuje na frekvenci **10 Hz** lokální 2D potenciálovou mapu sjízdnosti (`80 × 80` buněk po `10 × 10 cm` = `8.0 × 8.0 m`).

---

## 1. Fyzikální limity podvozku a terénu

- **Maximální přejezdný schod / obrubník (`max_step_cm`):** `4.0 cm` mezi sousedními buňkami (`10 cm`).
- **Maximální drsnost / nerovnost povrchu (`max_roughness_cm`):** `4.0 cm` (rozpětí `z_max - z_min` pojezdové vrstvy v buňce `10 × 10 cm`).
- **Maximální sklon terénu vůči gravitaci (`max_slope_deg`):** `25.0°` (sklon terénu po kompenzaci `pitch` a `roll` ze služby `fusion`).
- **Vertikální 3D překážka:** jakýkoliv bod ve výškovém pásmu `Z ∈ [12 cm, 130 cm]` nad lokální rovinou kol (stromy, sloupky, stěny, převislé větve).

---

## 2. Stabilizovaná rolující paměť (`140 × 140` buněk North-Up)

Aby se mapa při otáčení robota na místě (např. na křižovatce) nerozmazávala a aby si robot pamatoval i prostor za sebou a pod čumákem (mrtvý úhel před sedadlem):
1. Interní paměťová mřížka (`140 × 140` buněk po `10 cm` = `14 × 14 m`) je ukotvena **vůči severu (North-Up)**.
2. Posun mřížky o celočíselné násobky `10 cm` je řízen změnou WGS84 souřadnic `(lat, lon)` ze služby `fusion` (`SOLUTION`).
3. Pro výstup je z této stabilizované paměti vyříznuta a natočena podle aktuálního `heading` lokální mřížka `80 × 80` buněk v souřadném systému těla robota:
   - **Podélná osa `X` (`ix = 0..79`):** `[-2.0 m, +6.0 m)` (`2 m` za středem otáčení `CoR`, `6 m` vpřed). Pozice robota `CoR` leží na `ix = 20`.
   - **Příčná osa `Y` (`iy = 0..79`, CW):** `[-4.0 m, +4.0 m)` (`4 m` vlevo až `4 m` vpravo, kladný směr `+Y` je doprava CW). Pozice robota `CoR` leží na `iy = 40`.

---

## 3. Sjednocené potenciálové pole (`cost 0..100`)

Každá buňka výstupní mřížky `80 × 80` nese jednobajtovou potenciálovou cenu `uint8` v rozsahu `0..100`:
- **`0`:** Dokonale hladký a vodorovný povrch daleko od překážek.
- **`1..50`:** Sjízdné s rostoucí obtížností (zvýšená drsnost trávy/štěrku do `4 cm`, mírný sklon do `25°`).
- **`51..99`:** Odrazový (repulzní) polštář v okolí překážek, schodů a neviděných děr — kvadratický nárůst ceny ve vzdálenosti `35 cm .. 100 cm` od překážky, který přirozeně vytlačuje trajektorii (Elastic Band) do volného prostoru.
- **`100` (Nesjízdné / Zakázané):**
  - Buňka s překážkou (`Z ∈ [12, 130] cm`), schodem `> 4 cm`, drsností `> 4 cm` nebo sklonem `> 25°`.
  - **Inflace o poloměr robota (`35 cm`):** každá buňka ve vzdálenosti `≤ 35 cm` od nesjízdné buňky má rovněž cenu `100` (střed robota `CoR` tam nesmí vstoupit).
  - **Neviděná buňka (`!seen`):** buňky mimo dosah paprsků, za terénní hranou nebo pohlcené vodní plochou mají cenu `100` (ochrana proti pádu ze srázu a vjezdu do vody).

---

## 4. Rozhraní služby

### TCP Server (`127.0.0.1:9223`)
- `PING` $\rightarrow$ `PONG LIDAR-TRAVERSABILITY`
- `START` $\rightarrow$ `OK STARTED` (spustí 10 Hz vyhodnocování a ZMQ publikaci)
- `STOP` $\rightarrow$ `OK STOPPED` (zastaví vyhodnocování)
- `STATUS` $\rightarrow$ `<STATE> <JSON>` (diagnostika: počet platných/blokovaných buněk, volný koridor, úhly `heading`/`pitch`/`roll`, doba výpočtu)
- `TRAVERSABILITY` $\rightarrow$ vrátí poslední vyhodnocený JSON payload (nebo `ERR NOT_READY`)
- `EXIT` $\rightarrow$ `BYE`
- `SHUTDOWN` $\rightarrow$ ukončí proces (pouze pokud jsou všechny služby `LIDAR`, `LIDAR-COLLISIONS` i `LIDAR-TRAVERSABILITY` zastavené)

### ZMQ Publisher (`ipc:///tmp/robot-lidar-traversability`)
- **Topic:** `TRAVERSABILITY` (10 Hz)
- **Formát JSON payloadu:**
```json
{
  "timestamp": 1775417510.123,
  "heading": 124.5,
  "pitch": -1.2,
  "roll": 0.4,
  "grid_nx": 80,
  "grid_ny": 80,
  "cell_size_cm": 10,
  "origin_ix": 20,
  "origin_iy": 40,
  "valid_cells": 2450,
  "blocked_cells": 410,
  "min_corridor_clearance_cm": 340.0,
  "grid_b64": "dGVzdC1iYXNlNjQtYmluYXJ5LTY0MDAtYnl0ZXM=..."
}
```
Pole `grid_b64` obsahuje Base64-zakódované pole `uint8[6400]` v row-major pořadí `ix * 80 + iy`.

