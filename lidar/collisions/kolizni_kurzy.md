# Kolizní kurzy a prostorové vyhodnocení překážek (LiDAR L2)

Tento dokument specifikuje návrh a implementaci služby kolizních kurzů **`LIDAR-COLLISIONS`** (TCP port 9222). Služba běží v rámci jednoho společného procesu s nízkoúrovňovým ovladačem **`LIDAR`** (TCP port 9002), ale disponuje vlastní sadou vláken a plně odděleným řízením, diagnostikou i publikační sběrnicí.

---

## 1. Cíle a klíčové principy

1. **Vnitřní globální orientace (North-Up CW) a výstup v těle robota (Body-Up CW):**
   - Středem soustavy je střed otáčení robota (Center of Rotation – CoR).
   - Vnitřní mřížka **12 pater × 72 sektorů (po $5^\circ$)** je stabilizována vůči severu ($0^\circ$ = sever, clockwise), takže rotace robota nemá vliv na sledování rychlosti a směru přibližujících se předmětů.
   - Výstupní pole `distances_cm[72]` a `delta_d_cms[72]` se přemapovávají do **těla robota** ($0^\circ$ = vpřed, clockwise). Zadní slepá polokoule ($90^\circ..270^\circ$, indexy `18..53`) má v `distances_cm` hodnotu `-1`.
2. **Výšková stratifikace (12 pater):**
   - Válcovitá oblast kolem robota je rozdělena do 12 pater po 10 cm ($Z \in [10, 130)$ cm od země, kde $Z = 0$ je úroveň terénu).
   - Spodní patro (10–20 cm) nezakryje překážky ve vyšších patrech.
3. **Dynamické sledování a princip kolizních kurzů (CBDR):**
   - V každé buňce běží $\alpha$-$\beta$ filtr sledující vzdálenost $d_{est}$ a rychlost přibližování $v_{est}$ ($v_{est} > 0$ přibližování) a zároveň přesný azimut $\theta_{est}$ a jeho úhlovou rychlost $\omega_{est}$.
   - **CBDR (Constant Bearing, Decreasing Range):** Ze změny azimutu $\omega_{est}$ a rychlosti přibližování $v_{est}$ se určí nejbližší vzdálenost minutí $d_{CPA}$. Objekt v přední polokouli je na kolizním kurzu s robotem pouze tehdy, pokud $d_{CPA} \le R_{robot}$ ($37.5\text{ cm}$) po dobu alespoň 3 snímků (`min_hits_for_ttc = 3`). Tím jsou eliminovány falešné poplachy od stromů a sloupků míjených vedle cesty.
4. **Pevná kalibrační maska těla robota:**
   - Při příkazu `CALIBRATE` se kalibruje pouze transformace roviny země `T_CL`. Obrys vlastního šasi robota (`ignoreBox`) je dán pevnými geometrickými rozměry v souřadnicích LiDARu: `x ∈ [-0.70, +0.23] m`, `y ∈ [-0.35, +0.35] m`.

---

## 2. Výstupní JSON (`COLLISIONS`)

Publikováno na `ipc:///tmp/robot-lidar-collisions` (topic `COLLISIONS`, 10 Hz) a logováno službou `logger`:

```json
{
  "timestamp": 1729312000.123,
  "heading": 45.4,
  "min_ttc_s": 2.45,
  "min_ttc_angle_deg": 10.0,
  "distances_cm": [2000, 2000, 540, ...],
  "delta_d_cms": [0, 0, -180, ...]
}
```
- **`distances_cm[72]`**: Nejbližší vzdálenost v cm pro každý $5^\circ$ sektor v těle robota (`0` = vpřed, CW; `2000` = volno, `-1` = slepý sektor $90^\circ..270^\circ$).
- **`delta_d_cms[72]`**: Změna vzdálenosti za $1\text{ s}$ v cm/s (záporná = přibližování). Predikce za $1\text{ s}$ je prostý součet `distances_cm[i] + delta_d_cms[i]`.
- **`min_ttc_s`** a **`min_ttc_angle_deg`**: Nejkratší čas do kolize v přední polokouli a úhel hrozby v těle robota (CW).
