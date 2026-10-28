# Kolizní kurzy a prostorové vyhodnocení překážek (LiDAR L2)

Implementace služby kolizních kurzů **`LIDAR-COLLISIONS`** (TCP port 9222) běží v rámci jednoho společného procesu služby **`LIDAR`** (TCP port 9002). To umožňuje optimalizaci výkonu a sdílení dat mezi oběma službami.

---

## 1. Klíčové principy

1. **Eliminace těla robota (pevný obrys šasi):**
   - Při kalibraci (`CALIBRATE` na portu 9002) se kalibruje pouze rovina země (`T_CL`).
   - Maska vlastního těla robota (`ignoreBox`) má v konfiguraci pevně dané geometrické rozměry šasi v souřadnicích LiDARu: `x ∈ [-0.70, +0.23] m`, `y ∈ [-0.35, +0.35] m` (eliminuje riziko falešné slepé zóny při přítomnosti předmětů blízko robota během kalibrace).
2. **Vnitřní výpočet stabilizovaný vůči severu (North-Up CW):**
   - Vnitřní mřížka má **12 výškových pater po 10 cm** ($Z \in [10, 130)\text{ cm}$) a **72 azimutálních sektorů po $5^\circ$** ($360^\circ$, $0^\circ$ = sever, clockwise).
   - V každé buňce běží $\alpha$-$\beta$ filtr pro vzdálenost $d_{est}$ a rychlost přibližování $v_{est}$ a zároveň $\alpha$-$\beta$ filtr pro přesný azimut $\theta_{est}$ a úhlovou rychlost $\omega_{est}$.
   - **Princip námořních kolizních kurzů (CBDR – Constant Bearing, Decreasing Range):**
     Ze složek radiální rychlosti přibližování $v_r$ a tečné rychlosti $v_t = d \cdot |\omega|$ se určuje nejbližší vzdálenost minutí $d_{CPA}$. Do kolizního kurzu (`min_ttc_s`) vstupují pouze objekty v přední polokouli robota ($-90^\circ\text{ až }+90^\circ$), které mají alespoň 3 zásahy (`min_hits_for_ttc = 3`) a jejichž trajektorie protíná bezpečnostní válec těla robota ($d_{CPA} \le R_{robot} = 37.5\text{ cm}$). Stromy a sloupky míjené vedle cesty mají $d_{CPA} > 37.5\text{ cm}$ a nevyvolávají falešný kolizní poplach.
3. **Výstup v souřadnicích těla robota (Body-Up CW):**
   - Výstupní pole mají **72 prvků po $5^\circ$**, kde index `0` odpovídá směru přímo vpřed ($[0^\circ, 5^\circ)$) a rotace je **clockwise** ($90^\circ$ = vpravo, $180^\circ$ = vzad, $270^\circ$ = vlevo).
   - Protože LiDAR vidí pouze přední polokouli ($180^\circ$: indexy `0..17` pro $[0^\circ, 90^\circ)$ a `54..71` pro $[270^\circ, 360^\circ)$), zadní slepé sektory (`18..53`, tj. $[90^\circ, 270^\circ)$) nesou v `distances_cm` hodnotu **`-1`** a v `delta_d_cms` hodnotu **`0`**.
4. **Predikce pohybu překážky prostým součtem:**
   - Pole `delta_d_cms[72]` udává očekávanou změnu vzdálenosti v daném sektoru za $1\text{ s}$ v $\text{cm/s}$ (záporná hodnota = přibližování, včetně bočního přechodu překážky ze sousedního sektoru).
   - Predikovaná vzdálenost za $1\text{ s}$ v `pilot_maraton` je tak prostý součet: `d_pred_1s[i] = distances_cm[i] + delta_d_cms[i]`.

---

## 2. Výstupní JSON (`COLLISIONS`)

Publikováno s frekvencí **10 Hz** na ZMQ `ipc:///tmp/robot-lidar-collisions` (topic `COLLISIONS`) a dostupné na TCP portu `9222` příkazem `COLLISIONS` (automaticky logováno službou `logger`):

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

| Pole | Typ | Jednotka | Popis |
| :--- | :--- | :--- | :--- |
| **`timestamp`** | `float` | `s` | Časová značka vyhodnocení |
| **`heading`** | `float` | `deg CW` | Průměrný kompasový kurz robota během snímku ($0^\circ$ = sever) |
| **`min_ttc_s`** | `float` | `s` | Nejkratší čas do kolize (CBDR) v přední polokouli (`999.0` pokud není kolizní kurz) |
| **`min_ttc_angle_deg`** | `float` | `deg CW` | Směr nejbližší kolizní hrozby v těle robota ($0^\circ$ = vpřed, CW; `-1.0` pokud není hrozba) |
| **`distances_cm`** | `int[72]` | `cm` | Nejbližší vzdálenost napříč 12 patry pro 72 sektorů po $5^\circ$ (`2000` = volno, `-1` = slepý sektor $90^\circ..270^\circ$) |
| **`delta_d_cms`** | `int[72]` | `cm/s` | Změna vzdálenosti za $1\text{ s}$ (záporná = přibližování; predikce za $1\text{ s}$ je `distances_cm[i] + delta_d_cms[i]`) |

---

## 3. Napojení v `pilot_maraton` a pravidlo $\frac{2}{5}\,\text{TTC}$

Služba `pilot_maraton` odebírá `COLLISIONS` z `ipc:///tmp/robot-lidar-collisions` (a ponechává si `DISTANCE` z `ipc:///tmp/robot-lidar` jako záložní pojistku):
- **Zastavení na $0.7\text{ m}$ ($70\text{ cm}$):** Při překážce $\le 70\text{ cm}$ v přímém koridoru robota (šířka $75\text{ cm}$) nastaví cílovou rychlost na $0\text{ m/s}$.
- **Plná rychlost až $2.0\text{ m/s}$ při volnu $\ge 6.0\text{ m}$ ($600\text{ cm}$):** Mezi $0.7\text{ m}$ a $6.0\text{ m}$ plynule omezuje maximální rychlost.
- **Pravidlo zastavení za $\frac{2}{5}\,\text{TTC}$:** Robot musí být schopen zastavit v čase $t_{stop} = \frac{2}{5}\cdot\text{min\_ttc\_s}$ (čímž nechává protijedoucímu objektu čas zastavit a drží $\frac{1}{5}$ rezervu). Při predikovatelném brzdění $a_{nom} = 2\text{ m/s}^2$ omezuje rychlost na $v_{max} = a_{nom} \cdot \frac{2}{5}\,\text{TTC}$ a v případě nutnosti dynamicky zvyšuje brzdnou deceleraci až na nouzové maximum $0.4g \approx 4\text{ m/s}^2$.
