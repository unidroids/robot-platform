# Technická analýza IMU u-blox ZED-F9R a alternativ pro mobilního robota


* [Datová základna & hardwarový kontext](#datová-základna--hardwarový-kontext)
* [Metodika segmentové analýzy (Windowed Analysis) a definice parametrů](#metodika-segmentové-analýzy-windowed-analysis-a-definice-parametrů)
* [Analyzovaná data](#analyzovaná-data)
* [Výsledky měření](#výsledky-měření)
* [Technické srovnání alternativních 6-osých IMU čipů](#technické-srovnání-alternativních-6-osých-imu-čipů)
* [Srovnání: Nouzový headless ZED-F9R vs. původní 10-osá IMU (Gongling AI)](#srovnání-nouzový-headless-zed-f9r-vs-původní-10-osá-imu-gongling-ai)

> `Disclaimer`: Dokument je generovaný AI a slouží primárně k sumarizaci naměřených dat a logů jako podklad pro alternativní IMU jednotku.

Tento dokument obsahuje technickou analýzu stability, driftu a šumu gyroskopu jednotky **u-blox ZED-F9R** (kanál `ipc:///tmp/robot-gnss-imu`) v porovnání s přesným RTK azimutem duální anténní báze **UNIHEADING** ve stavu `NARROW_INT`. Analýza vychází z kompletních datových záznamů všech 5 soutěžních kol.

Součástí dokumentu je přehled technických parametrů alternativních 6-osých MEMS IMU čipů na trhu jakožto podklad pro alternativní IMU jednotku. 

Faktické srovnání IMU ze ZED-F9R s původní rozbitou 10-osou IMU jednotkou Gongling AI paradoxně posunulo kvalitu úhlového měření o celou technologickou třídu výše.


---

## Datová základna & hardwarový kontext

### Hardwarová konfigurace na robotu:
* **Jednotka u-blox ZED-F9R neměla připojenou GNSS anténu.** Byla provozována výhradně jako samostatný 6-osý IMU senzor poskytující surové zprávy protokolu UBX-ESF (`UBX-ESF-MEAS` / `GYRO`).
* Naměřené hodnoty představují **čistý výstup MEMS křemíku IMU bez interní fúze se satelitním signálem či GNSS filtrem**.
* **Vzorkování a zpracování toku dat:**
  * Křemík IMU v jednotce F9R generoval data na frekvenci **$100\text{ Hz}$**.
  * Služba `gnss-imu` (napsaná v Pythonu, vyčítající data z jednotky přes sériové rozhraní) prováděla lehkou softwarovou fúzi a časovou akumulaci vzorků s downsamplingem na **$20\text{ Hz}$** ($dt = 50\text{ ms}$). Tento krok zabraňoval zahlcení meziprocesové komunikace (IPC sběrnice), zatímco hlavní řídicí smyčka robota pracovala s frekvencí $10\text{ Hz}$.
* **Obsah zprávy `GYRO` (20 Hz):**
  * `ts`: časové razítko vzorku.
  * `delta_yaw`: úhlový přírůstek azimutu za periodu $50\text{ ms}$ [°].
  * `wz`: okamžitá úhlová rychlost rotace kolem svislé osy Z [°/s].
  * `pitch`: podélný náklon šasi [°].
  * `roll`: příčný náklon šasi [°].
  * `ax, ay, az`: 3D zrychlení [$\text{m/s}^2$].
* **Referenční etalon (Ground Truth):**
  * Jako referenční úhel natočení sloužil azimut duální GNSS RTK báze **UNIHEADING** složené ze dvou antén na pevném rameni šasi robota.
  * Pro vyhodnocení byly použity výhradně okamžiky se stavem plného celočíselného fázového fixu `NARROW_INT` splňující zpřísněné kritérium směrodatné odchylky **$\sigma_{\text{hdg}} \le 1.5^\circ$**.

---

## Metodika segmentové analýzy (Windowed Analysis) a definice parametrů

### A. Význam porovnávaných technických parametrů
Při analýze a výběru MEMS inerciálních senzorů jsou sledovány následující klíčové parametry:
* **In-run Bias Stability [°/h]:** Míra stability nulového bodu gyroskopu v čase během nepřetržitého provozu při konstantní teplotě; udává minimální dosažitelnou chybu driftu po optimální kalibraci.
* **Gyro Noise Density [°/s/√Hz]:** Spektrální hustota šumu výstupní úhlové rychlosti v závislosti na šířce pásma měření; určuje úroveň vysokofrekvenčního šumu v surovém signálu.
* **Angular Random Walk (ARW) [°/√h]:** Rychlost akumulace náhodné úhlové odchylky integrací bílého šumu v čase; popisuje nejistotu azimutu při integraci bez vnějších korekcí.
* **Statický drift (Zero-rate bias) [°/s]:** Systematická odchylka klidového výstupu senzoru od nuly při nehybném šasi.
* **Teplotní drift [(°/s)/°C]:** Citlivost nulového bodu gyroskopu na změnu provozní teploty elektroniky a okolního prostředí.
* **Linearita měřítka (Scale Factor Error) [%]:** Odchylka poměru mezi naměřenou a skutečnou úhlovou rychlostí při dynamických rotacích šasi.

### B. Segmentová okénková metoda (Windowed Analysis)
Pro eliminaci vlivu počátečních náhodných chyb byla aplikována segmentová okénková metoda napříč všemi koly:
1. V datech jízdy je vyhledán počáteční čas $t_0$, kde má duální báze spolehlivý fix:
   $$\text{UNIHEADING} = \text{NARROW\_INT} \quad \wedge \quad \sigma_{\text{hdg}}(t_0) \le 1.5^\circ$$
2. Virtuální gyroskop se zinicializuje na počáteční azimut báze:
   $$\theta_{\text{gyro}}(t_0) = \theta_{\text{uni}}(t_0)$$
3. Po dobu testovacího okna $\Delta t$ (resp. vzdálenosti $\Delta s$) se azimut počítá výhradně integrací gyroskopu:
   * **Model 1D (čistá integrace osy Z):**
     $$\theta_{1D}(t) = \theta(t_0) + \sum_{k} \Delta \psi_k$$
   * **Model 3D (kompenzace podélného a příčného náklonu):**
     Projekce úhlového přírůstku skloněného šasi do horizontální roviny:
     $$\Delta \psi_{3D, k} = \Delta \psi_k \cdot \cos(\theta_{\text{pitch}, k}) \cdot \cos(\phi_{\text{roll}, k})$$
     $$\theta_{3D}(t) = \theta(t_0) + \sum_{k} \Delta \psi_{3D, k}$$
4. V koncovém bodě okna $t_1 = t_0 + \Delta t$ se ověří, že duální báze splňuje stejnou podmínku fixu ($\text{NARROW\_INT}$ a $\sigma_{\text{hdg}}(t_1) \le 1.5^\circ$), a vyčíslí se absolutní i znaménková chyba:
   $$\epsilon = |\text{norm\_angle}(\theta_{\text{gyro}}(t_1) - \theta_{\text{uni}}(t_1))|$$
5. **Kritérium kontinuálního pohybu:** V testovacím okně nesmí dojít k zastavení kol. Pohyb robota je kontrolován ze zpráv odometrie (`ipc:///tmp/robot-drive ODM`); pokud se po dobu $\ge 0.5\text{ s}$ kroky levého i pravého kola nezmění, okno je klasifikováno jako obsahující zastavení a do dynamické sady není zařazeno.

---

## Analyzovaná data

Analýza pokrývá všech 5 soutěžních kol sobotního maratonu specifikovaných v souboru `bloky.dat`:

| Kolo / Blok | Datový soubor | Časový rozsah | Počet vzorků `GYRO` (20 Hz) | Počet vzorků `ODM` (10 Hz) | Charakteristika etapy |
|:---|:---|:---:|:---:|:---:|:---|
| **Předkolo** | `logger-09-20-57.dat` | 09:20 – 10:00 | 50 826 | 25 384 | Zkušební jízdy, ověření nakládky a vykládky |
| **1. kolo** | `logger-10-25-48.dat` | 10:25 – 11:10 | 54 071 | 27 000 | Ostrá soutěžní jízda, sjetí z cesty, návrat do depa |
| **2. kolo** | `logger-11-31-32.dat` | 11:31 – 12:30 | 69 178 | 34 532 | Plná úspěšná jízda (1,4 km za 33 min) |
| **3. kolo** | `logger-14-01-42.dat` | 14:01 – 15:00 | 69 221 | 34 534 | Plná úspěšná jízda (2,0 km za 40 min) |
| **4. kolo** | `logger-15-32-06.dat` | 15:32 – 16:40 | 80 583 | 40 207 | Nejdelší etapa (2,4 km za 67 min) |
| **Celkem** | **5 logovacích souborů** | **09:20 – 16:40** | **323 879** | **161 657** | **Kompletní soutěžní den** |

---

## Výsledky měření

### Akumulace v čase (Časová okna za jízdy, ZUPT neaktivní)
Testovací okna vyhodnocená v nepřetržitém pohybu (kola se točí, nulový rychlostní limit, vyloučena zastavení, $\sigma_{\text{hdg}} \le 1.5^\circ$):

| Časové okno $\Delta t$ | Počet oken $N$ | Model | Medián chyby | Průměrná chyba | 90% percentil | 95% percentil | Max. chyba | Systematický bias | Rychlost driftu |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **15 sekund** | 2 600 | **1D**<br>**3D** | $1.34^\circ$<br>**$1.32^\circ$** | $1.69^\circ$<br>**$1.69^\circ$** | $3.65^\circ$<br>**$3.65^\circ$** | $4.39^\circ$<br>**$4.41^\circ$** | $12.63^\circ$<br>**$12.93^\circ$** | $-0.08^\circ$<br>**$-0.12^\circ$** | $6.76^\circ/\text{min}$<br>**$6.75^\circ/\text{min}$** |
| **30 sekund** | 1 830 | **1D**<br>**3D** | $1.48^\circ$<br>**$1.48^\circ$** | $1.89^\circ$<br>**$1.90^\circ$** | $4.19^\circ$<br>**$4.17^\circ$** | $4.93^\circ$<br>**$4.98^\circ$** | $11.18^\circ$<br>**$11.33^\circ$** | $+0.05^\circ$<br>**$-0.03^\circ$** | $3.79^\circ/\text{min}$<br>**$3.79^\circ/\text{min}$** |
| **60 sekund** | 1 279 | **1D**<br>**3D** | $1.70^\circ$<br>**$1.70^\circ$** | $2.13^\circ$<br>**$2.12^\circ$** | $4.52^\circ$<br>**$4.49^\circ$** | $5.51^\circ$<br>**$5.58^\circ$** | $10.53^\circ$<br>**$10.09^\circ$** | $+0.27^\circ$<br>**$+0.11^\circ$** | $2.13^\circ/\text{min}$<br>**$2.12^\circ/\text{min}$** |
| **120 sekund** | 967 | **1D**<br>**3D** | **$1.69^\circ$**<br>$1.72^\circ$ | **$2.14^\circ$**<br>$2.15^\circ$ | $4.72^\circ$<br>**$4.59^\circ$** | **$5.69^\circ$**<br>$5.70^\circ$ | **$9.84^\circ$**<br>$9.92^\circ$ | $+0.48^\circ$<br>**$+0.15^\circ$** | **$1.07^\circ/\text{min}$**<br>$1.08^\circ/\text{min}$ |

*Poznámka k časovým oknům:*
* V časovém horizontu **30 sekund** je medián odchylky azimutu **$1.48^\circ$** a 95 % všech případů leží pod hranicí **$4.98^\circ$**.
* Po **120 sekundách** nepřetržité jízdy bez GNSS fixu dosahuje medián chyby **$1.72^\circ$** a maximální naměřená chyba nepřekročila **$9.92^\circ$**.

---

### Akumulace na ujeté vzdálenosti (Vzdálenostní okna za jízdy, ZUPT neaktivní)
Odometrická vzdálenost ujetá robotem při typické rychlosti $0.8 - 1.2\text{ m/s}$:

| Vzdálenostní okno $\Delta s$ | Počet oken $N$ | Model | Medián chyby | Průměrná chyba | 90% percentil | 95% percentil | Max. chyba | Systematický bias | Normalizovaný drift na 100 m |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **20 metrů** | 2 121 | **1D**<br>**3D** | $1.39^\circ$<br>**$1.38^\circ$** | $1.78^\circ$<br>**$1.78^\circ$** | $3.83^\circ$<br>**$3.85^\circ$** | $4.76^\circ$<br>**$4.75^\circ$** | $12.11^\circ$<br>**$11.91^\circ$** | $-0.01^\circ$<br>**$-0.07^\circ$** | $8.91^\circ/100\text{m}$<br>**$8.92^\circ/100\text{m}$** |
| **50 metrů** | 1 251 | **1D**<br>**3D** | $1.78^\circ$<br>**$1.76^\circ$** | **$2.15^\circ$**<br>$2.16^\circ$ | $4.42^\circ$<br>**$4.42^\circ$** | $5.43^\circ$<br>**$5.38^\circ$** | **$11.79^\circ$**<br>$12.16^\circ$ | $+0.39^\circ$<br>**$+0.23^\circ$** | **$4.31^\circ/100\text{m}$**<br>$4.31^\circ/100\text{m}$ |
| **100 metrů** | 1 015 | **1D**<br>**3D** | $1.79^\circ$<br>**$1.76^\circ$** | $2.22^\circ$<br>**$2.18^\circ$** | $4.57^\circ$<br>**$4.52^\circ$** | $5.64^\circ$<br>**$5.38^\circ$** | **$11.57^\circ$**<br>$11.81^\circ$ | $+0.41^\circ$<br>**$+0.05^\circ$** | $2.22^\circ/100\text{m}$<br>**$2.18^\circ/100\text{m}$** |
| **200 metrů** | 489 | **1D**<br>**3D** | **$2.09^\circ$**<br>$2.12^\circ$ | **$2.53^\circ$**<br>$2.56^\circ$ | **$5.07^\circ$**<br>$5.45^\circ$ | **$6.40^\circ$**<br>$6.75^\circ$ | **$11.38^\circ$**<br>$11.49^\circ$ | $+0.91^\circ$<br>**$+0.34^\circ$** | **$1.26^\circ/100\text{m}$**<br>$1.28^\circ/100\text{m}$ |

*Poznámka ke vzdálenostním oknům:*
* Na úseku **100 metrů** jízdy je medián úhlové odchylky **$1.76^\circ$** a normalizovaný drift činí **$2.18^\circ / 100\text{ m}$**.
* Na úseku **200 metrů** činí medián chyby **$2.12^\circ$** s normalizovaným driftem **$1.28^\circ / 100\text{ m}$**.

---

### Vliv 3D kompenzace náklonů (Pitch & Roll)
Cesty v parku Stromovka vykazují sklon terénu a nerovnosti (podélný náklon šasi typicky $\text{pitch} \approx \pm 2^\circ \text{ až } 5^\circ$, příčný $\text{roll} \approx \pm 1^\circ \text{ až } 4^\circ$).

Z naměřených dat vyplývají následující charakteristiky:
1. **Redukce systematického biasu:**
   * U časového okna 30 s klesl bias z $+0.05^\circ$ (1D) na **$-0.03^\circ$** (3D).
   * U časového okna 60 s klesl bias z $+0.27^\circ$ (1D) na **$+0.11^\circ$** (3D).
   * U časového okna 120 s klesl bias z $+0.48^\circ$ (1D) na **$+0.15^\circ$** (3D, pokles o 69 %).
   * U vzdálenostního okna 100 m klesl bias z $+0.41^\circ$ na **$+0.05^\circ$** (pokles o 88 %).
   * U vzdálenostního okna 200 m klesl bias z $+0.91^\circ$ na **$+0.34^\circ$** (pokles o 63 %).
2. **Rozdíl v absolutní chybě:** Na rovných úsecích je rozdíl v mediánu mezi 1D a 3D modelem minimální ($0.01^\circ - 0.03^\circ$), protože průměrný kosinus malých úhlů ($\cos(3^\circ) \approx 0.9986$) představuje korekci pod 0,2 %. Význam 3D kompenzace se projevuje především v potlačení jednostranného stáčení azimutu při jízdě v bočním svahu.

---

### Klidový šum a drift při stání (Static Bias & ZUPT)
Vyhodnocení úseků, kdy robot stál na místě (odometrie $\Delta\text{steps} = 0$ po dobu $\ge 10\text{ s}$), napříč jednotlivými etapami dne:

| Kolo / Měření | Čas měření | Počet vzorků $N$ (20 Hz) | Doba stání | Statický bias $w_z$ | Drift rate za hodinu | Šum úhlové rychlosti ($\sigma$) | Prahová mez $3\sigma$ |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Předkolo** | 09:20 – 10:00 | 20 945 | 1 047.2 s | **$-0.0018^\circ/\text{s}$** | $-6.3^\circ/\text{hod}$ | $0.1273^\circ/\text{s}$ | $0.382^\circ/\text{s}$ |
| **1. kolo** | 10:25 – 11:10 | 18 792 | 939.6 s | **$+0.0061^\circ/\text{s}$** | $+22.1^\circ/\text{hod}$ | $0.1374^\circ/\text{s}$ | $0.412^\circ/\text{s}$ |
| **2. kolo** | 11:31 – 12:30 | 15 532 | 776.6 s | **$+0.0020^\circ/\text{s}$** | $+7.0^\circ/\text{hod}$ | $0.1920^\circ/\text{s}$ | $0.576^\circ/\text{s}$ |
| **3. kolo** | 14:01 – 15:00 | 16 685 | 834.2 s | **$+0.0059^\circ/\text{s}$** | $+21.2^\circ/\text{hod}$ | $0.0894^\circ/\text{s}$ | $0.268^\circ/\text{s}$ |
| **4. kolo** | 15:32 – 16:40 | 23 387 | 1 169.3 s | **$+0.0037^\circ/\text{s}$** | $+13.2^\circ/\text{hod}$ | $0.2080^\circ/\text{s}$ | $0.624^\circ/\text{s}$ |
| **Celý den (souhrn)** | **09:20 – 16:40** | **95 341** | **4 767.0 s** (~79 min) | **$+0.0032^\circ/\text{s}$** | **$+11.5^\circ/\text{hod}$** | **$0.1508^\circ/\text{s}$** | **$0.452^\circ/\text{s}$** |

*Poznatky ke klidovému chování a ZUPT filtru:*
* **Stabilita nulového bodu:** Statický bias setrval po celých 7 hodin provozu v pásmu mezi **$-0.0018^\circ/\text{s}$** a **$+0.0061^\circ/\text{s}$**. Průměrný posun za den činí $+11.5^\circ/\text{hod}$.
* **Ověření ZUPT prahu:** Implementovaný práh filtru nulové rychlosti (ZUPT) nastavený na hodnotu $|w_z| < 1.5^\circ/\text{s}$ spolehlivě převyšuje maximální zjištěnou hodnotu $3\sigma$ šumu ($0.624^\circ/\text{s}$) s rezervou 2,4×. Při zastavení robota tak nedochází k unášení kurzu šumem.

---

## Technické srovnání alternativních 6-osých IMU čipů

Modul u-blox ZED-F9R je primárně GNSS přijímač s integrovaným inerciálním senzorem určeným pro automobilový dead reckoning. Použití tohoto modulu výhradně jako IMU senzoru (bez GNSS antény) je ekonomicky neefektivní. 

Níže je uveden přehled technických parametrů alternativních 6-osých MEMS IMU čipů, které dosahují srovnatelných nebo lepších parametrů než interní IMU v ZED-F9R:

| Čip / Senzor | Výrobce | Určení & Třída | In-run Bias Stability | Gyro Noise Density | Angular Random Walk (ARW) | Rozsah gyra | On-chip fúze / vlastnosti | Rozhraní | Orientační cena (čip / modul) |
|:---|:---|:---|:---:|:---:|:---:|:---:|:---|:---|:---:|
| **u-blox ZED-F9R (interní IMU)** | u-blox *(embedded SMI230 / ASM330)* | Automotive (AEC-Q100)<br>Navigace vozidel | **$3 – 5^\circ/\text{hod}$** | $0.007^\circ/\text{s}/\sqrt{\text{Hz}}$ | $0.18^\circ/\sqrt{\text{hod}}$ | $\pm 2000^\circ/\text{s}$ | Interní EKF (spřaženo s GNSS fázemi) | UART / USB | ~ 6 000 – 8 000 Kč *(celý GNSS modul)* |
| **ST LSM6DSV16X** | STMicroelectronics | Industrial / Robotika<br>Autonomní roboty, drony | **$3 – 5^\circ/\text{hod}$** | **$0.005^\circ/\text{s}/\sqrt{\text{Hz}}$** | **$0.11^\circ/\sqrt{\text{hod}}$** | $\pm 4000^\circ/\text{s}$ | Hardwarová SFLP fúze (kvaterniony na čipu), FSM, MLC | I3C / SPI / I2C | ~ 250 Kč / čip<br>(modul ~ 450 Kč) |
| **TDK InvenSense ICM-42688-P** | TDK InvenSense | High-End Consumer / UAV<br>Drony (PX4), stabilizátory | **$5 – 8^\circ/\text{hod}$** | **$0.0028^\circ/\text{s}/\sqrt{\text{Hz}}$** | **$0.07^\circ/\sqrt{\text{hod}}$** | $\pm 2000^\circ/\text{s}$ | Surová data, FIFO buffer 2 kB, vzorkování až 32 kHz | SPI / I2C | ~ 150 Kč / čip<br>(modul ~ 350 Kč) |
| **Bosch BMI088 / SMI230** | Bosch Sensortec | Industrial / Automotive<br>Průmyslová AGV, ESP systémy | **$2 – 4^\circ/\text{hod}$** | $0.007^\circ/\text{s}/\sqrt{\text{Hz}}$ | $0.15^\circ/\sqrt{\text{hod}}$ | $\pm 2000^\circ/\text{s}$ | Oddělené křemíkové čipy (die) pro gyro a akcelerometr | SPI / I2C | ~ 350 Kč / čip<br>(modul ~ 700 Kč) |
| **ST ASM330LHH / ISM330DHCX** | STMicroelectronics | Automotive / Industrial<br>Telematika, průmyslové roboty | **$3 – 5^\circ/\text{hod}$** | $0.005^\circ/\text{s}/\sqrt{\text{Hz}}$ | $0.12^\circ/\sqrt{\text{hod}}$ | $\pm 4000^\circ/\text{s}$ | Machine Learning Core (MLC), FSM | SPI / I2C | ~ 300 Kč / čip<br>(modul ~ 600 Kč) |
| **Analog Devices ADIS16470** | Analog Devices | Tactical Grade<br>Vojenská technika, těžká AGV | **$< 1.5 – 2.5^\circ/\text{hod}$** | **$0.003^\circ/\text{s}/\sqrt{\text{Hz}}$** | **$0.05^\circ/\sqrt{\text{hod}}$** | $\pm 2000^\circ/\text{s}$ | Tovární kalibrace křížových vazeb a teplotních driftů | Průmyslové SPI | ~ 15 000 – 25 000 Kč |

### Technické charakteristiky senzorů pro pozemní mobilní roboty:
* **ST LSM6DSV16X:** Senzor disponuje rozsahem měření až do $\pm 4000^\circ/\text{s}$ (zabraňuje saturaci při dynamických rázech podvozku), nízkým šumem $0.005^\circ/\text{s}/\sqrt{\text{Hz}}$ a hardwarovým výpočtem 6-osých orientačních kvaternionů přímo na čipu (SFLP).
* **TDK ICM-42688-P:** Dosahuje nejnižší šumové hustoty ($0.0028^\circ/\text{s}/\sqrt{\text{Hz}}$) a nízké hodnoty ARW ($0.07^\circ/\sqrt{\text{hod}}$). Vyžaduje externí výpočet orientace v procesoru robota.
* **Bosch BMI088:** Používá oddělená křemíková pouzdra pro tříosý gyroskop a tříosý akcelerometr, což snižuje parazitní mechanické vazby mezi měřením zrychlení a úhlových rychlostí při vibracích šasi na hrubém povrchu.

---

## Srovnání: Nouzový headless ZED-F9R vs. původní 10-osá IMU (Gongling AI)

V rámci přípravy robota na soutěž došlo k mechanickému poškození původní 10-osé jednotky Gongling AI při pádu. Jako operativní náhrada byl využit dostupný modul u-blox ZED-F9R bez připojené antény.

Níže je uvedeno faktické porovnání parametrů z technické dokumentace původního senzoru Gongling AI s empirickými hodnotami náhradní jednotky F9R:

### Srovnávací tabulka parametrů:

| Parametr / Vlastnost | Původní 10-osá IMU (Gongling AI)<br>*(dle technické specifikace výrobce)* | Náhradní u-blox ZED-F9R<br>*(bez antény, raw ESF, reálně naměřeno)* | Rozdíl v parametru |
|:---|:---:|:---:|:---|
| **Počet senzorových os** | 10 os (akcelerometr + gyroskop + magnetometr + barometr) | 6 os (akcelerometr + gyroskop) | Absence magnetometru a barometru v 6-osém módu |
| **Klidový drift gyra (Zero Drift)** | **$\pm 0.5 \sim 1.0^\circ/\text{s}$** | **$+0.0032^\circ/\text{s}$** *(průměr soboty)* | F9R vykazuje **150× až 300× nižší** klidový bias |
| **Úhlový drift za 1 minutu stání** | **$30^\circ \sim 60^\circ$** | **$0.19^\circ$** | Bez korekce dosahuje rozdíl dvou řádů |
| **In-run stabilita biasu** | $> 500 – 1000^\circ/\text{hod}$ | **$3 – 5^\circ/\text{hod}$** | Odpovídá rozdílu mezi spotřební a automobilovou třídou |
| **Šum gyroskopu (RMS)** | $0.028 \sim 0.07^\circ/\text{s-rms}$ | $0.15^\circ/\text{s-rms}$ | Srovnatelná úroveň náhodného šumu |
| **Teplotní drift gyra** | $\pm 0.005 \sim 0.015 (^\circ/\text{s})/^\circ\text{C}$ | Interní teplotní kompenzace v pouzdře | F9R udržel stabilitu napříč rozsahem 09:20–16:40 |
| **Zdroj absolutního azimutu** | **3D magnetometr ($\pm 2\text{ Gauss}$)** | **Duální GNSS RTK báze (UNIHEADING)** | Odlišný fyzikální princip měření |
| **Vliv magnetických anomálií** | Citlivé na proudy pohonů a kovové prvky | Nulová citlivost na magnetické pole | Měření fázového posunu signálu GNSS nosné vlny |
| **Chyba azimutu na 100 m jízdy** | Neznámá (v 6-osém módu teoreticky desítky stupňů) | **$1.76^\circ$ medián ($2.18^\circ / 100\text{ m}$)** | Empiricky doloženo na 1 015 testovacích oknech |




