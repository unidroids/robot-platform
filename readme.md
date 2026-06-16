# Unidroids Robot Platform

Tento repozitář obsahuje zdrojové kódy a dokumentaci autonomního robota týmu **Unidroids**. Náš robot je navržen s důrazem na modularitu, jednoduchost údržby a přehlednou softwarovou architekturu. 

Repozitář vznikl původně pro soutěž Robotour 2025, ale nyní slouží jako obecná softwarová platforma pro různé úkoly a mise.

## 🚗 Hardwarová výbava

| Komponenta         | Popis                                                            |
| ------------------ | ---------------------------------------------------------------- |
| Výpočetní jednotka | NVIDIA Jetson Orin Nano 8GB, JetPack 6.2                         |
| Kamery             | 2× Waveshare IMX219, 200°, CSI (stereo pohled dolů)              |
| LiDAR              | Unitree L2 (Ethernet)                                            |
| GNSS               | C102-F9R GNSS + IMU (USB)                                        |
| Mobilní základna   | Hoverboard s upraveným firmware (řízení přes USB sériovou linku) |
| Ovládání           | Gamepad a Android telefon (Infinix Smart 8)                      |
| Úložiště           | SSD Lexar NM620 2TB (Jetson root + data)                         |

## 🧠 Softwarová architektura

* **Operační systém:** Ubuntu (JetPack 6)
* **Programovací jazyk:** Python 3.10.12
* **Vývojové prostředí:** VSCode Remote – SSH (headless přes USB-C gadget mód)
* **Struktura:** oddělené služby pro kamery, LiDAR, řízení, GNSS a centrální FastAPI server
* **Komunikace:** 
  * Ovládání služeb probíhá přes TCP.
  * Sdílení dat je řešeno přes sdílenou paměť (shared memory) a ZeroMQ (ZMQ).
  * HTTP se používá pouze pro externí API (FastAPI).
* **Záznamy:** logování obrazu, lidarových dat, pohybu a GNSS do /robot/data/logs

## 🔌 Služby a porty

| Služba           | Port | Popis                                         |
| ---------------- | ---- | --------------------------------------------- |
| **CAMERA**       | 9001 | Snímání obrazu, segmentace a logování         |
| **LIDAR**        | 9002 | Zpracování dat z L2 lidaru                    |
| **DRIVE**        | 9003 | Řízení podvozku (hoverboard)                  |
| **JOURNEY**      | 9004 | Hlavní orchestrátor workflow                  |
| **GAMEPAD**      | 9005 | Poskytuje informace z gamepadu                |
| **GNSS**         | 9006 | Zpracování polohy (F9R) a IMU                 |
| **PERFECT**      | 9007 | PointPerfect NTRIP klient (korekční data)     |
| **PILOT**        | 9008 | Autonomní řízení podle GPS                    |
| **FUSION**       | 9009 | Lokální fúze polohy a odometrie               |
| **HEADING**      | 9010 | Výpočet orientace (externí kompas/IMU)        |
| **VISION**       | 9011 | Zpracování obrazu z kamer a detekce           |
| **PILOT-VISION** | 9102 | Vizuální navigace pro autonomní řízení        |


## 🏆 Soutěže a mise (Challenges)

Architektura je rozdělena na obecné jádro a specifické mise. Dokumentaci a záznamy k jednotlivým soutěžím najdete v podsložce `challenges/`:

* [Robotour 2025](challenges/Robotour_2025/robotour_2025.md) – Původní soutěž, pro kterou byl robot postaven.
* [Tulák po Krasu 2026](challenges/Tulak_po_Krasu_2026/tulak_po_krasu.md) – Úpravy a odladění vizuální navigace po čáře.

## 📁 Struktura projektu

```
/robot/opt/projects/robotour
├── server/          # socket + HTTP servery
├── journey/         # plánování a workflow
├── camera/          # čtení, segmentace a logování kamer
├── lidar/           # TCP server pro lidar, transformace a analýza bodů
├── gnns/            # čtení pozice a rychlosti
└── install/         # systemd skripty, udev, konfigurace
```

Repozitář: [https://github.com/unidroids/robotour2025](https://github.com/unidroids/robotour2025)
