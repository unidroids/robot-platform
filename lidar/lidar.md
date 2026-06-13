# Analýza služby LiDAR

Služba `lidar` v adresáři `/opt/projects/robotour/lidar` slouží jako komplexní prostředník (wrapper) a procesor dat pro 3D LiDAR Unitree (pomocí SDK v2). Zajišťuje připojení k senzoru, filtraci a transformaci bodů, výpočet vzdálenosti překážek a poskytování těchto dat dalším subsystémům robota prostřednictvím TCP a ZeroMQ.

## 1. Architektura a základní komponenty

Aplikace je napsána v C++17 a sestavuje se pomocí CMake. Skládá se z několika logických modulů:

- **[robot_lidar_tcp.cpp](file:///opt/projects/robotour/lidar/robot_lidar_tcp.cpp)**: Hlavní vstupní bod programu. Spouští TCP server naslouchající na `127.0.0.1:9002` a zpracovává příkazy od klientů.
- **[LidarController](file:///opt/projects/robotour/lidar/lidar_controller.hpp)**: Hlavní orchestrátor. Spravuje UDP připojení k samotnému LiDARu (přes `LidarReader`), sběr dat, aplikaci kalibrace a vyhodnocování mračen bodů.
- **[LidarPublisher](file:///opt/projects/robotour/lidar/lidar_publisher.hpp)**: Komponenta publikující data rychlostí 20 Hz na ZeroMQ socket `ipc:///tmp/robot-lidar`.
- **Zpracování dat**:
  - **[LidarPointBuffer](file:///opt/projects/robotour/lidar/point_buffer.hpp)**: Kruhový buffer (ring buffer) uchovávající posledních 65536 bodů (s kapacitou dumpování do PLY formátu).
  - **[LidarDistanceEvaluator](file:///opt/projects/robotour/lidar/distance_evaluator.hpp)**: Vyhodnocuje vzdálenost k nejbližší překážce v zadaném výškovém intervalu.
  - **[LidarReflectivityEvaluator](file:///opt/projects/robotour/lidar/reflectivity_image_evaluator.hpp)**: Ze zachycených bodů generuje "top-down" (BEV) 2D obrázek odrazivosti (PGM formát).

## 2. Rozhraní (API)

Aplikace nabízí dvě hlavní komunikační cesty pro ostatní moduly:

### A) TCP Server (Port 9002, lokálně)
Server běží v textovém režimu (příkaz zakončený `\n` nebo `\r\n`) a podporuje následující příkazy:
- `PING` $\rightarrow$ Vrací `PONG LIDAR`
- `START` $\rightarrow$ Spustí rotaci LiDARu a naslouchání dat. Vrací `OK STARTED` nebo `ERR START`.
- `STOP` $\rightarrow$ Zastaví rotaci LiDARu a sběr dat. Vrací `OK STOPPED`.
- `DISTANCE` $\rightarrow$ Vrací zformátovaný řetězec `1 <vzdálenost>`, nebo `-1 -1`, pokud buffer není naplněn.
- `CALIBRATE` $\rightarrow$ Spustí desetisekundový sběr pro kalibraci a uložení dat do `calibration.dat`.
- `REFLECTIVITY` $\rightarrow$ Vygeneruje obrázek s odrazivostí (intensity). Vrací `OK IMAGE <cesta>` nebo `ERR IMAGE`.
- `MODE <číslo>` $\rightarrow$ Přepne mód LiDARu (např. 2D vs 3D, Wide vs Standard FOV, zapnutí/vypnutí IMU).
- `EXIT` $\rightarrow$ Ukončí danou TCP relaci.
- `SHUTDOWN` $\rightarrow$ Úplně vypne a ukončí službu LiDAR.

### B) ZeroMQ Publisher
Publisher periodicky asynchronně čte stav z `LidarController` a na adrese `ipc:///tmp/robot-lidar` posílá JSON s aktuální vzdáleností ve formátu:
```json
distance/{"distance": <hodnota>}
```

## 3. Zpracování mračen bodů (Point Clouds)

1. **Čtení dat**: `LidarReader` naváže UDP spojení s LiDARem (`192.168.10.62:6101` $\rightarrow$ `192.168.10.2:6201`) a dekóduje pakety pomocí Unitree SDK.
2. **Transformace**: V `LidarPointBuffer` jsou body ihned převedeny z metrů do centimetrů a je na ně aplikována matice kalibrace (`T_CL`), případně defaultní matice náklonu.
3. **Maska robota (Ignore Box)**: Body uvnitř "ignore boxu" (tj. šasi robota) jsou ignorovány a zahozeny, aby je systém mylně nedetekoval jako překážku.
4. **Distance Evaluation**: Při dotazu na vzdálenost vyhledá algoritmus v kruhovém bufferu bod, který je v určeném Z rozsahu ($50-150$ cm výška) a je nejblíže počátku souřadnic, a vrátí jeho vzdálenost ve 2D ($XY$).

## 4. Logování a Diagnostika

Služba obsahuje velmi silnou vrstvu logování, což napovídá, že je používána i pro offline zpracování (např. trénink neuronových sítí, mapování):
- **Raw Logger**: Zapisuje nezpracované UDP pakety (Point, IMU, Version).
- **PLY Logger**: V `LidarPointBuffer` dojde po zaplnění 65536 bodů k exportu (tzv. "dump") obsahu do standardního formátu mračna bodů `.ply`. Soubory se obvykle ukládají do podadresářů v `/data/robot/lidar/`.

## Zhodnocení a doporučení
Aplikace je postavena na moderních C++ standardech. Přístup do kritických sekcí (buffers, state) je pečlivě zamykán mutexy. Použití IPC soketu pro 20Hz update vzdálenosti a odděleného TCP pro kontrolní příkazy je z hlediska designu a modularity velmi vhodné pro robotické prostředí.

> [!NOTE]
> V souboru `robot_lidar_tcp.cpp` je příkaz `CORIDORS` připraven, ale aktuálně neobsahuje žádnou implementaci (odkaz na `corridor_evaluator.hpp` není z příkazu explicitně volán). Pokud je detekce koridorů vyžadována, je potřeba doprogramovat její navázání.
