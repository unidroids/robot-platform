# Služba `pilot_waypoints` úspěšně naimplementována!

Kompletní nová navigační služba je připravena ve složce `/opt/projects/robotour/pilot_waypoints/`.

## 1. Zkopírované a upravené závislosti
Do složky byly nahrány zkopírované moduly `geo_utils.py`, `near_waypoint.py` a `drive_client.py`, aby služba nepodléhala změnám původního (brzy archivovaného) pilota.

## 2. Vyhledávání cesty - Dvoufázový přístup
Implementoval jsem inteligentní **Path Tracker**:
* **Fáze 1 (Ověření)**: Zkontroluje se, zda má robot smysluplný průsečík s posledně známým segmentem nebo plynule přešel na další. Pokud ne, spustí Fázi 2.
* **Fáze 2 (Vyhledávání)**: Iteruje celou trasu, vypočítá vzdálenost k segmentům a vybere ten nejbližší. 
* **Umělá úsečka**: Pokud nenajde přímý průsečík, spojí aktuální polohu robota a nejbližší waypoint na mapě do umělého segmentu, po kterém se robot automaticky navede zpět na trasu. Po dojetí na vzdálenost <0.5m se znovu zapne Fáze 2 a robot začne sledovat původní trasu.

## 3. Bezpečnostní kontrolní mechanismy (OOW) a přesnost GPS
Služba kombinuje tři paralelní stavy blokující/pozastavující robota:
* **TCP Polling (OOW)**: Každou sekundu se ptá `OOW` portu `9013`. Pokud ztratí spojení, přejde do `PAUSE` s logem ztráty OOW.
* **ZMQ Eventy (OOW)**: Zachytává události `ON`, `OFF`, `STOP`, `PAUSE`, `RESUME` a reaguje okamžitě přepsáním stavů.
* **GPS Přesnost**: Pokud přesáhne odhadovaná chyba (`hAcc`) `700 mm`, robot zabrzdí (rychlost 0, `send_drive` s 0 na kopci). Po zpřesnění pod `500 mm` se znovu rozjede.

## 4. Omezení zrychlení a Logování dat
Pro plynulou a bezpečnou jízdu byla přidána korekce na základě fyzikální kinematiky podvozku:
* **Odstředivé zrychlení ($a_c = v \cdot \omega$)**: Před aplikací jakýchkoli limitů se spočítá reálné boční zrychlení na základě rozchodu kol (530 mm). Pokud by robot chtěl jet do zatáčky tak, že překročí `0.5 m/s²`, plynule zmenší povel (poměrově zbrzdí), aby zachoval poloměr zatáčení, ale nepřekročil maximální G-force.
* **Lineární zrychlení a brzdění**: Následně se hlídá maximální skok dopředné (resp. brzdné) rychlosti za 1 periodu (10Hz).
* **Úhlové zrychlení**: Analogicky se omezí i to, jak rychle se smí robot roztočit kolem své osy.
* Tato matematika zajišťuje plynulý rozjezd po `RESUME`, bezpečný průjezd zatáčkou a plynulé zastavení po `PAUSE`.
* Na konci každé iterace se výsledky (přijatá GPS, steering komanda před a po omezení) zaznamenávají přes `data_logger.py` do složky `/data/robot/pilot_waypoints/`.

## 5. Rychlé ovládání přes TCP server
Služba poslouchá na TCP portu `9101` a odpovídá na všechny požadované příkazy (včetně upovídaného `STATUS`, který vrací index waypointu, pozici a navíc zdroj `PAUSE`/`RUNNING` – tj. vrací např. `PAUSED OOW_TCP Lost OOW connection`).

### Jak spustit
Spuštění služby:
```bash
python3 /opt/projects/robotour/pilot_waypoints/main.py
```
A do jiného terminálu můžete testovat povely (např. pomocí netcat):
```bash
echo "START 100 150" | nc 127.0.0.1 9101
echo "STATUS" | nc 127.0.0.1 9101
```
