# Pilot Waypoints Služba

Služba `pilot_waypoints` zajišťuje autonomní navigaci robota (Tříkolky) podle předem definovaných navigačních bodů (waypoints). Zpracovává aktuální polohu, orientaci, data z LiDARu pro antikolizi a bezpečně generuje řídicí povely pro motory.

## Architektura a Interakce

Tato mikroservisa je navržena k asynchronnímu běhu a interaguje s mnoha dalšími systémy v robotovi přes ZeroMQ (ZMQ) a TCP sokety.

### 1. ZMQ Odběry (Subscriptions)
Služba aktivně naslouchá na následujících ZMQ IPC soketech:

*   **`ipc:///tmp/robot-fusion`** (Topic: `SOLUTION`)
    *   **Význam:** Přijímá fúzovaná data o poloze (GPS souřadnice, aktuální azimut/heading). Tyto údaje používá `path_tracker` pro výpočet odchylky od trasy a vzdálenosti k dalšímu waypointu.
*   **`ipc:///tmp/robot-lidar`** (Topic: `DISTANCE`)
    *   **Význam:** Čte aktuální vzdálenost od překážek naměřenou LiDARem. V případě, že je překážka blíže než 70 cm, služba automaticky zastaví robota, bez ohledu na navigační cíl.
*   **`ipc:///tmp/robot-oow`** (Topics: `STATUS`, `CMD`)
    *   **Význam:** Bezpečnostní dohled (Officer of the Watch). Poslouchá změny stavů (`ON`/`OFF`) a zachytává krizové příkazy (např. vynucené přerušení `PAUSE` při ztrátě signálu z mobilní aplikace).

### 2. Odchozí TCP komunikace (Klienti)
*   **Drive Služba (`127.0.0.1:9003`)**
    *   Pomocí modulu `drive_client.py` se služba připojuje k ovládání motorů. Posílá příkazy jako `DRIVE pwm left right`, `STOP`, `BREAK`, jimiž fyzicky pohybuje robotem na základě vypočtené dráhy.
*   **OOW Poller (`127.0.0.1:9013` / TCP OOW)** *(Pozn.: port 9013 může být zastaralý vzhledem k přesunu OOW na 9030)*
    *   Záložní kontrolní mechanismus. Každou vteřinu posílá TCP dotaz `OOW\n` a očekává odpověď `ON` nebo `OFF`. Pokud OOW neodpoví nebo odpoví `OFF`, pilot se automaticky přepne do stavu `PAUSED`.

### 3. Příchozí TCP komunikace (Ovládání služby)
Služba samotná funguje jako TCP server naslouchající na portu **`9101`**. Z nadřazených aplikací (např. z Terminálu nebo mobilní aplikace přes HMI bridge) přijímá textové příkazy pro řízení mise:

*   `START [speed] [pwm]` - Spustí autonomní misi. Volitelně lze předat maximální rychlost a maximální PWM.
*   `STOP` - Okamžitě ukončí řízení a přepne stav na `STOPPED`.
*   `PAUSE` - Dočasně pozastaví autonomní jízdu (motory stojí, ale mise je aktivní).
*   `RESUME` - Obnoví jízdu po předchozím pozastavení.
*   `STATUS` - Vrací komplexní stav pilota (např. aktuální waypoint, vzdálenost, stav OOW, běh mise).
*   `PING` - Vrací `PONG PILOT_WAYPOINTS` pro kontrolu naživu.
*   `SHUTDOWN` / `EXIT` - Kompletně ukončí proces služby.

## Vnitřní stavový automat (State Machine)
Služba operuje ve čtyřech hlavních stavech:
1.  **STOPPED:** Služba běží, ale aktivně neřídí (motory stojí). Čeká na `START`.
2.  **RUNNING:** Normální autonomní jízda podle waypointů.
3.  **PAUSED:** Jízda dočasně přerušena (zásahem uživatele, detekcí překážky z LiDARu, ztrátou OOW signálu, nebo chybějící GPS/heading fixací).
4.  **FINISHED:** Konec trasy byl dosažen. Robot plynule zpomalil a zastavil.

## Zpracování trasy
*   Modul `path_tracker.py` se stará o načtení waypointů, sledování průjezdu jednotlivými body a výpočet kolmé odchylky od ideální linie (crosstrack error).
*   Algoritmus dynamicky zpomaluje robota do zatáček (výpočet založený na relativním azimutu dalšího bodu) a plynule upravuje otáčky levého a pravého kola pro přesné sledování trasy.
