
### Část 1: Technické požadavky (Architektura a Cílová platforma)

**Role:** Jsi expertní Android vývojář. Tvým úkolem je vytvořit základ a architekturu pro novou aplikaci.

**Cílové zařízení a platforma:**

* **Zařízení:** Infinix X6525.
* **OS:** Android 13 (API Level 33). Je nutné dodržovat restrikce této verze (zejména sítě a oprávnění polohy).
* **Závislosti:** Aplikace poběží offline na zařízení bez přihlášeného Google účtu.

**Technologický stack:**

* **Jazyk:** Kotlin.
* **UI:** Jetpack Compose (Single Activity Architecture).
* **Architektura:** MVVM (Model-View-ViewModel). Odděl striktně UI vrstvu (Compose) od byznys logiky (ViewModel) a datové vrstvy (TCP repozitář).
* **Asynchronní operace:** Kotlin Coroutines. Veškeré IO operace a síťová komunikace musí bezpodmínečně běžet na `Dispatchers.IO`, aby nedošlo k výjimce `NetworkOnMainThreadException`.

**Specifická nastavení (Manifest & UI):**

* **Orientace:** Vynucený Landscape mód (`android:screenOrientation="landscape"`).
* **Immersive Mode:** Skryj systémové lišty (Status bar a Navigation bar), aplikace musí běžet ve Fullscreenu. Displej se může standardně uspávat (nevyžadujeme Wakelock).
* **Oprávnění:** `ACCESS_FINE_LOCATION`, `ACCESS_COARSE_LOCATION` (řešeno přes Compose permissions) a `INTERNET`.
* **QR Skener:** Integruj `GmsBarcodeScanning` z Google Play Services (omezeno na `FORMAT_QR_CODE`). Zkus přidat do manifestu meta-data pro offline instalaci modulu (`com.google.mlkit.vision.DEPENDENCIES`). Nepoužívej CameraX.

---

### Část 2: Komunikační protokol a Síťová vrstva

**Role:** Implementuj datovou/síťovou vrstvu pro Android aplikaci založenou na čistých TCP socketech.

**Základní parametry spojení:**

* **Cíl:** `127.0.0.1`, port `9000` (komunikace probíhá přes ADB reverse tunel do robota).
* **Timeouty:** Spojení (Connection Timeout) = 2000 ms, Čtení (Read Timeout) = 2000 ms.
* **Životní cyklus socketu:** Pro každý odeslaný příkaz se otevře nový Socket, odešlou se data, počká se na přečtení odpovědi a Socket se bezpečně uzavře (`use` blok v Kotlinu).
* **Formát zpráv:** Každá odeslaná i přijatá zpráva je textový řetězec v kódování UTF-8, striktně zakončený znakem nového řádku `\n`.

**Implementuj TCP Repozitář (např. `RobotClient`), který podporuje tyto strukturované příkazy:**
Každá metoda repozitáře musí vracet `Result<String>` s ošetřením chyb (Timeout, Connection Refused).

1. **Základní příkazy robota:**
* `PING` -> Očekávaná odpověď: `PONG HMI\n` (Slouží k ověření funkčnosti tunelu).


2. **Aplikační příkazy (Prefix `MISSION <název_mise>`):**
Pro aktuální aplikaci implementuj metody pro misi `ROBOTOUR`:
* **Start:** `MISSION ROBOTOUR START geo:<lat>,<lon>\n` -> Očekává potvrzení (např. `OK`).
* **Stop:** `MISSION ROBOTOUR STOP\n` -> Očekává potvrzení.
* **Status:** `MISSION ROBOTOUR STATUS\n` -> Očekává textový stav (např. `Probíhá...` nebo `MISSION COMPLETE`).
* *(Rezerva do budoucna: `MISSION ROBOTOUR PAUSE`, `MISSION ROBOTOUR RESUME`)*.



---

### Část 3: Aplikační rozhraní (UI a State Machine)

**Role:** Implementuj UI vrstvu v Jetpack Compose a propoj ji přes ViewModel s TCP vrstvou.

**Stavy aplikace a Flow:**

1. **Init State (Ověření spojení):**
* Při spuštění aplikace pošli na pozadí příkaz `PING`.
* *Pokud timeout/chyba:* Zobraz chybovou obrazovku "Robot nedostupný (Zkontrolujte kabel)" a velké tlačítko "Zkusit znovu" (znovu pošle PING).
* *Pokud přijde `PONG HMI`:* Přejdi do State 1.


2. **State 1: Start Screen (Idle)**
* Zobrazí velké tlačítko "Naskenovat QR Code".
* Při stisku vyvolá systémový GMS QR Scanner. Pokud je sken zrušen uživatelem, zůstaň v tomto stavu.


3. **State 2: Validace Cíle**
* **Formát:** Ověř, že naskenovaný text začíná `geo:`. Pokud ne, vypiš chybu a ukaž tlačítko "Skenovat znovu" (návrat).
* **Vzdálenost:** Získej aktuální GPS polohu telefonu (`FusedLocationProviderClient`) a spočítej vzdálenost k cíli v metrech.
* **Zobrazení:** Vypiš cílové souřadnice a vypočtenou vzdálenost. (Vzdálenost do 1000m obarvi zeleně, nad 1000m červeně - ale neblokuj spuštění).
* **Akce:** Zobraz velká tlačítka "Vyrazit do destinace" (odesílá `MISSION ROBOTOUR START...`) a "Skenovat znovu".
* *Pokud server neodpoví na START OK, zobraz chybu a nech uživatele ve State 2.*


4. **State 3: Mise probíhá (Polling)**
* Aplikace přešla do tohoto stavu po úspěšném odeslání START.
* Zobraz: Text cíle a vyhrazené textové pole pro "Status z robota".
* **Polling:** Spusť Coroutine Timer, který každých **5 vteřin** odešle `MISSION ROBOTOUR STATUS`. Výsledek zobraz v textovém poli (při timeoutu pollingu zobraz "Čekám na data...").
* Zobraz červené tlačítko "Přerušit" (odesílá `MISSION ROBOTOUR STOP` a vrací UI do State 1).
* **Automatický konec:** Pokud v odpovědi na STATUS přijde text začínající `MISSION COMPLETE`, zastav polling, zobraz finální hlášku a nabídni tlačítko "Nová mise" (návrat do State 1).


