# OOW Client (Android)

Tento adresář obsahuje zdrojové kódy pro klientskou Android aplikaci. 

Z důvodu čistoty repozitáře a úspory místa obsahuje tento projekt **pouze zdrojové kódy a nezbytnou konfiguraci** (tzv. čistý snapshot). Veškeré vygenerované soubory a složky (jako jsou `build/`, `.gradle/`, `.idea/` nebo `local.properties`) jsou záměrně smazány a izolovány od Gitu.

## Jak aplikaci otevřít a pokračovat ve vývoji

Aby aplikace fungovala a šla zkompilovat, **neotevírejte zdrojové soubory jen jako text**. Postupujte následovně:

1. Zkopírujte/naklonujte si složku `oow_client` na lokální počítač s Windows/Mac/Linux.
2. Otevřete **Android Studio**.
3. Na úvodní obrazovce zvolte **Open** (nebo v horním menu `File -> Open...`).
4. Najděte a **vyberte složku `oow_client`** (tu, ve které se nachází tento soubor README) a potvrďte.
5. **Vyčkejte na dokončení "Gradle Sync".** 

Android Studio automaticky detekuje soubory `build.gradle`, samo si stáhne všechny potřebné verze knihoven a na pozadí si **znovu vygeneruje** všechny chybějící složky pro kompilaci. Jakmile synchronizace doběhne, projekt je plně funkční a připravený k vývoji nebo nasazení do zařízení.

## Architektura
* `app/src/main/java/...` – Zdrojové kódy v Kotlinu (`MainActivity.kt`).
* `app/src/main/res/` – Definice uživatelského rozhraní (`activity_main.xml`) a textů.
* `gradle/` a `build.gradle` – Konfigurace sestavení aplikace.