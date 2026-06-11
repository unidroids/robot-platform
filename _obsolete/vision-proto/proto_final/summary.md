# Nativní Vision Mikroslužba (ZMQ + SHM)

Tato fáze projektu úspěšně dokončila refaktorování zpracování obrazu na zařízeních NVIDIA Jetson do podoby dvou vysoce efektivních, bezvláknových a robustních skriptů: `capture.py` a `complete.py`.

> [!NOTE]
> Veškeré měření času je nyní napříč celou vizuální službou sjednoceno na absolutním **hardwarovém monotónním čase** čerpaném přímo z GStreamer bufferů (`buf.pts` + Base Time).

## Architektura a Výkon

### 1. Hardwarová Efektivita a Vytížení
- Ovladače kamer pracují na úrovni C přes GStreamer. Python funguje pouze jako úzká roura pro překlopení dat z NVMM (Nvidia paměti) do SHM a inicializaci enginu.
- Díky tomu CPU (`tegrastats`) ukazuje zatížení běžně v rozmezí 10-30 % s velkou rezervou, zatímco TensorRT inferenční smyčka dokáže vyhodnotit model přibližně za 18 milisekund.
- Celková end-to-end latence, tj. doba od sejmutí reálného snímku v pipeline, přes přesun do kontejneru a výpočet AI až na 3D body v Metrech, se drží stabilně na hranici **~45 ms (při 20 FPS)**.
- Spotřeba Jetson Orin Nano dosahuje velmi úsporných 8W.

### 2. Snímání (`capture.py`)
- Původní `time.time()` epochový čas byl kompletně nahrazen exaktním GStreamer `pts` časovačem (`(Base Time + pts) / 1e9`), synchronizovaným vůči nativnímu linuxovému `time.monotonic()`.
- Zpoždění přesunu GStreamer bufferu a 6 MB kopírování snímku do SHM je profilováno a logováno u každého dvacátého framu.
- Skript řídí mazání pamětí v `/dev/shm` striktně při inicializaci (`unlink()` a `create=True`).

### 3. Zpracování a AI inference (`complete.py`)
- Zcela odstraněno používání modulů `threading` a dedikovaných pracovních vláken. Analýza pro levou i pravou kameru probíhá plně jednovláknově a střídavě z jednoho ZMQ vstupu (`ipc:///tmp/robot-camera`).
- Model je inicializován přes YOLO TRT (`task='pose'`) a provádí se přes GPU transformaci na BEV (Birds-Eye-View) z předem vypočítaných `npz` mřížek.
- Pro odesílání metadat robotovi slouží nově otevřený ZMQ Publisher `ipc:///tmp/robot-vision`.

### 4. Automatický Self-Healing (Reconnect)
- Spotřebitelský skript `complete.py` implementuje "lock-free" ověřování integrity snímků (`header` s číslem snímku a `zmq_seq`).
- V případě, že se skript `capture.py` restartuje (nebo padne), dojde k přemazání paměti `/dev/shm`. Skript `complete.py` detekuje kolizi, provede automatický zahazovací drop a plynulý **SHM Reconnect** (uzavře starý inode a připojí se k nové paměti), čímž zamezí "ztrátě vlákna" a není nutné jej ručně restartovat.

## Geometrický model (Převod do Metrů)

Zpracování nyní obsahuje fyzikální převod pixelových keypointů z YOLO modelu do lokálního referenčního rámce robota.

> [!IMPORTANT]
> Souřadný systém má střed těsně pod hranou obrazu. 
> Střed robota je na `(X=0, Y=0)` a odpovídá spodnímu středu obrazu.

Převodní konstanta odráží proporci, ve které `3 metry = 480 pixelům`, tedy `1/160 m/px`.
- **Osa X (Hloubka):** Směřuje od robota *dopředu*. Spodní hrana obrazu (`y=480`) má hodnotu `X = 0 m`. Horní hrana (`y=0`) odpovídá `X = 3 m`.
- **Osa Y (Šířka):** Směřuje od robota *do stran*. Levá hrana obrazu (`x=0`) je `Y = +2 m`. Pravá hrana obrazu (`x=640`) dává `Y = -2 m`.
