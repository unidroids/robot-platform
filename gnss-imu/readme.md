# GNSS-IMU Služba (u-blox ZED-F9R / EVK-F9R) — Port 9016

Odlehčená služba pro příjem zpráv `UBX-ESF-RAW` (100Hz IMU surová data) ze zařízení u-blox ZED-F9R, integraci úhlového přírůstku kolem osy Z a publikaci do ZeroMQ s frekvencí 20 Hz.

## 🔌 Parametry a komunikace

* **Řídicí port:** `127.0.0.1:9016` (TCP)
* **Sériový port:** `/dev/robot-gnss-imu` (výchozí rychlost 921600 baud)
* **ZeroMQ IPC:** `ipc:///tmp/robot-gnss-imu`
* **ZeroMQ Topic:** `GYRO`
* **Výstupní frekvence:** 20 Hz (interval 50 ms)
* **Formát JSON zprávy:**
  ```json
  {
    "ts": 12345.6789,
    "delta_yaw": 0.1250,
    "wz": -2.4500
  }
  ```
  * `ts` – monotónní čas v sekundách
  * `delta_yaw` – integrovaný přírůstek úhlu ve stupních (°) od minulé 20Hz zprávy (kompasová orientace: otáčení doprava / CW je kladné `+`)
  * `wz` – okamžitá úhlová rychlost ve stupních za sekundu (°/s) z posledního vzorku

## 🧠 Architektura služby

* **`main.py`** – Vstupní bod, správa TCP serveru na portu 9016 a zachycení ukončovacích signálů (`SIGINT`, `SIGTERM`).
* **`client_handler.py`** – Obsluha TCP příkazů (`PING`, `START`, `STOP`, `STATUS`, `EXIT`, `SHUTDOWN`).
* **`service.py`** – Správce životního cyklu služby, 2sekundová synchronní kalibrace nulového biasu v klidu a 20Hz publisher smyčka.
* **`gnss_serial.py`** – Vlákno pro sériové čtení a odlehčený UBX framing s Fletcher-8 kontrolním součtem.
* **`handlers/esf_raw_handler.py`** – Dekódování 8B měřicích bloků zprávy `UBX-ESF-RAW` (škálování gyro $2^{-12}$, acc $2^{-10}$, temp $10^{-2}$).
* **`light_fusion.py`** – 100Hz integrační jádro s kompenzací biasu, ošetřením 32bitového přetečení `sTtag` a atomickým resetem akumulátoru při 20Hz odběru.

## 🕹️ Příkazy přes TCP (netcat)

```bash
# Test dostupnosti
echo "PING" | nc -q0 127.0.0.1 9016
# Odpověď: PONG GNSS-IMU

# Start služby (zahrnuje 2s synchronní kalibraci v klidu)
echo "START" | nc -q0 127.0.0.1 9016
# Odpověď: OK

# Stav služby
echo "STATUS" | nc -q0 127.0.0.1 9016
# Odpověď: RUNNING {"handled": 500, "published": 100, "bias_z": 0.182, "latest_wz": 0.05, "corrupted": 0, "chk_err": 0} {"ts": ...}

# Zastavení služby
echo "STOP" | nc -q0 127.0.0.1 9016
# Odpověď: OK
```

## 🛠️ Testování

Spuštění jednotkových testů:
```bash
python -m unittest test_light_fusion.py
python -m unittest test_tcp.py
```