# gnss_serial.py
import os
import serial
import threading
import queue
import sys
from datetime import datetime
from typing import Optional, Tuple

# Konfigurace zařízení - konstanty na začátku souboru dle zadání
DEFAULT_DEVICE = '/dev/robot-gnss-imu'
DEFAULT_BAUDRATE = 921600
DEFAULT_LOG_DIR = '/data/robot/gnss-imu'

class GnssSerialIO:
    """
    Sériová komunikace pro UBX stream:
    - Otevírá sériový port a běží ve vyhrazeném čtecím vlákně
    - Provádí rychlý UBX framing (Sync 0xB5 0x62, kontrola Fletcher checksumu)
    - Ukládá validní UBX zprávy (msg_class, msg_id, payload) do FIFO fronty
    - Loguje surová data ze sériové linky do /data/robot/gnss-imu/<rrrr-mm-dd>/<hh-mm-ss>-rx/tx.bin
    """

    def __init__(
        self,
        device: str = DEFAULT_DEVICE,
        baudrate: int = DEFAULT_BAUDRATE,
        fifo_size: int = 150,
        log_dir: str = DEFAULT_LOG_DIR
    ):
        self.device = device
        self.baudrate = baudrate
        self.log_dir = log_dir
        self._fifo: "queue.Queue[Tuple[int, int, bytes]]" = queue.Queue(maxsize=fifo_size)

        self._ser: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

        self._rx_log_file = None
        self._tx_log_file = None
        self._rx_log_path = None
        self._tx_log_path = None

        self.stats_received = 0
        self.stats_corrupted = 0
        self.stats_checksum_err = 0

    @property
    def rx_log_path(self) -> Optional[str]:
        return self._rx_log_path

    @property
    def tx_log_path(self) -> Optional[str]:
        return self._tx_log_path

    def open(self, start_time: Optional[datetime] = None) -> None:
        self._stop_event.clear()
        while not self._fifo.empty():
            try:
                self._fifo.get_nowait()
            except queue.Empty:
                break

        # Otevření logovacích souborů pro surová data (rx / tx) s časem příkazu START
        now = start_time or datetime.now()
        try:
            date_dir = os.path.join(self.log_dir, now.strftime('%Y-%m-%d'))
            os.makedirs(date_dir, exist_ok=True)
            time_prefix = now.strftime('%H-%M-%S')
            self._rx_log_path = os.path.join(date_dir, f"{time_prefix}-rx.bin")
            self._tx_log_path = os.path.join(date_dir, f"{time_prefix}-tx.bin")
            self._rx_log_file = open(self._rx_log_path, "wb")
            self._tx_log_file = open(self._tx_log_path, "wb")
            print(f"[GnssSerialIO] Log files opened on START: {self._rx_log_path}")
        except Exception as e:
            print(f"[GnssSerialIO] Logging disabled: {e}", file=sys.stderr)
            self._rx_log_file = None
            self._tx_log_file = None
            self._rx_log_path = None
            self._tx_log_path = None

        self._ser = serial.Serial(self.device, self.baudrate, timeout=0.05)
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        # Uzavření logovacích souborů na příkaz STOP
        if self._rx_log_file:
            try:
                self._rx_log_file.flush()
                self._rx_log_file.close()
                if self._rx_log_path:
                    print(f"[GnssSerialIO] Log file closed on STOP: {self._rx_log_path}")
            except Exception:
                pass
            self._rx_log_file = None
            self._rx_log_path = None

        if self._tx_log_file:
            try:
                self._tx_log_file.flush()
                self._tx_log_file.close()
                if self._tx_log_path:
                    print(f"[GnssSerialIO] Log file closed on STOP: {self._tx_log_path}")
            except Exception:
                pass
            self._tx_log_file = None
            self._tx_log_path = None

    def send_raw(self, data: bytes) -> bool:
        """Odešle data na sériový port a zaloguje je do tx.bin."""
        if not self._ser or not self._ser.is_open:
            return False
        try:
            self._ser.write(data)
            if self._tx_log_file:
                try:
                    self._tx_log_file.write(data)
                    self._tx_log_file.flush()
                except Exception:
                    pass
            return True
        except Exception as e:
            print(f"[GnssSerialIO] Write error: {e}", file=sys.stderr)
            return False

    def get_message(self, timeout: Optional[float] = 0.1) -> Optional[Tuple[int, int, bytes]]:
        """Vyzvedne z fronty jednu validní zprávu (msg_class, msg_id, payload)."""
        try:
            return self._fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_error_counters(self) -> Tuple[int, int]:
        return (self.stats_corrupted, self.stats_checksum_err)

    @staticmethod
    def _calculate_ubx_checksum(data: bytes) -> Tuple[int, int]:
        ck_a = 0
        ck_b = 0
        for b in data:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b

    def _reader(self) -> None:
        buffer = bytearray()
        max_payload_len = 512

        while not self._stop_event.is_set():
            try:
                if not self._ser or not self._ser.is_open:
                    break

                in_waiting = self._ser.in_waiting
                chunk = self._ser.read(in_waiting if in_waiting > 0 else 1)
                if not chunk:
                    continue

                if self._rx_log_file:
                    try:
                        self._rx_log_file.write(chunk)
                        self._rx_log_file.flush()
                    except Exception:
                        pass

                buffer.extend(chunk)

                # Hledání a zpracování UBX rámců
                while len(buffer) >= 8:  # Min. délka UBX rámce s 0B payloadem je 8B: [B5 62 class id len_lo len_hi ckA ckB]
                    # Vyhledání sync znaků 0xB5 0x62
                    sync_idx = buffer.find(b'\xB5\x62')
                    if sync_idx == -1:
                        # Žádný sync nenalezen, ponecháme maximálně 1 byte (může to být 0xB5)
                        if len(buffer) > 1 and buffer[-1] == 0xB5:
                            del buffer[:-1]
                        else:
                            buffer.clear()
                        break

                    if sync_idx > 0:
                        # Odstraníme data před syncem
                        self.stats_corrupted += sync_idx
                        del buffer[:sync_idx]

                    if len(buffer) < 6:
                        # Čekáme na zbytek hlavičky
                        break

                    msg_class = buffer[2]
                    msg_id = buffer[3]
                    payload_len = buffer[4] | (buffer[5] << 8)

                    if payload_len > max_payload_len:
                        # Neplatná délka payloadu -> desync, posuneme o 2 bajty za falešný sync
                        self.stats_corrupted += 2
                        del buffer[:2]
                        continue

                    total_frame_len = 6 + payload_len + 2
                    if len(buffer) < total_frame_len:
                        # Čekáme na zbytek payloadu a checksumu
                        break

                    # Rámec je kompletní
                    frame = buffer[:total_frame_len]
                    del buffer[:total_frame_len]

                    header_and_payload = frame[2:6 + payload_len]
                    ck_a_calc, ck_b_calc = self._calculate_ubx_checksum(header_and_payload)
                    ck_a_rec = frame[-2]
                    ck_b_rec = frame[-1]

                    if ck_a_calc == ck_a_rec and ck_b_calc == ck_b_rec:
                        self.stats_received += 1
                        payload = bytes(frame[6:6 + payload_len])
                        try:
                            self._fifo.put_nowait((msg_class, msg_id, payload))
                        except queue.Full:
                            # Drop nejstarší při přetečení fronty
                            try:
                                self._fifo.get_nowait()
                                self._fifo.put_nowait((msg_class, msg_id, payload))
                            except Exception:
                                pass
                    else:
                        self.stats_checksum_err += 1

            except serial.SerialException as e:
                if not self._stop_event.is_set():
                    print(f"[GnssSerialIO] SerialException: {e}", file=sys.stderr)
                break
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[GnssSerialIO] Reader error: {e}", file=sys.stderr)
                break
