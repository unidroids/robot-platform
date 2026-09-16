# gnss_serial.py
import serial
import threading
import queue
import sys
from typing import Optional, Tuple

# Konfigurace zařízení - konstanty na začátku souboru dle zadání
DEFAULT_DEVICE = '/dev/robot-gnss-imu'
DEFAULT_BAUDRATE = 921600

class GnssSerialIO:
    """
    Sériová komunikace pro UBX stream:
    - Otevírá sériový port a běží ve vyhrazeném čtecím vlákně
    - Provádí rychlý UBX framing (Sync 0xB5 0x62, kontrola Fletcher checksumu)
    - Ukládá validní UBX zprávy (msg_class, msg_id, payload) do FIFO fronty
    """

    def __init__(self, device: str = DEFAULT_DEVICE, baudrate: int = DEFAULT_BAUDRATE, fifo_size: int = 150):
        self.device = device
        self.baudrate = baudrate
        self._fifo: "queue.Queue[Tuple[int, int, bytes]]" = queue.Queue(maxsize=fifo_size)

        self._ser: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

        self.stats_received = 0
        self.stats_corrupted = 0
        self.stats_checksum_err = 0

    def open(self) -> None:
        self._stop_event.clear()
        while not self._fifo.empty():
            try:
                self._fifo.get_nowait()
            except queue.Empty:
                break

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
