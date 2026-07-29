import serial
import threading
import queue
import sys
import zlib
from typing import Optional

class GpsSerialIO:
    def __init__(self, device: str = '/dev/robot-gps', baudrate: int = 115200, fifo_size: int = 100):
        self.device = device
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None
        self._fifo: "queue.Queue[str]" = queue.Queue(maxsize=fifo_size)
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._err_checksum = 0

    def open(self):
        self._ser = serial.Serial(self.device, self.baudrate, timeout=0.5)
        self._stop_event.clear()
        if not self._reader_thread.is_alive():
            self._reader_thread = threading.Thread(target=self._reader, daemon=True)
            self._reader_thread.start()

    def close(self):
        self._stop_event.set()
        try:
            if self._reader_thread.is_alive():
                self._reader_thread.join(timeout=0.5)
        except Exception:
            pass
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def get_sentence(self, timeout: Optional[float] = None) -> Optional[str]:
        try:
            return self._fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_error_counters(self) -> int:
        return self._err_checksum

    def _reader(self):
        while not self._stop_event.is_set():
            try:
                if self._ser:
                    line = self._ser.readline()
                    if not line:
                        continue
                    
                    try:
                        line_str = line.decode('ascii', errors='ignore').strip()
                    except:
                        continue
                        
                    if not line_str:
                        continue
                        
                    if line_str.startswith('$'):
                        # NMEA CRC check
                        if '*' in line_str:
                            content, checksum = line_str.rsplit('*', 1)
                            calc_cs = 0
                            for char in content[1:]:
                                calc_cs ^= ord(char)
                            if f"{calc_cs:02X}" == checksum.upper():
                                self._fifo.put_nowait(line_str)
                            else:
                                self._err_checksum += 1
                                print(f"[GpsSerialIO] Checksum error NMEA: {line_str}", file=sys.stderr)
                                
                    elif line_str.startswith('#'):
                        # Unicore CRC check
                        if '*' in line_str:
                            content, checksum = line_str.rsplit('*', 1)
                            # CRC32 of content starting after '#'
                            crc_data = content[1:].encode('ascii')
                            calc_crc = zlib.crc32(crc_data) & 0xFFFFFFFF
                            if f"{calc_crc:08x}".upper() == checksum.upper():
                                self._fifo.put_nowait(line_str)
                            else:
                                self._err_checksum += 1
                                print(f"[GpsSerialIO] Checksum error Unicore: {line_str}", file=sys.stderr)
            except queue.Full:
                print("[GpsSerialIO] FIFO full - dropping sentence", file=sys.stderr)
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[GpsSerialIO] Reader error: {e}", file=sys.stderr)
