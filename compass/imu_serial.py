# imu_serial.py
import serial
import threading
import queue
import sys
from typing import Optional, Tuple

from parser import CompassParser, ParseResult

class ImuSerialIO:
    """
    IMU Serial I/O for 10-axis module.
    Handles reading bytes, parsing frames, and writing commands.
    """

    def __init__(self,
                 device: str = '/dev/robot-compass',
                 baudrate: int = 115200,
                 rx_fifo_size: int = 100,
                 write_fifo_size: int = 100):
        self.device = device
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None

        self._rx_fifo: "queue.Queue[bytes]" = queue.Queue(maxsize=rx_fifo_size)
        self._write_fifo: "queue.Queue[bytes]" = queue.Queue(maxsize=write_fifo_size)

        self._stop_event = threading.Event()
        self._write_event = threading.Event()
        self._reader_thread = None
        self._writer_thread = None

        self._parser = CompassParser()
        self._err_checksum = 0

    def open(self):
        self._ser = serial.Serial(self.device, self.baudrate, timeout=0.02)
        self._stop_event.clear()
        self._write_event.clear()
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._writer_thread = threading.Thread(target=self._writer, daemon=True)
        self._writer_thread.start()
        self._reader_thread.start()

    def close(self):
        self._stop_event.set()
        self._write_event.set()
        try:
            if self._reader_thread and self._reader_thread.is_alive():
                self._reader_thread.join(timeout=0.2)
        except Exception:
            pass
        try:
            if self._writer_thread and self._writer_thread.is_alive():
                self._writer_thread.join(timeout=0.2)
        except Exception:
            pass
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    def send_command(self, cmd_bytes: bytes) -> bool:
        try:
            self._write_fifo.put_nowait(cmd_bytes)
            self._write_event.set()
            return True
        except queue.Full:
            print("[ImuSerialIO] Write FIFO full - dropping command!", file=sys.stderr)
            return False

    def _writer(self):
        while not self._stop_event.is_set():
            self._write_event.wait(timeout=0.1)
            self._write_event.clear()
            while not self._write_fifo.empty():
                try:
                    data = self._write_fifo.get_nowait()
                except queue.Empty:
                    break
                try:
                    if self._ser and self._ser.writable():
                        self._ser.write(data)
                except Exception as e:
                    print(f"[ImuSerialIO] Writer error: {e}", file=sys.stderr)

    def get_message(self, timeout: Optional[float] = None) -> Optional[bytes]:
        try:
            return self._rx_fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_error_counters(self) -> int:
        return self._err_checksum

    def _reader(self):
        while not self._stop_event.is_set():
            try:
                b = self._ser.read(1)
                if not b:
                    continue
                typ, msg = self._parser.feed(b[0])

                if typ == ParseResult.PROCESSING:
                    continue

                if typ == ParseResult.MESSAGE:
                    try:
                        self._rx_fifo.put_nowait(msg)
                    except queue.Full:
                        print("[ImuSerialIO] RX FIFO full - dropping frame!", file=sys.stderr)

                elif typ == ParseResult.CHECKSUM_ERROR:
                    self._err_checksum += 1

            except Exception as e:
                print(f"[ImuSerialIO] Reader error: {e}", file=sys.stderr)
