import threading
import zmq
import json

from gnss_serial import GnssDualSerialIO
from handlers.bestnava_handler import BestnavaHandler
from handlers.gpgga_handler import GpggaHandler
from handlers.hwstatusa_handler import HwstatusaHandler
from handlers.uniheadinga_handler import UniHeadinAHandler
from rtk_poller import RtkPoller

class GnssDualService:
    def __init__(self):
        self.device = '/dev/robot-gnss-dual'
        self.baudrate = 921600
        
        self.running = False
        self._lock = threading.Lock()
        
        self.gnss_serial = None
        self.zmq_context = None
        self.zmq_pub = None
        self._dispatcher_thread = None
        self._stop_event = threading.Event()
        self.stats_handled = 0
        self.stats_unknown = 0
        
        self.gpgga_handler = None
        self.bestnava_handler = None
        self.hwstatusa_handler = None
        self.uniheadinga_handler = None
        self.rtk_poller = None
        
    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            
            self.zmq_context = zmq.Context.instance()
            self.zmq_pub = self.zmq_context.socket(zmq.PUB)
            self.zmq_pub.bind("ipc:///tmp/robot-gnss")
            
            self.gnss_serial = GnssDualSerialIO(self.device, self.baudrate)
            
            self.bestnava_handler = BestnavaHandler(self.zmq_pub)
            self.gpgga_handler = GpggaHandler(self.zmq_pub)
            self.hwstatusa_handler = HwstatusaHandler(self.zmq_pub)
            self.uniheadinga_handler = UniHeadinAHandler(self.zmq_pub)
                
            self.stats_handled = 0
            self.stats_unknown = 0
            
            self._stop_event.clear()
            self.gnss_serial.open()
            
            self.rtk_poller = RtkPoller(self.gnss_serial)
            self.rtk_poller.start()
            
            self._dispatcher_thread = threading.Thread(target=self._dispatcher, daemon=True)
            self._dispatcher_thread.start()
            
            self.running = True
            print("[SERVICE] GNSS-DUAL STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
                
            self._stop_event.set()
            
            if self.rtk_poller:
                self.rtk_poller.stop()
                self.rtk_poller = None
            
            if self.gnss_serial:
                self.gnss_serial.close()
                self.gnss_serial = None
                
            if self._dispatcher_thread and self._dispatcher_thread.is_alive():
                self._dispatcher_thread.join(timeout=2.0)
                self._dispatcher_thread = None
                
            if self.zmq_pub:
                self.zmq_pub.close()
                self.zmq_pub = None
                
            self.zmq_context = None
            
            self.gpgga_handler = None
            self.bestnava_handler = None
            self.hwstatusa_handler = None
            self.uniheadinga_handler = None
                
            self.running = False
            print("[SERVICE] GNSS-DUAL STOPPED")
            return "OK"

    def get_status(self):
        with self._lock:
            if not self.running:
                return "IDLE"
            errors = self.gnss_serial.get_error_counters() if self.gnss_serial else 0
            stats_json = json.dumps({"handled": self.stats_handled, "errors": errors, "unknown": self.stats_unknown})
            last_gpgga = self.gpgga_handler.get_last_json() if self.gpgga_handler else "{}"
            last_hwstatusa = self.hwstatusa_handler.get_last_json() if self.hwstatusa_handler else "{}"
            last_bestnava = self.bestnava_handler.get_last_json() if self.bestnava_handler else "{}"
            
            last_heading = "{}"
            if self.uniheadinga_handler:
                h = self.uniheadinga_handler.get_lastest()
                if h:
                    last_heading = h.decode('utf-8')
            
            return f"RUNNING {stats_json} {last_gpgga} {last_hwstatusa} {last_bestnava} {last_heading}"
            
    def _dispatcher(self):
        print("[SERVICE] Dispatcher thread started")
        while not self._stop_event.is_set():
            if not self.gnss_serial:
                break
            sentence = self.gnss_serial.get_sentence(timeout=0.1)
            if sentence:
                if sentence.startswith("$") and len(sentence.split(',', 1)[0]) == 6 and sentence.split(',', 1)[0].endswith("GGA"):
                    success = self.gpgga_handler.handle(sentence) if self.gpgga_handler else False
                    if success:
                        self.stats_handled += 1
                    else:
                        self.stats_unknown += 1
                elif sentence.startswith("#BESTNAVA"):
                    success = self.bestnava_handler.handle(sentence) if self.bestnava_handler else False
                    if success:
                        self.stats_handled += 1
                    else:
                        self.stats_unknown += 1
                elif sentence.startswith("#HWSTATUSA"):
                    success = self.hwstatusa_handler.handle(sentence) if self.hwstatusa_handler else False
                    if success:
                        self.stats_handled += 1
                    else:
                        self.stats_unknown += 1
                elif sentence.startswith("#UNIHEADINGA"):
                    if self.uniheadinga_handler:
                        try:
                            # UniHeadinAHandler expects bytes with \r\n
                            self.uniheadinga_handler.handle(sentence.encode('ascii') + b'\r\n')
                            self.stats_handled += 1
                        except Exception as e:
                            print(f"[SERVICE] Exception in UNIHEADINGA handler: {e}")
                            self.stats_unknown += 1
                else:
                    self.stats_unknown += 1
                    # To reduce log spam, we might only log unknown messages occasionally or hide them
                    # print(f"[SERVICE] Unknown or unhandled sentence: {sentence}")
        print("[SERVICE] Dispatcher thread ended")
