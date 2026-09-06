import threading
import zmq
import json

from gnss_serial import GnssDualSerialIO
from handlers.bestnava_handler import BestnavaHandler
from handlers.bestnavha_handler import BestnavhaHandler
from handlers.gpgga_handler import GpggaHandler
from handlers.hwstatusa_handler import HwstatusaHandler
from handlers.msposa_handler import MsposaHandler
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
        
        self.handlers = []
        self.gpgga_handler = None
        self.bestnava_handler = None
        self.bestnavha_handler = None
        self.hwstatusa_handler = None
        self.uniheadinga_handler = None
        self.msposa_handler = None
        self.rtk_poller = None
        
    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            
            self.zmq_context = zmq.Context.instance()
            self.zmq_pub = self.zmq_context.socket(zmq.PUB)
            self.zmq_pub.bind("ipc:///tmp/robot-gnss-dual")
            
            self.gnss_serial = GnssDualSerialIO(self.device, self.baudrate)
            
            self.bestnava_handler = BestnavaHandler(self.zmq_pub)
            self.bestnavha_handler = BestnavhaHandler(self.zmq_pub)
            self.gpgga_handler = GpggaHandler(self.zmq_pub)
            self.hwstatusa_handler = HwstatusaHandler(self.zmq_pub)
            self.uniheadinga_handler = UniHeadinAHandler(self.zmq_pub)
            self.msposa_handler = MsposaHandler(self.zmq_pub)
            
            self.handlers = []
            self.register_handler(self._is_gga, self.gpgga_handler)
            self.register_handler("#BESTNAVA", self.bestnava_handler)
            self.register_handler("#BESTNAVHA", self.bestnavha_handler)
            self.register_handler("#HWSTATUSA", self.hwstatusa_handler)
            self.register_handler("#UNIHEADINGA", self.uniheadinga_handler)
            self.register_handler("#MSPOSA", self.msposa_handler)
                
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
            
            self.handlers = []
            self.gpgga_handler = None
            self.bestnava_handler = None
            self.bestnavha_handler = None
            self.hwstatusa_handler = None
            self.uniheadinga_handler = None
            self.msposa_handler = None
                
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
            last_bestnavha = self.bestnavha_handler.get_last_json() if self.bestnavha_handler else "{}"
            last_heading = self.uniheadinga_handler.get_last_json() if self.uniheadinga_handler else "{}"
            last_msposa = self.msposa_handler.get_last_json() if self.msposa_handler else "{}"
            
            return f"RUNNING {stats_json} {last_gpgga} {last_hwstatusa} {last_bestnava} {last_bestnavha} {last_heading} {last_msposa}"
            
    @staticmethod
    def _is_gga(sentence: str) -> bool:
        return sentence.startswith("$") and len(sentence.split(',', 1)[0]) == 6 and sentence.split(',', 1)[0].endswith("GGA")

    def register_handler(self, matcher, handler):
        self.handlers.append((matcher, handler))

    def _dispatcher(self):
        print("[SERVICE] Dispatcher thread started")
        while not self._stop_event.is_set():
            if not self.gnss_serial:
                break
            sentence = self.gnss_serial.get_sentence(timeout=0.1)
            if sentence:
                handled = False
                for matcher, handler in self.handlers:
                    matched = matcher(sentence) if callable(matcher) else sentence.startswith(matcher)
                    if matched:
                        if handler and handler.handle(sentence):
                            self.stats_handled += 1
                        else:
                            self.stats_unknown += 1
                        handled = True
                        break
                if not handled:
                    self.stats_unknown += 1
                    # To reduce log spam, we might only log unknown messages occasionally or hide them
                    # print(f"[SERVICE] Unknown or unhandled sentence: {sentence}")
        print("[SERVICE] Dispatcher thread ended")
