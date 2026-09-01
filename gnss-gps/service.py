import threading
import zmq
import json
from gps_serial import GpsSerialIO
from handlers.bestnava_handler import BestnavaHandler
from handlers.gpgga_handler import GpggaHandler
from handlers.hwstatusa_handler import HwstatusaHandler
from rtk_poller import RtkPoller

class GpsService:
    def __init__(self):
        self.device = '/dev/robot-gnss-gps'
        self.baudrate = 921600
        
        self.running = False
        self._lock = threading.Lock()
        
        self.gps_serial = None
        self.zmq_context = None
        self.zmq_pub = None
        self._dispatcher_thread = None
        self._stop_event = threading.Event()
        self.stats_handled = 0
        self.stats_unknown = 0
        
        self.gpgga_handler = None
        self.bestnava_handler = None
        self.hwstatusa_handler = None
        self.rtk_poller = None
        
    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            
            # Kompletní reinicializace prostředků při každém startu
            self.zmq_context = zmq.Context.instance()
            self.zmq_pub = self.zmq_context.socket(zmq.PUB)
            self.zmq_pub.bind("ipc:///tmp/robot-gnss-gps")
            
            self.gps_serial = GpsSerialIO(self.device, self.baudrate)
            
            self.bestnava_handler = BestnavaHandler(self.zmq_pub)
            self.gpgga_handler = GpggaHandler(self.zmq_pub)
            self.hwstatusa_handler = HwstatusaHandler(self.zmq_pub)
                
            self.stats_handled = 0
            self.stats_unknown = 0
            
            self._stop_event.clear()
            self.gps_serial.open()
            
            self.rtk_poller = RtkPoller(self.gps_serial)
            self.rtk_poller.start()
            
            self._dispatcher_thread = threading.Thread(target=self._dispatcher, daemon=True)
            self._dispatcher_thread.start()
            
            self.running = True
            print("[SERVICE] GPS STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
                
            self._stop_event.set()
            
            if self.rtk_poller:
                self.rtk_poller.stop()
                self.rtk_poller = None
            
            # Zrušíme serial port první, to pomůže uvolnit read vlákno
            if self.gps_serial:
                self.gps_serial.close()
                self.gps_serial = None
                
            # Počkáme na uvolnění dispečerského vlákna s dostatečným timeoutem
            if self._dispatcher_thread and self._dispatcher_thread.is_alive():
                self._dispatcher_thread.join(timeout=2.0)
                self._dispatcher_thread = None
                
            if self.zmq_pub:
                self.zmq_pub.close()
                self.zmq_pub = None
                
            self.zmq_context = None
            
            # Bezpečné zapomenutí handlerů
            self.gpgga_handler = None
            self.bestnava_handler = None
            self.hwstatusa_handler = None
                
            self.running = False
            print("[SERVICE] GPS STOPPED")
            return "OK"

    def get_status(self):
        with self._lock:
            if not self.running:
                return "IDLE"
            errors = self.gps_serial.get_error_counters() if self.gps_serial else 0
            stats_json = json.dumps({"handled": self.stats_handled, "errors": errors, "unknown": self.stats_unknown})
            last_gpgga = self.gpgga_handler.get_last_json() if self.gpgga_handler else "{}"
            last_hwstatusa = self.hwstatusa_handler.get_last_json() if self.hwstatusa_handler else "{}"
            last_bestnava = self.bestnava_handler.get_last_json() if self.bestnava_handler else "{}"
            return f"RUNNING {stats_json} {last_gpgga} {last_hwstatusa} {last_bestnava}"
            
    def _dispatcher(self):
        print("[SERVICE] Dispatcher thread started")
        while not self._stop_event.is_set():
            if not self.gps_serial:
                break
            sentence = self.gps_serial.get_sentence(timeout=0.1)
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
                else:
                    self.stats_unknown += 1
                    print(f"[SERVICE] Unknown or unhandled sentence: {sentence}")
        print("[SERVICE] Dispatcher thread ended")
