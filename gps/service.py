import threading
import zmq
from gps_serial import GpsSerialIO
from handlers.bestnava_handler import BestnavaHandler
from handlers.gpgga_handler import GpggaHandler
from handlers.hwstatusa_handler import HwstatusaHandler

class GpsService:
    def __init__(self):
        self.device = '/dev/robot-gps'
        self.baudrate = 115200
        
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        
        self.gps_serial = None
        self.zmq_context = None
        self.zmq_pub = None
        self._dispatcher_thread = None
        self._stop_event = threading.Event()
        
        self.last_gpgga_json = "{}"
        
    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            
            if not self._initialized:
                self.zmq_context = zmq.Context.instance()
                self.zmq_pub = self.zmq_context.socket(zmq.PUB)
                self.zmq_pub.bind("ipc:///tmp/robot-gps")
                
                self.gps_serial = GpsSerialIO(self.device, self.baudrate)
                
                self.bestnava_handler = BestnavaHandler(self.zmq_pub)
                self.gpgga_handler = GpggaHandler(self.zmq_pub, self)
                self.hwstatusa_handler = HwstatusaHandler(self.zmq_pub)
                
                self._initialized = True
                
            self._stop_event.clear()
            self.gps_serial.open()
            
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
            if self.gps_serial:
                self.gps_serial.close()
                
            if self._dispatcher_thread and self._dispatcher_thread.is_alive():
                self._dispatcher_thread.join(timeout=0.5)
                
            self.running = False
            print("[SERVICE] GPS STOPPED")
            return "OK"

    def get_status(self):
        with self._lock:
            if not self.running:
                return "IDLE"
            return f"RUNNING {self.last_gpgga_json}"
            
    def update_last_gpgga(self, json_data: str):
        self.last_gpgga_json = json_data
            
    def _dispatcher(self):
        print("[SERVICE] Dispatcher thread started")
        while not self._stop_event.is_set():
            sentence = self.gps_serial.get_sentence(timeout=0.1)
            if sentence:
                if sentence.startswith("$GPGGA"):
                    self.gpgga_handler.handle(sentence)
                elif sentence.startswith("#BESTNAVA"):
                    self.bestnava_handler.handle(sentence)
                elif sentence.startswith("#HWSTATUSA"):
                    self.hwstatusa_handler.handle(sentence)
                else:
                    print(f"[SERVICE] Unknown or unhandled sentence: {sentence}")
        print("[SERVICE] Dispatcher thread ended")
