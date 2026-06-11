import numpy as np
import time
import os
import zmq
import threading
import struct
from multiprocessing import shared_memory
from datetime import datetime

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Konfigurace rozlišení a SHM
W_IN, H_IN = 1232, 1640
W_BEV, H_BEV = W_IN, H_IN
CHANNELS = 3

IMG_BYTES = W_BEV * H_BEV * CHANNELS
HEADER_SIZE = 16  # 8B (frame_seq) + 8B (timestamp)
SHM_SIZE = HEADER_SIZE + IMG_BYTES

class CameraService:
    def __init__(self):
        self.is_running = False
        self.thread = None
        self.loop = None
        
        self.shm_L = None
        self.shm_R = None
        self.img_data_L = None
        self.img_data_R = None
        self.zmq_pub = None
        self.context = None
        
        self.frame_seq_L = 0
        self.frame_seq_R = 0
        self.pipeline_L = None
        self.pipeline_R = None

    def create_shm(self, side):
        """Vytvoří nezávislou paměť pro danou stranu (left/right)."""
        shm_name = f'vision_shm_{side}'
        try: 
            shared_memory.SharedMemory(name=shm_name).unlink()
        except FileNotFoundError: 
            pass
        shm = shared_memory.SharedMemory(name=shm_name, create=True, size=SHM_SIZE)
        return shm

    def on_new_sample(self, sink, side):
        """Asynchronní callback volaný GStreamerem, když dorazí nový snímek (10Hz)."""
        cb_start_time = time.monotonic()
        
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        
        if result:
            # Převedeme PTS na absolutní systémový monotónní čas
            base_time = self.pipeline_L.get_base_time() if side == 'left' and self.pipeline_L else \
                        self.pipeline_R.get_base_time() if side == 'right' and self.pipeline_R else 0
                        
            capture_time = (base_time + buf.pts) / 1e9 if buf.pts != Gst.CLOCK_TIME_NONE else time.monotonic()
            
            # Očekáváme BGR formát, data překopírujeme přímo do numpy pole namapovaného na SHM
            raw_frame = np.ndarray(
                (H_BEV, W_BEV, CHANNELS),
                dtype=np.uint8,
                buffer=mapinfo.data
            )
            
            if side == 'left':
                struct.pack_into('q d', self.shm_L.buf, 0, -1, capture_time) # zámek
                np.copyto(self.img_data_L, raw_frame)
                struct.pack_into('q d', self.shm_L.buf, 0, self.frame_seq_L, capture_time) # odemčení
                self.zmq_pub.send_string(f"left/{self.frame_seq_L}/{capture_time}")
                self.frame_seq_L += 1
            else:
                struct.pack_into('q d', self.shm_R.buf, 0, -1, capture_time) # zámek
                np.copyto(self.img_data_R, raw_frame)
                struct.pack_into('q d', self.shm_R.buf, 0, self.frame_seq_R, capture_time) # odemčení
                self.zmq_pub.send_string(f"right/{self.frame_seq_R}/{capture_time}")
                self.frame_seq_R += 1
                
            buf.unmap(mapinfo)
        
        return Gst.FlowReturn.OK

    def get_gst_camera_pipeline(self, sensor_id, flip_method, log_file_pattern, sink_name):
        # Logování do sekvence JPEG pro Machine Learning pomocí hardwarového nvjpegenc.
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=10/1 ! "
            f"tee name=t "
            f"t. ! queue max-size-buffers=1 ! nvvidconv flip-method={flip_method} ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink name={sink_name} drop=true sync=false max-buffers=1 emit-signals=true "
            f"t. ! queue max-size-buffers=1 ! nvvidconv flip-method={flip_method} ! video/x-raw, format=I420 ! "
            f"videorate drop-only=true ! video/x-raw, framerate=1/1 ! "
            f"nvvidconv ! video/x-raw(memory:NVMM), format=I420 ! "
            f"nvjpegenc quality=70 ! multifilesink location={log_file_pattern}"
        )

    def _run_loop(self):
        Gst.init(None)
        self.loop = GLib.MainLoop()

        self.context = zmq.Context()
        self.zmq_pub = self.context.socket(zmq.PUB)
        self.zmq_pub.bind("ipc:///tmp/robot-camera")

        self.shm_L = self.create_shm('left')
        self.shm_R = self.create_shm('right')

        self.img_data_L = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=self.shm_L.buf[16:])
        self.img_data_R = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=self.shm_R.buf[16:])

        LOG_DIR = "/data/robot/camera"
        os.makedirs(LOG_DIR, exist_ok=True)
        
        LOG_TIME_STR = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        LOG_DIR_L = os.path.join(LOG_DIR, f"{LOG_TIME_STR}_left")
        LOG_DIR_R = os.path.join(LOG_DIR, f"{LOG_TIME_STR}_right")
        os.makedirs(LOG_DIR_L, exist_ok=True)
        os.makedirs(LOG_DIR_R, exist_ok=True)
        
        LOG_FILE_PATTERN_L = os.path.join(LOG_DIR_L, "frame_%06d.jpg")
        LOG_FILE_PATTERN_R = os.path.join(LOG_DIR_R, "frame_%06d.jpg")

        print("🚀 Inicializuji kamery a GStreamer pipeline přes `gi`...")
        
        pipe_str_L = self.get_gst_camera_pipeline(0, 3, LOG_FILE_PATTERN_L, "appsink_L")
        self.pipeline_L = Gst.parse_launch(pipe_str_L)
        appsink_L = self.pipeline_L.get_by_name("appsink_L")
        appsink_L.connect("new-sample", self.on_new_sample, 'left')
        
        pipe_str_R = self.get_gst_camera_pipeline(1, 1, LOG_FILE_PATTERN_R, "appsink_R")
        self.pipeline_R = Gst.parse_launch(pipe_str_R)
        appsink_R = self.pipeline_R.get_by_name("appsink_R")
        appsink_R.connect("new-sample", self.on_new_sample, 'right')

        start_time = time.monotonic()

        print(f"🟢 Spouštím levou kameru... Logy do: {LOG_DIR_L}")
        self.pipeline_L.set_state(Gst.State.PLAYING)
        
        # Posun pro střídavé zpracování
        while time.monotonic() - start_time < 0.03:
            continue
        
        print(f"🟢 Spouštím pravou kameru... Logy do: {LOG_DIR_R}")
        self.pipeline_R.set_state(Gst.State.PLAYING)

        print("🎥 CameraService běží asynchronně.")
        self.is_running = True
        
        try:
            self.loop.run()
        except Exception as e:
            print(f"Vyjímka: {e}")

        print("🛑 CameraService: Posílám signál EOS pro čisté ukončení...")
        self.pipeline_L.send_event(Gst.Event.new_eos())
        self.pipeline_R.send_event(Gst.Event.new_eos())
        
        # Krátká pauza, aby matroskamux / nvjpegenc stačil zapsat data na disk
        time.sleep(1.0)
        
        self.pipeline_L.set_state(Gst.State.NULL)
        self.pipeline_R.set_state(Gst.State.NULL)
        
        self.shm_L.close()
        self.shm_L.unlink()
        self.shm_R.close()
        self.shm_R.unlink()
        
        self.zmq_pub.close()
        self.context.term()
        
        self.is_running = False
        print("✅ CameraService: Vše bezpečně ukončeno.")

    def start(self) -> bool:
        """Spustí GStreamer smyčku na pozadí, pokud už neběží."""
        if self.is_running:
            return False
        
        self.frame_seq_L = 0
        self.frame_seq_R = 0
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
        # Wait a moment for loop to actually start and set is_running
        for _ in range(20):
            if self.is_running: break
            time.sleep(0.1)
            
        return True

    def stop(self) -> bool:
        """Zastaví GStreamer smyčku a vyčistí prostředky."""
        if not self.is_running or self.loop is None:
            return False
        
        self.loop.quit()
        if self.thread:
            self.thread.join(timeout=5.0)
        return True

    def get_status(self) -> str:
        """Vrátí aktuální status služby jako text."""
        if self.is_running:
            return f"RUNNING L:{self.frame_seq_L} R:{self.frame_seq_R}"
        return "IDLE"

# Globální instance pro snadné použití
service = CameraService()
