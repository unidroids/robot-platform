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

# Konfigurace rozlišení a SHM pro SLOŽENÝ obraz
# Původní obraz po rotaci má 1232x1640. Dva vedle sebe budou mít šířku 2464.
W_BEV, H_BEV = 2464, 1640
CHANNELS = 3

IMG_BYTES = W_BEV * H_BEV * CHANNELS
HEADER_SIZE = 16  # 8B (frame_seq) + 8B (timestamp)
SHM_SIZE = HEADER_SIZE + IMG_BYTES

class CameraService:
    def __init__(self):
        self.is_running = False
        self.thread = None
        self.loop = None
        
        self.shm = None
        self.img_data = None
        self.zmq_pub = None
        self.context = None
        
        self.frame_seq = 0
        self.pipeline = None

    def create_shm(self):
        """Vytvoří sdílenou paměť pro sloučený obraz."""
        shm_name = 'vision_shm_combined'
        try: 
            shared_memory.SharedMemory(name=shm_name).unlink()
        except FileNotFoundError: 
            pass
        return shared_memory.SharedMemory(name=shm_name, create=True, size=SHM_SIZE)

    def on_new_sample(self, sink, data=None):
        """Asynchronní callback volaný GStreamerem, když dorazí nový spojený snímek (10Hz)."""
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        
        if result:
            base_time = self.pipeline.get_base_time() if self.pipeline else 0
            capture_time = (base_time + buf.pts) / 1e9 if buf.pts != Gst.CLOCK_TIME_NONE else time.monotonic()
            
            raw_frame = np.ndarray(
                (H_BEV, W_BEV, CHANNELS),
                dtype=np.uint8,
                buffer=mapinfo.data
            )
            
            # Zámek
            struct.pack_into('q d', self.shm.buf, 0, -1, capture_time) 
            # Kopie dat
            np.copyto(self.img_data, raw_frame)
            # Odemčení
            struct.pack_into('q d', self.shm.buf, 0, self.frame_seq, capture_time) 
            
            # Publikování přes ZMQ (pouze jeden společný stream)
            message = f"combined/{self.frame_seq}/{capture_time}"
            self.zmq_pub.send_string(message)
            self.frame_seq += 1
            if self.frame_seq % 10 == 0:
                print(f"📌 DEBUG message: {message}")
                
            buf.unmap(mapinfo)
        
        return Gst.FlowReturn.OK

    def get_combined_camera_pipeline(self, log_file_pattern, sink_name):
        """
        GStreamer definice, která čte obě kamery, rotuje je, spojí vedle sebe do jednoho obrazu,
        větví ho pro 10 Hz výstup (BGR) a 1 Hz logování na disk (JPEG).
        """
        return (
            # --- Levá kamera (sensor-id=0, flip=3) ---
            f"nvarguscamerasrc sensor-id=0 aeregion=\"86 320 722 950 1.0\" exposuretimerange=\"34000 5000000\" gainrange=\"1.0 16.0\" ! "
            f"video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=10/1 ! "
            f"nvvidconv flip-method=3 ! video/x-raw(memory:NVMM), width=1232, height=1640 ! "
            f"queue max-size-buffers=2 leaky=downstream ! comp.sink_0 "
            
            # --- Pravá kamera (sensor-id=1, flip=1) ---
            f"nvarguscamerasrc sensor-id=1 aeregion=\"524 330 1148 954 1.0\" exposuretimerange=\"34000 5000000\" gainrange=\"1.0 16.0\" ! "
            f"video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=10/1 ! "
            f"nvvidconv flip-method=1 ! video/x-raw(memory:NVMM), width=1232, height=1640 ! "
            f"queue max-size-buffers=2 leaky=downstream ! comp.sink_1 "
            
            # --- Kompozitor (Složení obrazů vedle sebe) ---
            f"nvcompositor name=comp "
            f"sink_0::xpos=0 sink_0::ypos=0 sink_0::width=1232 sink_0::height=1640 "
            f"sink_1::xpos=1232 sink_1::ypos=0 sink_1::width=1232 sink_1::height=1640 ! "
            f"video/x-raw(memory:NVMM), width=2464, height=1640, format=RGBA, framerate=10/1 ! "
            f"tee name=t "
            
            # --- Větev 1: Aplikace (10 Hz BGRx -> BGR) ---
            f"t. ! queue max-size-buffers=2 leaky=downstream ! nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink name={sink_name} drop=true sync=false max-buffers=1 emit-signals=true "
            
            # --- Větev 2: Logování na disk (1 Hz JPEG) ---
            f"t. ! queue max-size-buffers=2 leaky=downstream ! videorate drop-only=true ! "
            f"video/x-raw(memory:NVMM), framerate=1/1 ! "
            f"nvvidconv ! video/x-raw(memory:NVMM), format=I420 ! "
            f"nvjpegenc quality=70 ! multifilesink location={log_file_pattern}"
        )

    def _run_loop(self):
        Gst.init(None)
        self.loop = GLib.MainLoop()

        self.context = zmq.Context()
        self.zmq_pub = self.context.socket(zmq.PUB)
        self.zmq_pub.bind("ipc:///tmp/robot-camera")

        self.shm = self.create_shm()
        self.img_data = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=self.shm.buf[16:])

        LOG_DIR = "/data/robot/camera"
        os.makedirs(LOG_DIR, exist_ok=True)
        
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S")
        
        BASE_LOG_DIR = os.path.join(LOG_DIR, date_str, time_str, "combined")
        os.makedirs(BASE_LOG_DIR, exist_ok=True)
        
        LOG_FILE_PATTERN = os.path.join(BASE_LOG_DIR, "frame_%06d.jpg")

        print("🚀 Inicializuji spojenou GStreamer pipeline přes `gi`...")
        
        pipe_str = self.get_combined_camera_pipeline(LOG_FILE_PATTERN, "appsink_combined")
        try:
            self.pipeline = Gst.parse_launch(pipe_str)
        except Exception as e:
            print(f"❌ GStreamer pipeline error: {e}")
            self.shm.close()
            self.shm.unlink()
            self.zmq_pub.close()
            self.context.term()
            return
        
        appsink = self.pipeline.get_by_name("appsink_combined")
        appsink.connect("new-sample", self.on_new_sample)

        print(f"🟢 Spouštím kamery... Logy do: {BASE_LOG_DIR}")
        self.pipeline.set_state(Gst.State.PLAYING)

        print("🎥 CameraService běží asynchronně.")
        self.is_running = True
        
        try:
            self.loop.run()
        except Exception as e:
            print(f"Vyjímka: {e}")

        print("🛑 CameraService: Posílám signál EOS pro čisté ukončení...")
        self.pipeline.send_event(Gst.Event.new_eos())
        
        time.sleep(1.0)
        
        self.pipeline.set_state(Gst.State.NULL)
        
        self.shm.close()
        self.shm.unlink()
        
        self.zmq_pub.close()
        self.context.term()
        
        self.is_running = False
        print("✅ CameraService: Vše bezpečně ukončeno.")

    def start(self) -> str:
        if self.is_running:
            return "ALREADY_RUNNING"
        
        self.frame_seq = 0
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
        for _ in range(20):
            if self.is_running: 
                return "OK"
            if not self.thread.is_alive():
                return "ERROR"
            time.sleep(0.1)
            
        return "TIMEOUT"

    def stop(self) -> bool:
        if not self.is_running or self.loop is None:
            return False
        
        self.loop.quit()
        if self.thread:
            self.thread.join(timeout=5.0)
        return True

    def get_status(self) -> str:
        if self.is_running:
            return f"RUNNING Combined Frames: {self.frame_seq}"
        return "IDLE"

# Globální instance
service = CameraService()