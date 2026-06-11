import numpy as np
import time
import os
import zmq
import signal
import sys
import struct
from multiprocessing import shared_memory
from datetime import datetime

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# --- 1. KONFIGURACE ROZLIŠENÍ (Již otočeno!) ---
W_IN, H_IN = 1232, 1640  # Výsledek po VIC flip-method
W_BEV, H_BEV = W_IN, H_IN

CHANNELS = 3

IMG_BYTES = W_BEV * H_BEV * CHANNELS
HEADER_SIZE = 16  # 8B (frame_seq) + 8B (timestamp)
SHM_SIZE = HEADER_SIZE + IMG_BYTES

LOG_DIR = "/data/robot/camera"
os.makedirs(LOG_DIR, exist_ok=True)

# Společný časový prefix pro logy obou kamer
LOG_TIME_STR = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
LOG_DIR_L = os.path.join(LOG_DIR, f"{LOG_TIME_STR}_left")
LOG_DIR_R = os.path.join(LOG_DIR, f"{LOG_TIME_STR}_right")
os.makedirs(LOG_DIR_L, exist_ok=True)
os.makedirs(LOG_DIR_R, exist_ok=True)

LOG_FILE_PATTERN_L = os.path.join(LOG_DIR_L, "frame_%06d.jpg")
LOG_FILE_PATTERN_R = os.path.join(LOG_DIR_R, "frame_%06d.jpg")

# --- 2. ZMQ A SHM INICIALIZACE ---
def create_shm(side):
    """Vytvoří nezávislou paměť pro danou stranu (left/right)."""
    shm_name = f'vision_shm_{side}'
    try: 
        shared_memory.SharedMemory(name=shm_name).unlink()
    except FileNotFoundError: 
        pass
    shm = shared_memory.SharedMemory(name=shm_name, create=True, size=SHM_SIZE)
    return shm

context = zmq.Context()
zmq_pub = context.socket(zmq.PUB)
zmq_pub.bind("ipc:///tmp/vision_sync")

# Globální stavy
shm_L = create_shm('left')
shm_R = create_shm('right')

img_data_L = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=shm_L.buf[16:])
img_data_R = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=shm_R.buf[16:])

frame_seq_L = 0
frame_seq_R = 0

# --- 3. CALLBACKY PRO GStreamer ---
def on_new_sample(sink, side):
    """Asynchronní callback volaný GStreamerem, když dorazí nový snímek (10Hz)."""
    global frame_seq_L, frame_seq_R
    
    sample = sink.emit("pull-sample")
    if not sample:
        return Gst.FlowReturn.ERROR

    buf = sample.get_buffer()
    result, mapinfo = buf.map(Gst.MapFlags.READ)
    
    if result:
        # Původně time.time(), nyní používáme Presentation Time Stamp z metadat GStreameru
        capture_time = buf.pts / 1e9 if buf.pts != Gst.CLOCK_TIME_NONE else time.time()
        #if buf.pts == Gst.CLOCK_TIME_NONE:
        #    print(f"time = {time.time()}, capture_time = {capture_time}, frame_seq = {frame_seq_L if side == 'left' else frame_seq_R}, side = {side}", flush=True)
        #else:
        #    print(f"buf.pts = {buf.pts}, capture_time = {capture_time}, frame_seq = {frame_seq_L if side == 'left' else frame_seq_R}, side = {side}", flush=True)
        # Očekáváme BGR formát, data překopírujeme přímo do numpy pole namapovaného na SHM
        raw_frame = np.ndarray(
            (H_BEV, W_BEV, CHANNELS),
            dtype=np.uint8,
            buffer=mapinfo.data
        )
        
        if side == 'left':
            struct.pack_into('q d', shm_L.buf, 0, -1, capture_time) # zámek
            np.copyto(img_data_L, raw_frame)
            struct.pack_into('q d', shm_L.buf, 0, frame_seq_L, capture_time) # odemčení
            zmq_pub.send_string(f"left/{frame_seq_L}/{capture_time}")
            frame_seq_L += 1
            if frame_seq_L % 20 == 0:
                print(f"✅ Zpracováno {frame_seq_L} levých a {frame_seq_R} pravých snímků", flush=True)
        else:
            struct.pack_into('q d', shm_R.buf, 0, -1, capture_time) # zámek
            np.copyto(img_data_R, raw_frame)
            struct.pack_into('q d', shm_R.buf, 0, frame_seq_R, capture_time) # odemčení
            zmq_pub.send_string(f"right/{frame_seq_R}/{capture_time}")
            frame_seq_R += 1
            
        buf.unmap(mapinfo)
    
    return Gst.FlowReturn.OK

def get_gst_camera_pipeline(sensor_id, flip_method, log_file_pattern, sink_name):
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

loop = GLib.MainLoop()

def signal_handler(sig, frame):
    print("\n🧯 Detekováno Ctrl+C, zahajuji čisté vypnutí GStreameru...")
    loop.quit()

def main():
    Gst.init(None)
    signal.signal(signal.SIGINT, signal_handler)

    print("🚀 Inicializuji kamery a GStreamer pipeline přes `gi`...")
    
    # --- LEVÁ KAMERA ---
    pipe_str_L = get_gst_camera_pipeline(0, 3, LOG_FILE_PATTERN_L, "appsink_L")
    pipeline_L = Gst.parse_launch(pipe_str_L)
    appsink_L = pipeline_L.get_by_name("appsink_L")
    appsink_L.connect("new-sample", on_new_sample, 'left')
    
    # --- PRAVÁ KAMERA ---
    pipe_str_R = get_gst_camera_pipeline(1, 1, LOG_FILE_PATTERN_R, "appsink_R")
    pipeline_R = Gst.parse_launch(pipe_str_R)
    appsink_R = pipeline_R.get_by_name("appsink_R")
    appsink_R.connect("new-sample", on_new_sample, 'right')

    start_time = time.time()

    print(f"🟢 Spouštím levou kameru... Logy do: {LOG_DIR_L}")
    pipeline_L.set_state(Gst.State.PLAYING)
    
    # Posun pro střídavé zpracování
    while time.time() - start_time < 0.03:
        continue
    
    print(f"🟢 Spouštím pravou kameru... Logy do: {LOG_DIR_R}")
    pipeline_R.set_state(Gst.State.PLAYING)

    print("🎥 Systém běží čistě asynchronně. Pro ukončení stiskněte Ctrl+C.")
    
    try:
        loop.run()
    except Exception as e:
        print(f"Vyjímka: {e}")
        
    print("🛑 Posílám signál EOS pro čisté ukončení...")
    pipeline_L.send_event(Gst.Event.new_eos())
    pipeline_R.send_event(Gst.Event.new_eos())
    
    # Krátká pauza, aby matroskamux stačil zapsat data na disk
    time.sleep(1.0)
    
    pipeline_L.set_state(Gst.State.NULL)
    pipeline_R.set_state(Gst.State.NULL)
    
    shm_L.close()
    shm_L.unlink()
    shm_R.close()
    shm_R.unlink()
    
    zmq_pub.close()
    context.term()
        
    print("✅ Vše bezpečně ukončeno. Můžeš spustit znovu.")

if __name__ == "__main__":
    main()