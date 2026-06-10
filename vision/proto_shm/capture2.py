import cv2
import numpy as np
import time
import os
from multiprocessing import shared_memory

# --- CONFIG (Spojené rozlišení) ---
W_EACH, H_EACH = 1232, 1640  # Rozměr jedné otočené kamery
W_COMB, H_COMB = W_EACH * 2, H_EACH  # 2464 x 1640 (Slepeno vedle sebe)
CHANNELS = 3
HEADER_SIZE = 16
SHM_SIZE = HEADER_SIZE + (W_COMB * H_COMB * CHANNELS)

LOG_DIR = "/data/robot/camera"
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f"combined_{int(time.time())}.mp4")

def get_stereo_pipeline():
    """Neprůstřelná GStreamer pipeline pro Jetson nvcompositor."""
    return (
        "nvcompositor name=comp "
        f"sink_0::xpos=0 sink_0::ypos=0 sink_0::width={W_EACH} sink_0::height={H_EACH} "
        f"sink_1::xpos={W_EACH} sink_1::ypos=0 sink_1::width={W_EACH} sink_1::height={H_EACH} ! "
        # 1. Kompozitor musí znát velikost finálního plátna (2464x1640) a vyžaduje RGBA
        f"video/x-raw(memory:NVMM), width={W_COMB}, height={H_COMB}, format=RGBA ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false "
        
        # Levá kamera (ID 0) -> Převod do RGBA těsně před vstupem do kompozitoru
        f"nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=10/1 ! "
        f"nvvidconv flip-method=3 ! video/x-raw(memory:NVMM), width={W_EACH}, height={H_EACH}, format=RGBA ! comp.sink_0 "
        
        # Pravá kamera (ID 1) -> Převod do RGBA těsně před vstupem do kompozitoru
        f"nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=10/1 ! "
        f"nvvidconv flip-method=1 ! video/x-raw(memory:NVMM), width={W_EACH}, height={H_EACH}, format=RGBA ! comp.sink_1"
    )

if __name__ == "__main__":
    print("🎬 Startuji Konsolidovaný Kamerový Systém...")
    
    # 1. Příprava jedné sdílené paměti a roury
    SHM_NAME = "vision_shm_stereo"
    PIPE_PATH = "/dev/shm/vision_sync_stereo.pipe"
    
    try: shared_memory.SharedMemory(name=SHM_NAME).unlink()
    except FileNotFoundError: pass
    if os.path.exists(PIPE_PATH): os.remove(PIPE_PATH)
    
    shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    os.mkfifo(PIPE_PATH)
    fd = os.open(PIPE_PATH, os.O_RDWR)
    sync_pipe = os.fdopen(fd, 'w')
    
    img_data = np.ndarray((H_COMB, W_COMB, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])

    # 2. Inicializace kamer a hardwarového nahrávání
    cap = cv2.VideoCapture(get_stereo_pipeline(), cv2.CAP_GSTREAMER)
    
    log_pipeline = (
        f"appsrc ! video/x-raw, format=BGR ! videoconvert ! video/x-raw, format=I420 ! "
        f"nvv4l2h264enc bitrate=4000000 ! h264parse ! mp4mux ! filesink location={LOG_FILE}"
    )
    # Nahráváme přímo ten složený obraz 1x za vteřinu (1 Hz)
    video_logger = cv2.VideoWriter(log_pipeline, cv2.CAP_GSTREAMER, 0, 1.0, (W_COMB, H_COMB))

    print("🟢 Stereo kamera i logger běží na jedné vlně. Čekám na Docker...")
    
    frame_seq = 0
    last_log_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret: continue
            
            capture_time = time.time()
            
            # Zápis celého spojeného obrazu do RAM
            np.copyto(img_data, frame)
            
            # Uvolnění bariéry pro Docker
            sync_pipe.write(f"{frame_seq}|{capture_time}\n")
            sync_pipe.flush()
            
            # Odlehčené logování na 1 Hz bez nutnosti vláken a zámků
            if capture_time - last_log_time >= 1.0:
                video_logger.write(frame)
                last_log_time = capture_time
                
            frame_seq += 1

    except KeyboardInterrupt:
        print("\n🧯 Vypínám kamery...")
    finally:
        cap.release()
        video_logger.release()
        shm.close()
        shm.unlink()
        sync_pipe.close()
        if os.path.exists(PIPE_PATH): os.remove(PIPE_PATH)
        print("✅ Úklid dokončen.")