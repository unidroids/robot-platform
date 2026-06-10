import cv2
import numpy as np
import time
import os
import threading
from multiprocessing import shared_memory

# --- 1. KONFIGURACE ROZLIŠENÍ (Již otočeno!) ---
W_IN, H_IN = 1232, 1640  # Výsledek po VIC flip-method
W_BEV, H_BEV = W_IN, H_IN

CHANNELS = 3

IMG_BYTES = W_BEV * H_BEV * CHANNELS
HEADER_SIZE = 16  # 8B (frame_seq) + 8B (timestamp)
SHM_SIZE = HEADER_SIZE + IMG_BYTES

LOG_DIR = "/data/robot/camera"
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f"stereo_raw_{int(time.time())}.mp4")

# --- 2. SDÍLENÝ STAV PRO LOGGER ---
latest_raw_L = None
latest_raw_R = None
raw_lock = threading.Lock()
shutdown_flag = threading.Event()

# --- 3. POMOCNÉ FUNKCE IPC ---
def create_shm_and_pipe(side):
    """Vytvoří nezávislou paměť a rouru pro danou stranu (left/right)."""
    shm_name = f'vision_shm_{side}'
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    
    # Úklid staré paměti a roury
    try: shared_memory.SharedMemory(name=shm_name).unlink()
    except FileNotFoundError: pass
    if os.path.exists(pipe_path): os.remove(pipe_path)
    
    # Vytvoření nových
    shm = shared_memory.SharedMemory(name=shm_name, create=True, size=SHM_SIZE)
    os.mkfifo(pipe_path)
    
    # O_RDWR zabrání zablokování, i když Docker ještě nečte
    fd = os.open(pipe_path, os.O_RDWR)
    pipe_file = os.fdopen(fd, 'w')
    
    return shm, pipe_file

def get_gst_camera(sensor_id, flip_method):
    # Senzor čte 1640x1232 (nativní), nvvidconv to hardwarově otočí pomocí flip-method
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=10/1 ! "
        f"nvvidconv flip-method={flip_method} ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"
    )
    
# --- 4. VLÁKNO KAMERY ---
def camera_worker(side, sensor_id, flip_method):
    global latest_raw_L, latest_raw_R
    
    print(f"[{side}] Inicializuji kameru a IPC...")
    shm, sync_pipe = create_shm_and_pipe(side)
    
    # Namapování struktury
    img_data = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=shm.buf[16:])
    
    cap = cv2.VideoCapture(get_gst_camera(sensor_id, flip_method), cv2.CAP_GSTREAMER)
    frame_seq = 0
    
    print(f"🟢 [{side}] Kamera běží.")
    
    while not shutdown_flag.is_set():
        ret, raw_frame = cap.read()
        capture_time = time.time()
        
        if not ret:
            continue
            
        # 1. Odeslání RAW obrazu (už otočeného) do bufferu pro Logger
        with raw_lock:
            if side == 'left': latest_raw_L = raw_frame
            else: latest_raw_R = raw_frame
            
        # 2. Zápis do RAM (žádné BEV úpravy se tu nedělají, to řeší Docker!)
        np.copyto(img_data, raw_frame)
        
        # 3. Synchronizační bariéra pro Docker
        sync_pipe.write(f"{frame_seq}|{capture_time}\n")
        sync_pipe.flush()
        
        frame_seq += 1

    # Úklid vlákna
    cap.release()
    shm.close()
    shm.unlink()
    sync_pipe.close()

    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    if os.path.exists(pipe_path):
        os.remove(pipe_path)

    print(f"🛑 [{side}] Ukončeno.")

# --- 5. VLÁKNO LOGGERU (1 Hz) ---
def logger_worker():
    print("📝 [logger] Inicializuji hardwarový enkodér...")
    pipeline = (
        f"appsrc ! video/x-raw, format=BGR ! "
        "videoconvert ! video/x-raw, format=I420 ! "
        "nvv4l2h264enc bitrate=4000000 insert-sps-pps=true ! "
        f"h264parse ! mp4mux ! filesink location={LOG_FILE}"
    )
    
    # VideoWriter se automaticky přizpůsobí. 
    # Bude mít na šířku 1232 * 2 = 2464, a výšku 1640.
    writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 1.0, (W_IN * 2, H_IN))
    
    print("🟢 [logger] Běží (1 Hz).")
    while not shutdown_flag.is_set():
        time.sleep(1.0)
        
        with raw_lock:
            rL = latest_raw_L
            rR = latest_raw_R
            
        if rL is not None and rR is not None:
            stereo_raw = np.hstack((rL, rR))
            writer.write(stereo_raw)

    writer.release()
    print("🛑 [logger] Video uloženo a uzavřeno.")

# --- HLAVNÍ PROGRAM ---
if __name__ == "__main__":
    # Předáváme parametr 'flip_method' (3 a 1)
    t_left = threading.Thread(target=camera_worker, args=('left', 0, 3))
    t_right = threading.Thread(target=camera_worker, args=('right', 1, 1))
    t_logger = threading.Thread(target=logger_worker)

    t_left.start()
    t_right.start()
    t_logger.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n🧯 Detekováno Ctrl+C, zahajuji čisté vypnutí...")
        shutdown_flag.set()
        
        t_left.join()
        t_right.join()
        t_logger.join()
        print("✅ Vše bezpečně ukončeno. Můžeš spustit znovu.")