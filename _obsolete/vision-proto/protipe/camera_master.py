import cv2
import numpy as np
import time
from multiprocessing import shared_memory

# Parametry obrazu
W, H = 1280, 720
CHANNELS = 3
FRAME_SIZE = W * H * CHANNELS

def create_shared_memory(name):
    """Vytvoří sdílenou paměť pro jeden snímek."""
    try:
        # Zkusíme smazat starou paměť, pokud po pádu zůstala viset
        shm = shared_memory.SharedMemory(name=name)
        shm.unlink()
    except FileNotFoundError:
        pass
    
    # Vytvoření nové sdílené paměti
    return shared_memory.SharedMemory(name=name, create=True, size=FRAME_SIZE)

def get_pipeline(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={W}, height={H}, format=NV12, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"
    )

print("Inicializuji sdílenou paměť...")
shm_left = create_shared_memory('camera_left')
shm_right = create_shared_memory('camera_right')

# Vytvoření numpy polí, která ukazují přímo do sdílené paměti (zápis do nich se rovnou propíše do RAM)
frame_left_shm = np.ndarray((H, W, CHANNELS), dtype=np.uint8, buffer=shm_left.buf)
frame_right_shm = np.ndarray((H, W, CHANNELS), dtype=np.uint8, buffer=shm_right.buf)

print("Otevírám kamery...")
cap_left = cv2.VideoCapture(get_pipeline(0), cv2.CAP_GSTREAMER)
cap_right = cv2.VideoCapture(get_pipeline(1), cv2.CAP_GSTREAMER)

# Nastavení logování (VideoWriter) - 1 Hz znamená 1 FPS pro video
# H.264 hardwarový kodek pro Jetson (aby to nezatěžovalo CPU)
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
logger = cv2.VideoWriter('/data/logs/camera/stereo_log.mp4', fourcc, 1.0, (W*2, H))

last_log_time = time.time()
print("Kamery běží. Poskytuji data do sdílené paměti...")

try:
    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()

        if not retL or not retR:
            print("Chyba čtení z kamer, zkouším znovu...")
            time.sleep(0.1)
            continue

        # 1. ZÁPIS DO SDÍLENÉ PAMĚTI (To si přečte Docker)
        # Pouhé zkopírování dat do předpřipravené matice (téměř 0ms latence)
        np.copyto(frame_left_shm, frameL)
        np.copyto(frame_right_shm, frameR)

        # 2. LOGOVÁNÍ NA DISK (1 Hz)
        current_time = time.time()
        if current_time - last_log_time >= 1.0: # Uběhla 1 vteřina
            # Spojíme snímky vedle sebe a zapíšeme do videa
            stereo_frame = np.hstack((frameL, frameR))
            logger.write(stereo_frame)
            last_log_time = current_time

except KeyboardInterrupt:
    print("\nUkončuji camera_master...")
finally:
    cap_left.release()
    cap_right.release()
    logger.release()
    # Úklid sdílené paměti
    shm_left.close()
    shm_left.unlink()
    shm_right.close()
    shm_right.unlink()
    print("Paměť uvolněna.")