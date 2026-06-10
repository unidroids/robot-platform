import cv2
import numpy as np
import time
import os
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister

# 1. Parametry obrazu z kamer (musí odpovídat camera_master.py)
W, H = 1280, 720
CHANNELS = 3

print("Připojuji se ke sdílené paměti kamer...")
try:
    # Připojení k existující paměti
    shm_left = shared_memory.SharedMemory(name='camera_left')
    shm_right = shared_memory.SharedMemory(name='camera_right')

    # OPRAVA VAROVÁNÍ: Řekneme Pythonu v Dockeru, ať se nesnaží paměť při ukončení zničit
    # (Protože skutečným vlastníkem je nativní skript camera_master.py)
    unregister(shm_left._name, 'shared_memory')
    unregister(shm_right._name, 'shared_memory')

    # Namapování paměti přímo na numpy pole
    frameL = np.ndarray((H, W, CHANNELS), dtype=np.uint8, buffer=shm_left.buf)
    frameR = np.ndarray((H, W, CHANNELS), dtype=np.uint8, buffer=shm_right.buf)
    print("Úspěšně připojeno k RAM!")
    
except FileNotFoundError:
    print("Chyba: Sdílená paměť neexistuje! Běží nativně 'camera_master.py'?")
    exit(1)

output_dir = "test_output_raw"
os.makedirs(output_dir, exist_ok=True)
print(f"Začínám číst data. Snímky se ukládají do složky '{output_dir}'. (Stiskni Ctrl+C pro ukončení)")

frame_count = 0
last_save_time = time.time()

try:
    while True:
        # Vytvoříme rychlou kopii aktuálního stavu paměti
        current_frameL = frameL.copy()
        current_frameR = frameR.copy()

        # Uložíme snímek každou 1 vteřinu (abychom nezaplnili disk)
        current_time = time.time()
        if current_time - last_save_time >= 1.0:
            # Spojíme levý a pravý obraz vedle sebe
            combined = np.hstack((current_frameL, current_frameR))
            
            filename = os.path.join(output_dir, f"raw_frame_{frame_count}.jpg")
            cv2.imwrite(filename, combined)
            print(f"Uložen čistý snímek z paměti: {filename}")
            
            last_save_time = current_time
            frame_count += 1
            
        time.sleep(0.05) # Malá pauza, aby smyčka nevytěžovala procesor na 100%

except KeyboardInterrupt:
    print("\nUkončuji přijímač...")

finally:
    # Jen uzavřeme náš "přístup" k paměti, paměť samotná zůstává žít pro camera_master
    shm_left.close()
    shm_right.close()
    print("Odpojeno ze sdílené paměti. Žádné úniky paměti nehrozí!")