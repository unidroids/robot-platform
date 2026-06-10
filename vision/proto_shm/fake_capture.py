import os
import cv2
import time
import numpy as np
import glob
from multiprocessing import shared_memory

# --- 1. KONFIGURACE ---
# Cesta k reálným nasnímaným datům z Jetsonu
IMAGE_DIR = "/data/robot/cameras-2"  

W_IN, H_IN = 1232, 1640
CHANNELS = 3
HEADER_SIZE = 16
IMG_BYTES = W_IN * H_IN * CHANNELS
SHM_SIZE = HEADER_SIZE + IMG_BYTES
FPS_TARGET = 10.0  # Rychlost simulace (10 Hz)

def create_shm_and_pipe(side):
    """Vytvoří IDENTICKOU paměť a rouru jako nativní kamery."""
    shm_name = f'vision_shm_{side}'
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    
    # Úklid, pokud předtím běžely skutečné kamery nebo simulátor spadl
    try: shared_memory.SharedMemory(name=shm_name).unlink()
    except FileNotFoundError: pass
    if os.path.exists(pipe_path): os.remove(pipe_path)
    
    # Vytvoření nové paměti se stejným jménem
    shm = shared_memory.SharedMemory(name=shm_name, create=True, size=SHM_SIZE)
    os.mkfifo(pipe_path)
    
    # Otevření roury pro zápis (neblokující pro čtečku)
    fd = os.open(pipe_path, os.O_RDWR)
    pipe_file = os.fdopen(fd, 'w')
    
    return shm, pipe_file

if __name__ == "__main__":
    print(f"🎬 Startuji Kamerový Simulátor (Režim SITL - Software In The Loop)")
    print(f"📂 Načítám RAW snímky z: {IMAGE_DIR}")
    
    # Najde a seřadí obrázky, aby jízda dávala smysl chronologicky
    image_files = sorted(glob.glob(os.path.join(IMAGE_DIR, "*.jpg")))
    
    if not image_files:
        print("❌ V adresáři nejsou žádné .jpg soubory!")
        exit(1)
        
    print(f"✅ Nalezeno {len(image_files)} stereo obrázků.")

    # Vytvoření iluze běžících kamer
    print("🛠️ Vytvářím sdílenou paměť a IPC roury...")
    shm_L, pipe_L = create_shm_and_pipe('left')
    shm_R, pipe_R = create_shm_and_pipe('right')
    
    # Namapování přímo do paměti (od 16. bytu dál)
    img_data_L = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm_L.buf[HEADER_SIZE:])
    img_data_R = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm_R.buf[HEADER_SIZE:])

    print("🟢 Simulátor běží na 10 Hz! Můžeš spustit AI v Dockeru.")
    print("-" * 50)

    frame_seq = 0
    sleep_time = 1.0 / FPS_TARGET

    try:
        while True:  # Smyčka pro nekonečné přehrávání trasy
            for img_path in image_files:
                start_time = time.time()
                
                # 1. Načtení fotky (např. stereo_19700101_010145.jpg)
                img = cv2.imread(img_path)
                if img is None:
                    continue
                
                # 2. Rozseknutí stereo obrazu (očekáváme spojený 3280x1232)
                img_L = img[:, :W_IN]
                img_R = img[:, W_IN:]

                # 3. Zápis do sdílené paměti (tohle AI uvidí jako obraz z kamery)
                np.copyto(img_data_L, img_L)
                np.copyto(img_data_R, img_R)

                # 4. Synchronizace: Oklamání AI falešným časem zachycení
                capture_time = time.time() 
                
                pipe_L.write(f"{frame_seq}|{capture_time}\n")
                pipe_L.flush()
                
                pipe_R.write(f"{frame_seq}|{capture_time}\n")
                pipe_R.flush()

                if frame_seq % 10 == 0:
                    print(f"📡 Přehrávám: Frame {frame_seq} -> {os.path.basename(img_path)}")

                frame_seq += 1

                # Udržení stabilní simulované frekvence (10 FPS)
                elapsed = time.time() - start_time
                time_to_sleep = sleep_time - elapsed
                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)
                    
            print("🔄 Trasa dojeta na konec.")
            break


    except KeyboardInterrupt:
        print("\n🧯 Detekováno Ctrl+C, ukončuji simulaci.")
    finally:
        # 1. Uzavření a smazání LEVÉ strany
        shm_L.close()
        shm_L.unlink()   # Smaže RAM
        pipe_L.close()   # Zavře souborový popisovač
        if os.path.exists('/dev/shm/vision_sync_left.pipe'):
            os.remove('/dev/shm/vision_sync_left.pipe')  # SMAŽE ROURU!
        
        # 2. Uzavření a smazání PRAVÉ strany
        shm_R.close()
        shm_R.unlink()   # Smaže RAM
        pipe_R.close()   # Zavře souborový popisovač
        if os.path.exists('/dev/shm/vision_sync_right.pipe'):
            os.remove('/dev/shm/vision_sync_right.pipe') # SMAŽE ROURU!
            
        print("✅ Úklid IPC dokončen (RAM i Roury byly bezpečně smazány).")
