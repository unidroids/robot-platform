import os
import time
import numpy as np
import threading
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister

# Volitelně zde později importuješ YOLO
# from ultralytics import YOLO
# model = YOLO('yolo11n.pt')

# --- KONFIGURACE (Musí přesně odpovídat nativní službě!) ---
#W_BEV, H_BEV = 1200, 900
W_IN, H_IN = 1640, 1232
W_BEV, H_BEV = W_IN, H_IN

CHANNELS = 3

IMG_BYTES = W_BEV * H_BEV * CHANNELS
HEADER_SIZE = 16
SHM_SIZE = HEADER_SIZE + IMG_BYTES

def vision_worker(side):
    """Nezávislé vlákno pro zpracování jedné kamery."""
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    shm_name = f'vision_shm_{side}'

    print(f"[{side}] Čekám na spuštění nativní služby (rouru)...")
    # Skript nezkolabuje, pokud nativní část ještě neběží, prostě počká
    while not os.path.exists(pipe_path):
        time.sleep(0.5)

    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        # OPRAVA PYTHON BUGU: Zabráníme Dockeru, aby po ukončení paměť smazal
        unregister(shm._name, 'shared_memory')
    except FileNotFoundError:
        print(f"❌ [{side}] Chyba: Sdílená paměť neexistuje!")
        return

    # Namapování přímo na sekci s obrazem (přeskakujeme 16 bajtů hlavičky)
    img_data = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=shm.buf[16:])

    print(f"🟢 [{side}] Připojeno k RAM a rouře. Začínám číst data...")

    # Otevření roury pro čtení
    with open(pipe_path, 'r') as sync_pipe:
        while True:
            # 1. ČEKÁNÍ NA SIGNÁL (Tady vlákno spí a bere 0% CPU)
            line = sync_pipe.readline()
            
            if not line:
                print(f"🛑 [{side}] Roura byla uzavřena (Nativní master skončil).")
                break
            
            # 2. ROZLUŠTĚNÍ METADAT
            frame_seq, capture_time = line.strip().split('|')
            capture_time = float(capture_time)
            
            # 3. BEZPEČNÉ PŘEČTENÍ OBRAZU (Blesková lokální kopie BEV)
            # Protože máme signál z roury, víme na 100%, že data v RAM jsou kompletní
            local_frame = img_data.copy()
            
            # 4. VÝPOČET ZPOŽDĚNÍ (Čas mezi kamerou a Dockerem)
            latency = (time.time() - capture_time) * 1000
            
            # --- ZDE PŘIJDE TVÉ YOLO ZPRACOVÁNÍ ---
            # results = model.predict(local_frame, verbose=False)
            # data_ven = format_json(results, capture_time)
            # tcp_socket.send(data_ven)
            
            # Pro testovací účely vypíšeme info každý 30. frame (cca 1x za vteřinu)
            if int(frame_seq) % 30 == 0:
                print(f"[{side}] Frame: {frame_seq} | Latence IPC: {latency:.2f} ms | Čas: {capture_time:.2f}")

    shm.close()

if __name__ == "__main__":
    print("🚀 Startuji Vision AI Mikroslužbu v Dockeru...")
    
    t_left = threading.Thread(target=vision_worker, args=('left',))
    t_right = threading.Thread(target=vision_worker, args=('right',))

    t_left.start()
    t_right.start()

    try:
        # Místo nekonečného spánku se ptáme: "Žije ještě alespoň jedno vlákno?"
        while t_left.is_alive() or t_right.is_alive():
            time.sleep(1) # Hlavní vlákno odpočívá, aby nežralo CPU
            
        print("ℹ️ Nativní služba ukončila zápis. Všechna vlákna přirozeně doběhla.")
        
    except KeyboardInterrupt:
        print("\n🧯 Detekováno Ctrl+C (Uživatel přerušil program).")
        # Zde by se případně nastavil shutdown_flag, kdybyom měli nějakou delší smyčku

    finally:
        # .join() zaručí, že hlavní program nespadne / neukončí se dříve, 
        # než obě pracovní vlákna provedou svůj závěrečný úklid (shm.close() atd.)
        print("⏳ Čekám na bezpečné uzavření pamětí...")
        t_left.join()
        t_right.join()
        print("✅ Vision služba bezpečně vypnuta.")