import os
import time
import numpy as np
import threading
import torch
import torch.nn.functional as F
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
from ultralytics import YOLO

# ==========================================
# 1. KONFIGURACE PAMĚTI A OBRAZU
# ==========================================
W_IN, H_IN = 1640, 1232  # Surové rozlišení JEDNÉ kamery v RAM
CHANNELS = 3
HEADER_SIZE = 16
IMG_BYTES = W_IN * H_IN * CHANNELS

# ==========================================
# 2. PŘÍPRAVA AI A TRANSFORMAČNÍCH MATIC
# ==========================================
print("🚀 Načítám YOLOv8 Pose model...")
# Načtení tvého vlastního natrénovaného modelu
model = YOLO('best.pt') 
model.fuse()         # Optimalizace modelu (sloučení vrstev pro rychlost)
model.model.eval()   # Přepnutí modelu do produkčního (inference) režimu

def prepare_cuda_grid(npz_path, side, device='cuda'):
    """Připraví CUDA grid načtením parametrů z tvého npz souboru."""
    print(f"⚙️ Načítám BEV mapu pro kameru: {side}...")
    npz = np.load(npz_path)
    
    # Názvy klíčů přesně podle tvého exportéru (u_map_L, v_map_L, atd.)
    map_x = npz[f'u_map_{side}']
    map_y = npz[f'v_map_{side}']
    
    # Grid_sample vyžaduje normalizaci [-1, 1] vůči původnímu obrazu (1640x1232)
    grid_x = (2.0 * map_x / (W_IN - 1)) - 1.0
    grid_y = (2.0 * map_y / (H_IN - 1)) - 1.0
    
    # Sloučení do formátu (H, W, 2) a odeslání do GPU
    grid = np.stack((grid_x, grid_y), axis=-1)
    return torch.from_numpy(grid).float().unsqueeze(0).to(device)

# Načtení map (spustí se jen jednou při startu kontejneru)
# Cesta k tvému npz souboru, který jsi vytvořil exportérem
NPZ_FILE = "00_bev_transform.npz"
grid_L = prepare_cuda_grid(NPZ_FILE, 'L')
grid_R = prepare_cuda_grid(NPZ_FILE, 'R')

# ==========================================
# 3. PRACOVNÍ VLÁKNO PRO KAŽDOU KAMERU
# ==========================================
def vision_worker(side, grid):
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    shm_name = f'vision_shm_{side}'

    print(f"[{side}] Čekám na spuštění nativní služby...")
    while not os.path.exists(pipe_path):
        time.sleep(0.5)

    # Připojení do sdílené paměti zapsané nativním systémem
    shm = shared_memory.SharedMemory(name=shm_name)
    unregister(shm._name, 'shared_memory') # Ochrana proti automatickému smazání
    
    # Namapování obrazu
    img_data = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])

    print(f"🟢 [{side}] Připojeno k RAM a připraveno k inference na CUDA!")

    with open(pipe_path, 'r') as sync_pipe:
        while True:
            line = sync_pipe.readline()
            if not line: break # Roura byla uzavřena
            
            frame_seq, capture_time = line.strip().split('|')
            capture_time = float(capture_time)
            
            # --- ŘEŠENÍ BUFFER BLOATU (Zahození starých snímků) ---
            # Pokud nám snímek ležel v rouře déle než 100ms, zahodíme ho.
            # Tím držíme robota 100% v reálném čase.
            if (time.time() - capture_time) * 1000 > 100:
                continue
                
            # 1. Bleskové zkopírování snímku z RAM (ochrana proti přepsání)
            raw_frame = img_data.copy()
            
            # ========================================================
            # ⏱️ BEV TRANSFORMACE (GPU)
            # ========================================================
            t_start = time.perf_counter()
            
            # Přesun do GPU: (H, W, C) -> (1, C, H, W)
            img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).float().unsqueeze(0).to('cuda', non_blocking=True) / 255.0
            
            # CUDA Transformace (mode='bicubic' jako ve tvém tréninku)
            bev_tensor = F.grid_sample(img_tensor, grid, mode='bicubic', align_corners=True)
            
            # Padding pro YOLO (stride 32: z 1200x900 na 1216x928)
            padded_bev = F.pad(bev_tensor, (0, 16, 0, 28), mode='constant', value=0)
            
            torch.cuda.synchronize() # Počkáme, až GPU dokončí práci pro měření
            transform_latency = (time.perf_counter() - t_start) * 1000
            
            # ========================================================
            # 🤖 YOLO POSE INFERENCE
            # ========================================================
            # YOLO dostává obraz, který je už fyzicky umístěn v GPU
            #results = model.predict(padded_bev, imgsz=1216, verbose=False, device=0)
            
            # TIP: Zde bude následovat zpracování klíčových bodů a odeslání na UDP
            # if results[0].keypoints is not None:
            #     points = results[0].keypoints.xy.cpu().numpy()
            
            # Výpočet celkové latence od HW čipu kamery až po hotovou detekci
            total_latency = (time.time() - capture_time) * 1000
            
            # Výpis do konzole každý 30. snímek
            if int(frame_seq) % 10 == 0:
                print(f"[{side}] Frame: {frame_seq} | BEV (GPU): {transform_latency:.2f} ms | Latence celkem: {total_latency:.2f} ms")

    shm.close()

# ==========================================
# 4. SPRÁVCE VLÁKEN (MAIN)
# ==========================================
if __name__ == "__main__":
    print("🚀 Startuji Vision AI Mikroslužbu v Dockeru...")
    
    # Zásadní ochrana pro Jetson (omezí každé vlákno na 40 % kapacity GPU paměti)
    torch.cuda.set_per_process_memory_fraction(0.4)
    
    # Spuštění čteček pro levou a pravou kameru
    t_left = threading.Thread(target=vision_worker, args=('L', grid_L))
    t_right = threading.Thread(target=vision_worker, args=('R', grid_R))
    
    t_left.start()
    t_right.start()

    try:
        # Hlavní vlákno jemně hlídá, zda pracovníci žijí
        while t_left.is_alive() or t_right.is_alive():
            time.sleep(1)
            
        print("ℹ️ Nativní služba ukončila zápis. Všechna vlákna přirozeně doběhla.")
        
    except KeyboardInterrupt:
        print("\n🧯 Detekováno Ctrl+C (Uživatel přerušil program).")

    finally:
        print("⏳ Čekám na bezpečné uzavření vláken...")
        t_left.join()
        t_right.join()
        print("✅ Vision služba bezpečně vypnuta.")