import os
import time
import cv2
import numpy as np
import torch
import torch.nn.functional as F

# --- KONFIGURACE ---
W_IN, H_IN = 1640, 1232  # Rozlišení původní kamery
TARGET_W, TARGET_H = 640, 480  # Cílové rozlišení pro YOLO
DUMMY_NPZ = "dummy_transform.npz"

def generate_dummy_npz():
    """Vytvoří cvičný .npz soubor s mapami 1200x900 (původní BEV)."""
    print("🛠️ Generuji cvičný .npz soubor pro test...")
    # Mapy původně měly velikost 1200x900 a obsahovaly float souřadnice z obrazu 1640x1232
    u_map = np.random.uniform(0, W_IN, (900, 1200)).astype(np.float32)
    v_map = np.random.uniform(0, H_IN, (900, 1200)).astype(np.float32)
    np.savez(DUMMY_NPZ, u_map_L=u_map, v_map_L=v_map)

def prepare_cuda_grid(npz_path, side, target_w=TARGET_W, target_h=TARGET_H, device='cuda'):
    print(f"⚙️ Načítám a zmenšuji BEV mapu pro kameru: {side} na {target_w}x{target_h}...")
    npz = np.load(npz_path)
    
    # Načtení původních map (1200x900)
    map_x_full = npz[f'u_map_{side}']
    map_y_full = npz[f'v_map_{side}']
    
    # ZMENŠENÍ MAP (Provede se pouze jednou při startu!)
    map_x = cv2.resize(map_x_full, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    map_y = cv2.resize(map_y_full, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    
    # Normalizace [-1, 1] vůči PŮVODNÍMU obrázku z kamery (protože W_IN, H_IN se nemění)
    grid_x = (2.0 * map_x / (W_IN - 1)) - 1.0
    grid_y = (2.0 * map_y / (H_IN - 1)) - 1.0
    
    # Výsledný grid má teď fyzický rozměr (480, 640, 2)
    grid = np.stack((grid_x, grid_y), axis=-1)
    return torch.from_numpy(grid).float().unsqueeze(0).to(device)

if __name__ == "__main__":
    print("🚀 Startuji benchmark transformace...")
    
    # 1. Příprava dat
    if not os.path.exists(DUMMY_NPZ):
        generate_dummy_npz()
        
    grid = prepare_cuda_grid(DUMMY_NPZ, 'L')
    
    # 2. Vytvoření cvičného "RAW" snímku (jako by přišel z RAM roury)
    raw_frame = np.random.randint(0, 255, (H_IN, W_IN, 3), dtype=np.uint8)
    
    print("\n🔥 KROK 1: Měření přesunu do GPU (1640x1232)")
    # Tohle se děje uvnitř smyčky (přesun z numpy CPU -> PyTorch GPU)
    t0 = time.perf_counter()
    img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).float().unsqueeze(0).to('cuda', non_blocking=True) / 255.0
    torch.cuda.synchronize()
    t_transfer = (time.perf_counter() - t0) * 1000
    print(f"   ⏱️ Přesun RAM -> GPU trval: {t_transfer:.3f} ms")

    print("\n🔥 KROK 2: Zahřívání CUDA jader (Warm-up)")
    for _ in range(10):
        _ = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
    torch.cuda.synchronize()
    print("   ✅ GPU je zahřáté a připravené.")

    print("\n🔥 KROK 3: Ostrý Benchmark BEV transformace (1000 průchodů)")
    iterations = 1000
    
    # Začátek měření
    torch.cuda.synchronize()
    t_start = time.perf_counter()
    
    for _ in range(iterations):
        bev_640 = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
        
    # Čekáme na fyzické dokončení posledního frame
    torch.cuda.synchronize()
    t_end = time.perf_counter()
    
    total_time = (t_end - t_start) * 1000
    avg_time = total_time / iterations
    
    print("-" * 50)
    print("📊 VÝSLEDKY BENCHMARKU:")
    print(f"Počet snímků:      {iterations}")
    print(f"Výsledný tvar:     {bev_640.shape} (Batch, Channels, Height, Width)")
    print(f"Celkový čas:       {total_time:.2f} ms")
    print(f"⏱️ PRŮMĚR NA 1 FRAME: {avg_time:.3f} ms")
    print("-" * 50)
    
    # Úklid
    if os.path.exists(DUMMY_NPZ):
        os.remove(DUMMY_NPZ)