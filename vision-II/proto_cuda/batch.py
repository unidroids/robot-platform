import os
import time
import cv2
import numpy as np
import torch
import torch.nn.functional as F
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
from ultralytics import YOLO

# --- KONFIGURACE ---
W_IN, H_IN = 1232, 1640
TARGET_W, TARGET_H = 640, 480
CHANNELS = 3
HEADER_SIZE = 16
NPZ_FILE = "00_bev_transform.npz"

print("🚀 Startuji Vision Mikroslužbu (Režim BATCH=2)...")

def prepare_batched_grid(npz_path, target_w=TARGET_W, target_h=TARGET_H, device='cuda'):
    """Připraví spojenou BEV mřížku [2, H, W, 2] pro jednorázovou transformaci obou očí."""
    npz = np.load(npz_path)
    grids = []
    for side in ['L', 'R']:
        map_x = cv2.resize(npz[f'u_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        map_y = cv2.resize(npz[f'v_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        grid_x = (2.0 * map_x / (W_IN - 1)) - 1.0
        grid_y = (2.0 * map_y / (H_IN - 1)) - 1.0
        grids.append(torch.from_numpy(np.stack((grid_x, grid_y), axis=-1)).half().to(device))
    
    # Slepí mřížku pro levou a pravou kameru do jednoho tensoru
    return torch.stack(grids, dim=0)

def init_ipc(side):
    """Pomocná funkce pro bezpečné připojení ke sdílené paměti."""
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    shm_name = f'vision_shm_{side}'
    print(f"[{side}] Čekám na spuštění nativní služby...")
    
    shm = None
    while shm is None:
        if os.path.exists(pipe_path):
            try: shm = shared_memory.SharedMemory(name=shm_name)
            except FileNotFoundError: time.sleep(0.2)
        else: time.sleep(0.5)

    unregister(shm._name, 'shared_memory')
    img_data = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])
    sync_pipe = open(pipe_path, 'r')
    return shm, sync_pipe, img_data

if __name__ == "__main__":
    # 1. Příprava spojené mřížky
    batched_grid = prepare_batched_grid(NPZ_FILE)
    
    # 2. Načtení TensorRT modelu (Nyní stačí POUZE JEDNOU, ale s batch=2)
    print("🧠 Načítám model cara.engine (očekává batch=2)...")
    model = YOLO('cara.engine', task='pose')
    
    # 3. Připojení kamerových rour
    shm_L, pipe_L, img_L = init_ipc('left')
    shm_R, pipe_R, img_R = init_ipc('right')
    
    print("🟢 Připojeno! Jedu asynchronně v režimu jedné masivní dávky.")

    while True:
        # Přečtení synchronizace z obou kamer současně
        line_L = pipe_L.readline()
        line_R = pipe_R.readline()
        
        if not line_L or not line_R: break
        
        frame_seq_L, cap_time_L = line_L.strip().split('|')
        frame_seq_R, cap_time_R = line_R.strip().split('|')
        
        # Zprůměrování času (fotí prakticky identicky)
        capture_time = (float(cap_time_L) + float(cap_time_R)) / 2.0
        
        # Ochrana proti buffer bloatu
        if (time.time() - capture_time) * 1000 > 100:
            continue
            
        raw_L = img_L.copy()
        raw_R = img_R.copy()
        
        # 4. BATCHING A INFERENCE
        # Převedení obou obrázků a jejich spojení (stack) do jednoho tensoru [2, 3, H, W]
        tensor_L = torch.from_numpy(raw_L).permute(2, 0, 1)
        tensor_R = torch.from_numpy(raw_R).permute(2, 0, 1)
        
        # Přesun celého balíku naráz do GPU
        batched_input = torch.stack([tensor_L, tensor_R], dim=0).to('cuda', non_blocking=True).half() / 255.0
        
        # Jediný grid_sample zdeformuje levý i pravý obrázek najednou!
        batched_bev = F.grid_sample(batched_input, batched_grid, mode='bilinear', align_corners=True)
        
        # Inherence nad 2 obrázky najednou
        results = model.predict(batched_bev, verbose=False)
        
        # 5. VYPRACOVÁNÍ VÝSLEDKŮ
        PIXEL_TO_METERS = 4.0 / TARGET_W
        IMAGE_CENTER_X = TARGET_W / 2
        IMAGE_BOTTOM_Y = TARGET_H
        
        if int(frame_seq_L) % 20 == 0:
            print("-" * 40)
            
        # Výsledky přijdou zpět jako pole o 2 prvcích (0=Left, 1=Right)
        for idx, side in enumerate(['left', 'right']):
            r = results[idx]
            pts_count = 0
            
            if r.keypoints is not None and len(r.keypoints) > 0:
                points_px = r.keypoints.xy[0].cpu().numpy()
                valid_pts = [p for p in points_px if not (p[0] == 0 and p[1] == 0)]
                pts_count = len(valid_pts)
                
                if pts_count > 0 and int(frame_seq_L) % 20 == 0:
                    px_x, px_y = valid_pts[0]
                    robot_x = (px_x - IMAGE_CENTER_X) * PIXEL_TO_METERS
                    robot_y = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                    print(f"[{side}] Frame {frame_seq_L} | Bodů: {pts_count} | 📍 {robot_y:.3f}m před, {robot_x:.3f}m do strany")
            
            elif int(frame_seq_L) % 20 == 0:
                print(f"[{side}] Frame {frame_seq_L} | Bodů: 0")

        if int(frame_seq_L) % 20 == 0:
            latency = (time.time() - capture_time) * 1000
            print(f"⚡ Celková Batched Latence: {latency:.2f} ms")