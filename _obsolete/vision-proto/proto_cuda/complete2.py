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
W_EACH, H_EACH = 1232, 1640
W_COMB, H_COMB = W_EACH * 2, H_EACH  # 2464 x 1640
TARGET_W, TARGET_H = 640, 480       # model cara-batch.engine
CHANNELS = 3
HEADER_SIZE = 16
LOG_INTERVAL = 20  # Vypisovat log každých X snímků

print("🚀 Startuji Konsolidovanou Vision Mikroslužbu (Batch=2, Single-Thread)...")

# Příprava spojené mřížky pro BEV [2, 640, 640, 2]
npz = np.load("00_bev_transform.npz")
grids = []
for side in ['L', 'R']:
    map_x = cv2.resize(npz[f'u_map_{side}'], (TARGET_W, TARGET_H), interpolation=cv2.INTER_LINEAR)
    map_y = cv2.resize(npz[f'v_map_{side}'], (TARGET_W, TARGET_H), interpolation=cv2.INTER_LINEAR)
    grid_x = (2.0 * map_x / (W_EACH - 1)) - 1.0
    grid_y = (2.0 * map_y / (H_EACH - 1)) - 1.0
    grids.append(torch.from_numpy(np.stack((grid_x, grid_y), axis=-1)).half().to('cuda'))
batched_grid = torch.stack(grids, dim=0)

# Inicializace modelu
model = YOLO('cara-batch.engine', task='pose')

# Připojení ke sdílenému stereo vstupu
PIPE_PATH = "/dev/shm/vision_sync_stereo.pipe"
SHM_NAME = "vision_shm_stereo"

print("⏳ Čekám na spuštění hardwarového stereo ovladače...")
shm = None
while shm is None:
    if os.path.exists(PIPE_PATH):
        try: shm = shared_memory.SharedMemory(name=SHM_NAME)
        except FileNotFoundError: time.sleep(0.2)
    else: time.sleep(0.5)

unregister(shm._name, 'shared_memory')
img_data = np.ndarray((H_COMB, W_COMB, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])

print("🟢 Spojení navázáno! Zpracovávám konsolidovaný hardwarový stream.")

with open(PIPE_PATH, 'r') as sync_pipe:
    while True:
        line = sync_pipe.readline()
        if not line: break
        
        frame_seq, cap_time = line.strip().split('|')
        capture_time = float(cap_time)
        
        if (time.time() - capture_time) * 1000 > 100:
            continue  # Ochrana proti buffer bloatu
            
        # --- START DETAILNÍHO PROFILOVÁNÍ ---
        t_start = time.time()
        
        # 1. CPU kopie z SHM paměti (režie RAM -> RAM)
        raw_stereo = img_data.copy()
        t_cpu_copy = (time.time() - t_start) * 1000
        
        # 2. Přenos do GPU a konverze na FP16 (Host-to-Device transfer)
        t_gpu_trans_start = time.time()
        tensor_stereo = torch.from_numpy(raw_stereo).permute(2, 0, 1).to('cuda', non_blocking=True).half() / 255.0
        torch.cuda.synchronize()  # Počkáme na dokončení přenosu do VRAM
        t_gpu_transfer = (time.time() - t_gpu_trans_start) * 1000
        
        # 3. GPU Rozseknutí a sestavení dávky (Batch=2)
        t_batch_start = time.time()
        left_eye = tensor_stereo[:, :, :W_EACH]
        right_eye = tensor_stereo[:, :, W_EACH:]
        batched_input = torch.stack([left_eye, right_eye], dim=0)
        torch.cuda.synchronize()  # Slicing a stacking v GPU paměti
        t_batch_build = (time.time() - t_batch_start) * 1000
        
        # 4. Společná BEV transformace (F.grid_sample) + TensorRT Inference
        t_ai_start = time.time()
        batched_bev = F.grid_sample(batched_input, batched_grid, mode='bilinear', align_corners=True)
        results = model.predict(batched_bev, verbose=False)
        torch.cuda.synchronize()  # Klíčové: Počkáme, až TensorRT engine dokončí výpočet sítě
        t_ai_inference = (time.time() - t_ai_start) * 1000
        
        # 5. Vyhodnocení metrických souřadnic na CPU
        t_post_start = time.time()
        PIXEL_TO_METERS = 4.0 / TARGET_W
        IMAGE_CENTER_X = TARGET_W / 2
        IMAGE_BOTTOM_Y = TARGET_H
        
        show_log = (int(frame_seq) % LOG_INTERVAL == 0)
        if show_log:
            print(f"\n--- 🛰️ Telemetrie Frame {frame_seq} ---")
            
        for idx, side in enumerate(['Levá', 'Pravá']):
            r = results[idx]
            pts_count = 0
            if r.keypoints is not None and len(r.keypoints) > 0:
                points_px = r.keypoints.xy[0].cpu().numpy()
                valid_pts = [p for p in points_px if not (p[0] == 0 and p[1] == 0)]
                pts_count = len(valid_pts)
                
                if pts_count > 0 and show_log:
                    px_x, px_y = valid_pts[0]
                    robot_x = (px_x - IMAGE_CENTER_X) * PIXEL_TO_METERS
                    robot_y = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                    print(f" 👁️ [{side} Kamera] Bodů: {pts_count} | Cíl: {robot_y:.3f}m před, {robot_x:.3f}m do strany")
            elif show_log:
                print(f" 👁️ [{side} Kamera] Žádná čára nebyla nalezena.")
                
        t_post_process = (time.time() - t_post_start) * 1000
        
        # Výpis rozpadu latence do konzole
        if show_log:
            total_latency = (time.time() - capture_time) * 1000
            print(f"📊 [ROZPAD LATENCE]")
            print(f"  └─ 1. CPU kopie z SHM:        {t_cpu_copy:.2f} ms")
            print(f"  └─ 2. Přenos RAM -> GPU:      {t_gpu_transfer:.2f} ms")
            print(f"  └─ 3. GPU Výřez & Batching:   {t_batch_build:.2f} ms")
            print(f"  └─ 4. BEV + TensorRT (YOLO):  {t_ai_inference:.2f} ms")
            print(f"  └─ 5. Post-processing (CPU):  {t_post_process:.2f} ms")
            print(f"⚡ Celková latence od záblesku (HW->AI): {total_latency:.2f} ms")

shm.close()