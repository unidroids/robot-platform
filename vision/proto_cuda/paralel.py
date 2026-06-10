import os
import time
import cv2
import numpy as np
import threading
import torch
import torch.nn.functional as F
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister

# --- KONFIGURACE ---
W_IN, H_IN = 1232, 1640  # Už otočené rozměry z kamer!
TARGET_W, TARGET_H = 640, 480
CHANNELS = 3
HEADER_SIZE = 16
NPZ_FILE = "00_bev_transform.npz"

print("🚀 Startuji Vision Mikroslužbu (Paralelní CUDA Streams)...")

def prepare_cuda_grid(npz_path, side, target_w=TARGET_W, target_h=TARGET_H, device='cuda'):
    """Připraví a zmenší BEV mapu pro bleskovou transformaci."""
    print(f"⚙️ Připravuji BEV mapu pro kameru: {side}...")
    npz = np.load(npz_path)
    
    map_x = cv2.resize(npz[f'u_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    map_y = cv2.resize(npz[f'v_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    
    grid_x = (2.0 * map_x / (W_IN - 1)) - 1.0
    grid_y = (2.0 * map_y / (H_IN - 1)) - 1.0
    
    grid = np.stack((grid_x, grid_y), axis=-1)
    return torch.from_numpy(grid).half().unsqueeze(0).to(device)

# Načtení transformačních mřížek
grid_L = prepare_cuda_grid(NPZ_FILE, 'L')
grid_R = prepare_cuda_grid(NPZ_FILE, 'R')

def vision_worker(side, grid):
    from ultralytics import YOLO  # Import uvnitř vlákna
    
    # 1. KAŽDÉ VLÁKNO DOSTANE SVOJE VLASTNÍ GPU KANÁLY
    stream = torch.cuda.Stream(device=0)
    
    print(f"🧠 [{side}] Načítám nezávislý model pro toto vlákno...")
    local_model = YOLO('cara.engine', task='pose')
    
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    shm_name = f'vision_shm_{side}'

    print(f"[{side}] Čekám na spuštění nativní služby (Roura + RAM)...")
    shm = None
    while shm is None:
        if os.path.exists(pipe_path):
            try:
                shm = shared_memory.SharedMemory(name=shm_name)
            except FileNotFoundError:
                time.sleep(0.2)
        else:
            time.sleep(0.5)

    unregister(shm._name, 'shared_memory')
    img_data = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])

    print(f"🟢 [{side}] Připojeno! Jedu asynchronně na vlastním CUDA Streamu.")

    with open(pipe_path, 'r') as sync_pipe:
        while True:
            line = sync_pipe.readline()
            if not line: break
            
            frame_seq, capture_time = line.strip().split('|')
            capture_time = float(capture_time)
            
            if (time.time() - capture_time) * 1000 > 100:
                continue
                
            raw_frame = img_data.copy()
            
            # 2. AKTIVACE PARALELNÍHO KANÁLU V GPU
            with torch.cuda.stream(stream):
                # non_blocking=True povolí přenos do GPU na pozadí, zatímco druhé vlákno už počítá
                img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).unsqueeze(0).to('cuda', non_blocking=True).half() / 255.0
                bev_640 = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
                
                # Inference se spustí dedikovaně v tomto streamu
                results = local_model.predict(bev_640, verbose=False, device=0)
                
                # Počkáme na dokončení výpočtu POUZE tohoto konkrétního streamu
                stream.synchronize()
            
            r = results[0]
            
            # Přepočet na metry (CPU)
            PIXEL_TO_METERS = 4.0 / TARGET_W
            IMAGE_CENTER_X = TARGET_W / 2
            IMAGE_BOTTOM_Y = TARGET_H
            
            line_data = {"side": side, "frame": frame_seq, "points": []}
            if r.keypoints is not None and len(r.keypoints) > 0:
                points_px = r.keypoints.xy[0].cpu().numpy()
                for px_x, px_y in points_px:
                    if px_x == 0 and px_y == 0: continue
                    robot_x_meters = (px_x - IMAGE_CENTER_X) * PIXEL_TO_METERS
                    robot_y_meters = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                    line_data["points"].append({"x": round(float(robot_x_meters), 3), "y": round(float(robot_y_meters), 3)})
            
            # Statistiky každých 20 snímků
            latency = (time.time() - capture_time) * 1000
            if int(frame_seq) % 20 == 0:
                pts_count = len(line_data['points'])
                print(f"[{side}] Frame {frame_seq} | Nalezeno bodů: {pts_count} | Celková latence: {latency:.2f} ms")
                if pts_count > 0:
                    closest = line_data['points'][0]
                    print(f"   📍 Nejbližší bod trasy: {closest['y']}m před, {closest['x']}m do strany")

    shm.close()

if __name__ == "__main__":
    torch.cuda.set_per_process_memory_fraction(0.5)
    
    t_left = threading.Thread(target=vision_worker, args=('left', grid_L))
    t_right = threading.Thread(target=vision_worker, args=('right', grid_R))
    
    t_left.start()
    t_right.start()

    try:
        while t_left.is_alive() or t_right.is_alive(): time.sleep(1)
    except KeyboardInterrupt:
        print("\n🧯 Ukončuji...")