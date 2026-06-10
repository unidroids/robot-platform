import os
import time
import numpy as np
import threading
import cv2
import torch
import torch.nn.functional as F
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
from ultralytics import YOLO
import json

# --- KONFIGURACE ---
W_IN, H_IN = 1232, 1640
TARGET_W, TARGET_H = 640, 480
CHANNELS = 3
HEADER_SIZE = 16
NPZ_FILE = "00_bev_transform.npz"

# Přidáno pro ukládání vizualizací
DEBUG_DIR = "/workspace/vision/debug_output"
os.makedirs(DEBUG_DIR, exist_ok=True)

# Převodní konstanty (1200 px = 4 metry => zmenšeno na 640 px)
PIXEL_TO_METERS = 4.0 / TARGET_W
IMAGE_CENTER_X = TARGET_W / 2
IMAGE_BOTTOM_Y = TARGET_H

print("🚀 Startuji Vision Mikroslužbu (TensorRT Edice)...")

# Načtení zkompilovaného enginu! (Nezapomeň na task='pose')
print("🧠 Načítám model cara.engine...")
model = YOLO('cara.engine', task='pose')

def prepare_cuda_grid(npz_path, side, target_w=TARGET_W, target_h=TARGET_H, device='cuda'):
    """Připraví a ZMENŠÍ BEV mapu pro bleskovou transformaci."""
    print(f"⚙️ Připravuji BEV mapu pro kameru: {side}...")
    npz = np.load(npz_path)
    
    map_x = cv2.resize(npz[f'u_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    map_y = cv2.resize(npz[f'v_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    
    grid_x = (2.0 * map_x / (W_IN - 1)) - 1.0
    grid_y = (2.0 * map_y / (H_IN - 1)) - 1.0
    
    grid = np.stack((grid_x, grid_y), axis=-1)
    return torch.from_numpy(grid).half().unsqueeze(0).to(device)

grid_L = prepare_cuda_grid(NPZ_FILE, 'L')
grid_R = prepare_cuda_grid(NPZ_FILE, 'R')

def vision_worker(side, grid):
    pipe_path = f'/dev/shm/vision_sync_{side}.pipe'
    shm_name = f'vision_shm_{side}'

    print(f"[{side}] Čekám na spuštění nativní služby...")
    while not os.path.exists(pipe_path): time.sleep(0.5)


    print(f"[{side}] Čekám na spuštění nativní služby (Roura + RAM)...")
    
    shm = None
    while shm is None:
        if os.path.exists(pipe_path):
            try:
                # Zkusíme se připojit. Pokud to selže, zachytíme chybu a čekáme dál.
                shm = shared_memory.SharedMemory(name=shm_name)
            except FileNotFoundError:
                # Roura sice existuje ("duch" po pádu), ale RAM ještě ne. Čekáme.
                time.sleep(0.2)
        else:
            time.sleep(0.5)
    unregister(shm._name, 'shared_memory')


    # shm = shared_memory.SharedMemory(name=shm_name)
    # unregister(shm._name, 'shared_memory')

    img_data = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])

    print(f"🟢 [{side}] Připojeno! Jedu na plný výkon.")

    with open(pipe_path, 'r') as sync_pipe:
        while True:
            line = sync_pipe.readline()
            if not line: break
            
            frame_seq, capture_time = line.strip().split('|')
            capture_time = float(capture_time)
            
            # --- OCHRANA PROTI ZPOŽDĚNÍ (100 ms limit) ---
            if (time.time() - capture_time) * 1000 > 100:
                continue
                
            # 1. Rychlé kopírování z RAM
            raw_frame = img_data.copy()
            
            # 2. CUDA TRANSFORMACE (Bilinear)
            # Tensor převádíme na .half() pro TensorRT
            img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).unsqueeze(0).to('cuda', non_blocking=True).half() / 255.0
            bev_640 = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
            
            # 3. YOLO TENSOR-RT INFERENCE
            results = model.predict(bev_640, verbose=False, device=0)
            r = results[0]
            
            # 4. EXTRAKCE BODŮ DO REÁLNÉHO SVĚTA (Metry)
            line_data = {"side": side, "frame": frame_seq, "points": []}
            
            if r.keypoints is not None and len(r.keypoints) > 0:
                # Ošetření: Vezmeme jen body, které mají nenulovou souřadnici (YOLO občas vrací 0,0 pro nejisté body)
                points_px = r.keypoints.xy[0].cpu().numpy()
                
                for px_x, px_y in points_px:
                    if px_x == 0 and px_y == 0: continue
                    
                    robot_x_meters = (px_x - IMAGE_CENTER_X) * PIXEL_TO_METERS
                    robot_y_meters = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                    line_data["points"].append({"x": round(float(robot_x_meters), 3), "y": round(float(robot_y_meters), 3)})
            
            # TADY: Odeslání dat přes UDP. Prozatím jen vypíšeme do konzole (každý 30. snímek)
            latency = (time.time() - capture_time) * 1000

            # --- VIZUÁLNÍ KONTROLA (Uloží každý 2. snímek) ---
            if int(frame_seq) % 2 == 0:
                
                # 1. Extrakce Tensoru z GPU zpět do OpenCV obrázku
                # - squeeze(0): odstraní Batch rozměr
                # - cpu().float(): přesune do RAM a převede z FP16 na klasický float
                # - permute(1, 2, 0): přehodí z (Barva, Y, X) na (Y, X, Barva)
                debug_img = (bev_640.squeeze(0).cpu().float().permute(1, 2, 0).numpy() * 255.0).astype(np.uint8)
                
                # OpenCV občas odmítá kreslit do tensor-polí, ascontiguousarray to srovná v RAM
                debug_img = np.ascontiguousarray(debug_img)

                # 2. Vykreslení nalezených bodů a spojnic
                if r.keypoints is not None and len(r.keypoints) > 0:
                    points_px = r.keypoints.xy[0].cpu().numpy()
                    valid_pts = []
                    
                    # Filtrace platných bodů
                    for px_x, px_y in points_px:
                        if px_x == 0 and px_y == 0: continue
                        valid_pts.append((int(px_x), int(px_y)))
                        
                    # Kreslení žluté páteřní čáry
                    for j in range(1, len(valid_pts)):
                        pt1 = valid_pts[j-1]
                        pt2 = valid_pts[j]
                        cv2.line(debug_img, pt1, pt2, (0, 255, 255), 2)

                    # Kreslení červených bodů a indexů
                    for j, (px, py) in enumerate(valid_pts):
                        cv2.circle(debug_img, (px, py), 4, (0, 0, 255), -1)
                        # Černý stín pod textem
                        cv2.putText(debug_img, str(j), (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
                        # Zelený text
                        cv2.putText(debug_img, str(j), (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                # 3. Přidání OSD (informace o snímku přímo do obrazu)
                #osd_text = f"[{side}] Frame {frame_seq} | Points: {pts_count} | Latency: {latency:.1f}ms"
                #cv2.putText(debug_img, osd_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                #cv2.putText(debug_img, osd_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # 4. Uložení na disk (díky namapování Volume z Dockeru to uvidíš i na Windows!)
                #save_path = os.path.join(DEBUG_DIR, f"bev_{side}_{frame_seq:04d}.jpg")
                save_path = os.path.join(DEBUG_DIR, f"bev_{side}_{int(frame_seq):04d}.jpg")
                
                #zakomentovaný po kompletním testu
                #cv2.imwrite(save_path, debug_img)
                print(f"   📸 Uložen kontrolní snímek: {save_path}")


            if int(frame_seq) % 20 == 0:
                pts_count = len(line_data['points'])
                print(f"[{side}] Frame {frame_seq} | Nalezeno bodů: {pts_count} | Celková latence hw->ai: {latency:.2f} ms")
                if pts_count > 0:
                    closest = line_data['points'][0] # První bod je ten "nejblíž" k robotovi
                    print(f"   📍 Nejbližší bod trasy: {closest['y']}m před, {closest['x']}m do strany")

    shm.close()

if __name__ == "__main__":
    # Nastavení limitu paměti (stále dobré nechat)
    torch.cuda.set_per_process_memory_fraction(0.4)
    
    # t_left = threading.Thread(target=vision_worker, args=('L', grid_L))
    # t_right = threading.Thread(target=vision_worker, args=('R', grid_R))
    t_left = threading.Thread(target=vision_worker, args=('left', grid_L))
    t_right = threading.Thread(target=vision_worker, args=('right', grid_R))
    
    t_left.start()
    t_right.start()

    try:
        while t_left.is_alive() or t_right.is_alive(): time.sleep(1)
    except KeyboardInterrupt:
        print("\n🧯 Ukončuji...")
    finally:
        t_left.join()
        t_right.join()
        print("✅ Hotovo.")