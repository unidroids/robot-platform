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
import zmq
import struct

# --- KONFIGURACE ---
W_IN, H_IN = 1232, 1640
TARGET_W, TARGET_H = 640, 480
CHANNELS = 3
HEADER_SIZE = 16
NPZ_FILE = "00_bev_transform.npz"

# Přidáno pro ukládání vizualizací
DEBUG_DIR = "/data/robot/vision"
os.makedirs(DEBUG_DIR, exist_ok=True)

# Převodní konstanty (1200 px = 4 metry => zmenšeno na 640 px)
PIXEL_TO_METERS = 4.0 / TARGET_W
IMAGE_CENTER_X = TARGET_W / 2
IMAGE_BOTTOM_Y = TARGET_H

print("🚀 Startuji Vision Mikroslužbu (TensorRT Edice)...")

# Načtení zkompilovaného enginu! (Nezapomeň na task='pose')
print("🧠 Načítám model cara.engine...")
model = YOLO('cara-single.engine', task='pose')

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
    shm_name = f'vision_shm_{side}'

    print(f"[{side}] Čekám na spuštění nativní služby (ZMQ + RAM)...")
    
    # 1. Připojení k ZMQ
    context = zmq.Context.instance()
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.CONFLATE, 1)  # Zachovat jen nejnovější zprávu (zahodit staré, pokud nestíháme)
    sub.connect("ipc:///tmp/vision_sync")
    sub.setsockopt_string(zmq.SUBSCRIBE, f"{side}/")
    
    # 2. Připojení k SHM
    shm = None
    while shm is None:
        try:
            shm = shared_memory.SharedMemory(name=shm_name)
        except FileNotFoundError:
            time.sleep(0.2)
    unregister(shm._name, 'shared_memory')

    img_data = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])

    print(f"🟢 [{side}] Připojeno! Jedu na plný výkon.")

    while True:
        # Přečteme synchronizační událost ze ZMQ
        msg = sub.recv_string()
        parts = msg.split('/')
        if len(parts) != 3: continue
        
        _, zmq_frame_seq, zmq_capture_time = parts
        zmq_frame_seq = int(zmq_frame_seq)
        zmq_capture_time = float(zmq_capture_time)
        
        # --- OCHRANA PROTI ZPOŽDĚNÍ ---
        # Již není potřeba měřit vůči time.time() (capture_time je PTS od kamery a ne epoch time).
        # Navíc zmq.CONFLATE už automaticky zahazuje staré zprávy ve frontě.
        
        # --- START DETAILNÍHO PROFILOVÁNÍ ---
        t_start = time.time()
        
        # 1. Ochrana proti přepsání během čtení (lock-free)
        retries = 0
        read_success = False
        while retries < 5:
            # Přečíst hlavičku PŘED kopírováním
            header_before = struct.unpack_from('q d', shm.buf, 0)
            seq_before = header_before[0]
            
            # Pokud se zapisuje (-1) nebo ZMQ zpráva nesouhlasí se sdílenou pamětí
            if seq_before == -1 or seq_before != zmq_frame_seq:
                time.sleep(0.001)
                retries += 1
                continue
            
            # Rychlé kopírování z RAM
            raw_frame = img_data.copy()
            
            # Přečíst hlavičku PO kopírování
            header_after = struct.unpack_from('q d', shm.buf, 0)
            seq_after = header_after[0]
            
            # Pokud se hlavička změnila BĚHEM kopírování, obraz je zkorumpovaný (smíchané dva snímky)
            if seq_after != seq_before:
                retries += 1
                continue
                
            read_success = True
            break
            
        if not read_success:
            print(f"[{side}] VAROVÁNÍ: Kolize při čtení SHM, zahazuji snímek {zmq_frame_seq}")
            continue

        capture_time = zmq_capture_time
        frame_seq = zmq_frame_seq
        t_cpu_copy = (time.time() - t_start) * 1000
            
        # 2. PŘENOS RAM -> GPU (Včetně permute a normalizace)
        t_gpu_start = time.time()
        img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).unsqueeze(0).to('cuda', non_blocking=True).half() / 255.0
        torch.cuda.synchronize()
        t_gpu_transfer = (time.time() - t_gpu_start) * 1000
        
        # 3. BEV TRANSFORMACE (Grid Sample v CUDA)
        t_bev_start = time.time()
        bev_640 = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
        torch.cuda.synchronize()
        t_bev = (time.time() - t_bev_start) * 1000
        
        # 4. YOLO TENSOR-RT INFERENCE
        t_ai_start = time.time()
        results = model.predict(bev_640, verbose=False, device=0)
        torch.cuda.synchronize()
        t_ai = (time.time() - t_ai_start) * 1000
        
        # 5. EXTRAKCE BODŮ DO REÁLNÉHO SVĚTA (Post-processing na CPU)
        t_post_start = time.time()
        r = results[0]
        line_data = {"side": side, "frame": frame_seq, "points": []}
        
        if r.keypoints is not None and len(r.keypoints) > 0:
            points_px = r.keypoints.xy[0].cpu().numpy()
            for px_x, px_y in points_px:
                if px_x == 0 and px_y == 0: continue
                robot_x_meters = (px_x - IMAGE_CENTER_X) * PIXEL_TO_METERS
                robot_y_meters = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                line_data["points"].append({"x": round(float(robot_x_meters), 3), "y": round(float(robot_y_meters), 3)})
        
        t_post = (time.time() - t_post_start) * 1000
        total_latency = (time.time() - capture_time) * 1000

        # --- VÝPIS STATISTIK A LATENCE (Každý 20. snímek) ---
        if int(frame_seq) % 20 == 0:
            pts_count = len(line_data['points'])
            print(f"\n[{side}] Frame {frame_seq} | Nalezeno bodů: {pts_count}")
            
            if pts_count > 0:
                closest = line_data['points'][0]
                print(f"   📍 Nejbližší bod trasy: {closest['y']}m před, {closest['x']}m do strany")
            
            print(f"📊 [ROZPAD LATENCE] Capture time: {capture_time}, Capture frame seq: {frame_seq}")
            print(f"  └─ 1. CPU kopie z SHM:        {t_cpu_copy:.2f} ms")
            print(f"  └─ 2. Přenos RAM -> GPU:      {t_gpu_transfer:.2f} ms")
            print(f"  └─ 3. BEV Transformace (CUDA):{t_bev:.2f} ms")
            print(f"  └─ 4. TensorRT (YOLO):        {t_ai:.2f} ms")
            print(f"  └─ 5. Post-processing (CPU):  {t_post:.2f} ms")
            print(f"⚡ Celková latence (hw->ai):     {total_latency:.2f} ms")

        # --- VIZUÁLNÍ KONTROLA (Uloží každý 2. snímek) --- vypnuto
        if int(frame_seq) % 2 == 0 and False:
            
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