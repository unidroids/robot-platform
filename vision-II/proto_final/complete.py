import os
import time
import numpy as np
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
# Výpočet měřítka: 3m odpovídá 480px a 4m odpovídá 640px. Měřítko je 1/160 m/px.
PIXEL_TO_METERS = 4.0 / TARGET_W
IMAGE_CENTER_X = TARGET_W / 2
IMAGE_BOTTOM_Y = TARGET_H

print("🚀 Startuji Vision Mikroslužbu (TensorRT Edice)...")

# Nastavení limitu paměti (stále dobré nechat)
torch.cuda.set_per_process_memory_fraction(0.4)

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

def main():
    print("Čekám na spuštění nativní služby (ZMQ + RAM)...")
    
    # 1. Připojení k ZMQ
    context = zmq.Context.instance()
    
    # SUB (Příjem událostí z kamer)
    sub = context.socket(zmq.SUB)
    sub.setsockopt(zmq.CONFLATE, 1)  # Zachovat jen nejnovější zprávu
    sub.connect("ipc:///tmp/robot-camera")
    sub.setsockopt_string(zmq.SUBSCRIBE, "") # Odebíráme vše (left i right)
    
    # PUB (Odesílání hotového pose)
    pub = context.socket(zmq.PUB)
    pub.bind("ipc:///tmp/robot-vision")
    
    # 2. Připojení k SHM pro obě kamery
    shm_left = None
    shm_right = None
    
    while shm_left is None or shm_right is None:
        try:
            if shm_left is None:
                shm_left = shared_memory.SharedMemory(name='vision_shm_left')
            if shm_right is None:
                shm_right = shared_memory.SharedMemory(name='vision_shm_right')
        except FileNotFoundError:
            time.sleep(0.2)
            
    unregister(shm_left._name, 'shared_memory')
    unregister(shm_right._name, 'shared_memory')

    img_data_left = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm_left.buf[HEADER_SIZE:])
    img_data_right = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm_right.buf[HEADER_SIZE:])

    print("🟢 Připojeno! Jednovláknová hlavní smyčka jede na plný výkon.")

    try:
        while True:
            # Přečteme synchronizační událost ze ZMQ
            msg = sub.recv_string()
            parts = msg.split('/')
            if len(parts) != 3: continue
            
            side = parts[0]
            zmq_frame_seq = int(parts[1])
            zmq_capture_time = float(parts[2]) # TOTO JE NYNÍ ABSOLUTNÍ MONOTONIC TIME
            
            # Nastavíme ukazatele podle toho, o kterou kameru jde
            if side == 'left':
                shm = shm_left
                img_data = img_data_left
                grid = grid_L
            else:
                shm = shm_right
                img_data = img_data_right
                grid = grid_R
                
            # --- START DETAILNÍHO PROFILOVÁNÍ ---
            t_start = time.time()
            
            # 1. Ochrana proti přepsání během čtení (lock-free)
            retries = 0
            read_success = False
            while retries < 20:
                # Přečíst hlavičku PŘED kopírováním
                header_before = struct.unpack_from('q d', shm.buf, 0)
                seq_before = header_before[0]
                
                # Pokud se zapisuje (-1) nebo ZMQ zpráva nesouhlasí se sdílenou pamětí
                if seq_before == -1 or seq_before != zmq_frame_seq:
                    time.sleep(0.005)
                    retries += 1
                    continue
                
                # Rychlé kopírování z RAM
                raw_frame = img_data.copy()
                
                # Přečíst hlavičku PO kopírování
                header_after = struct.unpack_from('q d', shm.buf, 0)
                seq_after = header_after[0]
                
                # Pokud se hlavička změnila BĚHEM kopírování, obraz je zkorumpovaný
                if seq_after != seq_before:
                    retries += 1
                    continue
                    
                read_success = True
                break
                
            if not read_success:
                print(f"[{side}] VAROVÁNÍ: Kolize při čtení SHM, zahazuji snímek {zmq_frame_seq}. Zkouším RECONNECT...")
                # Pokud se capture.py restartovalo, může být nutné se znovu připojit na nový blok paměti
                try:
                    shm.close()
                    new_shm = shared_memory.SharedMemory(name=f'vision_shm_{side}')
                    unregister(new_shm._name, 'shared_memory')
                    if side == 'left':
                        shm_left = new_shm
                        img_data_left = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm_left.buf[HEADER_SIZE:])
                    else:
                        shm_right = new_shm
                        img_data_right = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm_right.buf[HEADER_SIZE:])
                    print(f"[{side}] Úspěšný SHM reconnect k nové paměti!")
                except Exception as e:
                    print(f"[{side}] SHM reconnect selhal: {e}")
                continue

            capture_time = zmq_capture_time
            frame_seq = zmq_frame_seq
            
            # --- START DETAILNÍHO PROFILOVÁNÍ ---
            t_start = time.monotonic()
            t_cpu_copy = (time.monotonic() - t_start) * 1000 # Téměř nula, ale necháme
                
            # 2. PŘENOS RAM -> GPU (Včetně permute a normalizace)
            t_gpu_start = time.monotonic()
            img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).unsqueeze(0).to('cuda', non_blocking=True).half() / 255.0
            torch.cuda.synchronize()
            t_gpu_transfer = (time.monotonic() - t_gpu_start) * 1000
            
            # 3. BEV TRANSFORMACE (Grid Sample v CUDA)
            t_bev_start = time.monotonic()
            bev_640 = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
            torch.cuda.synchronize()
            t_bev = (time.monotonic() - t_bev_start) * 1000
            
            # 4. YOLO TENSOR-RT INFERENCE
            t_ai_start = time.monotonic()
            results = model.predict(bev_640, verbose=False, device=0)
            torch.cuda.synchronize()
            t_ai = (time.monotonic() - t_ai_start) * 1000
            
            # 5. EXTRAKCE BODŮ DO REÁLNÉHO SVĚTA
            t_post_start = time.monotonic()
            r = results[0]
            
            out_points = []
            
            if r.keypoints is not None and len(r.keypoints) > 0:
                points_px = r.keypoints.xy[0].cpu().numpy()
                for px_x, px_y in points_px:
                    if px_x == 0 and px_y == 0: continue
                    # X: dopředu (spodní hrana obrazu je X=0, nahoru se X zvětšuje)
                    robot_x_meters = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                    # Y: doleva (levá polovina obrazu je kladné Y, pravá je záporné)
                    robot_y_meters = (IMAGE_CENTER_X - px_x) * PIXEL_TO_METERS
                    
                    out_points.append({"x": round(float(robot_x_meters), 3), "y": round(float(robot_y_meters), 3)})
            
            t_post = (time.monotonic() - t_post_start) * 1000
            process_latency = t_cpu_copy + t_gpu_transfer + t_bev + t_ai + t_post
            end_to_end_latency = (time.monotonic() - capture_time) * 1000

            # --- ODESLÁNÍ VÝSLEDKŮ ---
            msg_data = {
                "time": capture_time,
                "side": side,
                "frame": frame_seq,
                "pose": out_points
            }
            pub.send_string(f"vision/{json.dumps(msg_data)}")

            # --- VÝPIS STATISTIK A LATENCE (Každý 20. snímek) ---
            if int(frame_seq) % 20 == 0:
                pts_count = len(out_points)
                print(f"\n[{side}] Frame {frame_seq} | Nalezeno bodů: {pts_count}")
                
                if pts_count > 0:
                    closest = out_points[0]
                    print(f"   📍 Nejbližší bod trasy: X={closest['x']}m vpřed, Y={closest['y']}m doleva")
                
                print(f"📊 [ROZPAD LATENCE] Capture time: {capture_time}, Capture frame seq: {frame_seq}")
                print(f"  └─ 1. CPU kopie z SHM:        {t_cpu_copy:.2f} ms")
                print(f"  └─ 2. Přenos RAM -> GPU:      {t_gpu_transfer:.2f} ms")
                print(f"  └─ 3. BEV Transformace (CUDA):{t_bev:.2f} ms")
                print(f"  └─ 4. TensorRT (YOLO):        {t_ai:.2f} ms")
                print(f"  └─ 5. Post-processing (CPU):  {t_post:.2f} ms")
                print(f"⏱️ Čas samotného zpracování:    {process_latency:.2f} ms")
                print(f"⚡ Celková latence (kamera->ai): {end_to_end_latency:.2f} ms")

            # --- VIZUÁLNÍ KONTROLA (vypnuto) ---
            if int(frame_seq) % 2 == 0 and False:
                debug_img = (bev_640.squeeze(0).cpu().float().permute(1, 2, 0).numpy() * 255.0).astype(np.uint8)
                debug_img = np.ascontiguousarray(debug_img)
                if r.keypoints is not None and len(r.keypoints) > 0:
                    points_px = r.keypoints.xy[0].cpu().numpy()
                    valid_pts = []
                    for px_x, px_y in points_px:
                        if px_x == 0 and px_y == 0: continue
                        valid_pts.append((int(px_x), int(px_y)))
                    for j in range(1, len(valid_pts)):
                        pt1 = valid_pts[j-1]
                        pt2 = valid_pts[j]
                        cv2.line(debug_img, pt1, pt2, (0, 255, 255), 2)
                    for j, (px, py) in enumerate(valid_pts):
                        cv2.circle(debug_img, (px, py), 4, (0, 0, 255), -1)
                        cv2.putText(debug_img, str(j), (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
                        cv2.putText(debug_img, str(j), (px + 5, py - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                save_path = os.path.join(DEBUG_DIR, f"bev_{side}_{int(frame_seq):04d}.jpg")
                print(f"   📸 Uložen kontrolní snímek: {save_path}")

    except KeyboardInterrupt:
        print("\n🧯 Ukončuji...")
    finally:
        if shm_left: shm_left.close()
        if shm_right: shm_right.close()
        print("✅ Hotovo.")

if __name__ == "__main__":
    main()