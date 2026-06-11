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
import threading
import gc

# --- KONFIGURACE ---
W_IN, H_IN = 1232, 1640
TARGET_W, TARGET_H = 640, 480
CHANNELS = 3
HEADER_SIZE = 16
NPZ_FILE = os.path.join(os.path.dirname(__file__), "00_bev_transform.npz")
ENGINE_FILE = os.path.join(os.path.dirname(__file__), "cara-single.engine")

PIXEL_TO_METERS = 4.0 / TARGET_W
IMAGE_CENTER_X = TARGET_W / 2
IMAGE_BOTTOM_Y = TARGET_H

def prepare_cuda_grid(npz_path, side, target_w=TARGET_W, target_h=TARGET_H, device='cuda'):
    """Připraví a ZMENŠÍ BEV mapu pro bleskovou transformaci."""
    npz = np.load(npz_path)
    
    map_x = cv2.resize(npz[f'u_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    map_y = cv2.resize(npz[f'v_map_{side}'], (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    
    grid_x = (2.0 * map_x / (W_IN - 1)) - 1.0
    grid_y = (2.0 * map_y / (H_IN - 1)) - 1.0
    
    grid = np.stack((grid_x, grid_y), axis=-1)
    return torch.from_numpy(grid).half().unsqueeze(0).to(device)


class VisionService:
    def __init__(self):
        self.is_running = False
        self.thread = None
        self.shutdown_event = threading.Event()
        
        self.model = None
        self.grid_L = None
        self.grid_R = None
        
        self.last_seq_left = -1
        self.last_seq_right = -1

    def _init_model(self):
        if self.model is None:
            print("🧠 [Vision] Načítám model a enginy do GPU...")
            torch.cuda.set_per_process_memory_fraction(0.4)
            self.model = YOLO(ENGINE_FILE, task='pose')
            self.grid_L = prepare_cuda_grid(NPZ_FILE, 'L')
            self.grid_R = prepare_cuda_grid(NPZ_FILE, 'R')
            print("✅ [Vision] Model inicializován.")

    def _free_model(self):
        if self.model is not None:
            print("🧹 [Vision] Uvolňuji model a čistím GPU paměť...")
            del self.model
            del self.grid_L
            del self.grid_R
            self.model = None
            self.grid_L = None
            self.grid_R = None
            gc.collect()
            torch.cuda.empty_cache()
            print("✅ [Vision] Paměť uvolněna.")

    def _run_loop(self):
        print("🚀 [Vision] Startuji hlavní smyčku...")
        
        context = zmq.Context.instance()
        
        # ZMQ SUB
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.CONFLATE, 1)
        sub.connect("ipc:///tmp/robot-camera")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # Poller pro neblokující čtení
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        
        # ZMQ PUB
        pub = context.socket(zmq.PUB)
        pub.bind("ipc:///tmp/robot-vision")
        
        shm_left = None
        shm_right = None
        img_data_left = None
        img_data_right = None
        
        def try_connect_shm(side):
            try:
                shm = shared_memory.SharedMemory(name=f'vision_shm_{side}')
                unregister(shm._name, 'shared_memory')
                img_data = np.ndarray((H_IN, W_IN, CHANNELS), dtype=np.uint8, buffer=shm.buf[HEADER_SIZE:])
                return shm, img_data
            except FileNotFoundError:
                return None, None

        print("🟢 [Vision] Vlákno aktivní, čekám na data z kamer...")

        try:
            while not self.shutdown_event.is_set():
                # Timeout 500ms, abychom mohli kontrolovat shutdown_event
                socks = dict(poller.poll(500))
                
                if sub not in socks:
                    continue # Žádná zpráva, jedeme znovu
                    
                msg = sub.recv_string()
                parts = msg.split('/')
                if len(parts) != 3: continue
                
                side = parts[0]
                zmq_frame_seq = int(parts[1])
                capture_time = float(parts[2])
                
                # Udržujeme čísla framů pro případný STATUS
                if side == 'left':
                    self.last_seq_left = zmq_frame_seq
                else:
                    self.last_seq_right = zmq_frame_seq
                
                # Zpožděné připojení k SHM
                if side == 'left' and shm_left is None:
                    shm_left, img_data_left = try_connect_shm('left')
                if side == 'right' and shm_right is None:
                    shm_right, img_data_right = try_connect_shm('right')
                    
                if side == 'left' and shm_left is None: continue
                if side == 'right' and shm_right is None: continue
                
                # Výběr referencí
                shm = shm_left if side == 'left' else shm_right
                img_data = img_data_left if side == 'left' else img_data_right
                grid = self.grid_L if side == 'left' else self.grid_R
                
                # Lock-free čtení
                retries = 0
                read_success = False
                while retries < 20:
                    seq_before = struct.unpack_from('q d', shm.buf, 0)[0]
                    if seq_before == -1 or seq_before != zmq_frame_seq:
                        time.sleep(0.005)
                        retries += 1
                        continue
                        
                    raw_frame = img_data.copy()
                    
                    seq_after = struct.unpack_from('q d', shm.buf, 0)[0]
                    if seq_after != seq_before:
                        retries += 1
                        continue
                        
                    read_success = True
                    break
                    
                if not read_success:
                    print(f"[{side}] Kolize při čtení SHM, zkouším RECONNECT...")
                    try:
                        shm.close()
                        new_shm, new_img_data = try_connect_shm(side)
                        if new_shm is not None:
                            if side == 'left':
                                shm_left, img_data_left = new_shm, new_img_data
                            else:
                                shm_right, img_data_right = new_shm, new_img_data
                            print(f"[{side}] Úspěšný SHM reconnect k nové paměti!")
                    except Exception as e:
                        print(f"[{side}] Reconnect selhal: {e}")
                    continue

                # --- AI Inference ---
                img_tensor = torch.from_numpy(raw_frame).permute(2, 0, 1).unsqueeze(0).to('cuda', non_blocking=True).half() / 255.0
                bev_640 = F.grid_sample(img_tensor, grid, mode='bilinear', align_corners=True)
                
                results = self.model.predict(bev_640, verbose=False, device=0)
                r = results[0]
                
                out_points = []
                if r.keypoints is not None and len(r.keypoints) > 0:
                    points_px = r.keypoints.xy[0].cpu().numpy()
                    for px_x, px_y in points_px:
                        if px_x == 0 and px_y == 0: continue
                        robot_x_meters = (IMAGE_BOTTOM_Y - px_y) * PIXEL_TO_METERS
                        robot_y_meters = (IMAGE_CENTER_X - px_x) * PIXEL_TO_METERS
                        out_points.append({"x": round(float(robot_x_meters), 3), "y": round(float(robot_y_meters), 3)})
                
                # --- Odeslání ---
                msg_data = {
                    "time": capture_time,
                    "side": side,
                    "frame": zmq_frame_seq,
                    "pose": out_points
                }
                pub.send_string(f"vision/{json.dumps(msg_data)}")

        except Exception as e:
            print(f"❌ [Vision] Chyba smyčky: {e}")
        finally:
            if shm_left: shm_left.close()
            if shm_right: shm_right.close()
            pub.close()
            sub.close()
            print("🛑 [Vision] Smyčka ukončena.")
            
    def start(self) -> bool:
        if self.is_running: return False
        
        self.shutdown_event.clear()
        self._init_model()
        
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        self.is_running = True
        return True

    def stop(self) -> bool:
        if not self.is_running: return False
        
        self.shutdown_event.set()
        if self.thread:
            self.thread.join(timeout=3.0)
            
        self._free_model()
        self.is_running = False
        return True

    def get_status(self) -> str:
        if self.is_running:
            return f"RUNNING L:{self.last_seq_left} R:{self.last_seq_right}"
        return "IDLE"

service = VisionService()
