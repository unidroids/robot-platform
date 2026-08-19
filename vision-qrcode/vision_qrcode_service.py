import os
from datetime import datetime
import zmq
import threading
import time
import struct
import cv2
import numpy as np
from multiprocessing import shared_memory
from multiprocessing.resource_tracker import unregister
from pyzbar.pyzbar import decode, ZBarSymbol

W_BEV, H_BEV = 2464, 1640
W_IN = 1232
CHANNELS = 3
HEADER_SIZE = 16

class VisionQRCodeService:
    def __init__(self):
        self.is_running = False
        self.thread = None
        self.shutdown_event = threading.Event()
        
        self.u_map_L = None
        self.v_map_L = None
        self.u_map_R = None
        self.v_map_R = None
        
        self.processed_frames = 0
        self.found_qrcodes = 0
        self.last_qrcode = ""
        
        self.log_dir = None
        self.last_log_time = 0.0

    def _init_model(self):
        print("🚀 [Vision-QRCode] Načítání transformačních map...")
        try:
            data = np.load('/opt/projects/robotour/vision/00_bev_transform.npz')
            self.u_map_L = data['u_map_L']
            self.v_map_L = data['v_map_L']
            self.u_map_R = data['u_map_R']
            self.v_map_R = data['v_map_R']
            print("✅ [Vision-QRCode] Transformační mapy načteny.")
        except Exception as e:
            print(f"❌ [Vision-QRCode] Chyba načítání map: {e}")

    def _free_model(self):
        self.u_map_L = None
        self.v_map_L = None
        self.u_map_R = None
        self.v_map_R = None

    def _run_loop(self):
        context = zmq.Context.instance()
        
        sub = context.socket(zmq.SUB)
        sub.connect("ipc:///tmp/robot-camera")
        sub.setsockopt_string(zmq.SUBSCRIBE, "COMBINED")
        
        pub = context.socket(zmq.PUB)
        pub.bind("ipc:///tmp/robot-vision-qrcode")
        
        poller = zmq.Poller()
        poller.register(sub, zmq.POLLIN)
        
        shm = None
        img_data = None
        
        def try_connect_shm():
            try:
                shm_tmp = shared_memory.SharedMemory(name='vision_shm_combined')
                unregister(shm_tmp._name, 'shared_memory')
                img_data_tmp = np.ndarray((H_BEV, W_BEV, CHANNELS), dtype=np.uint8, buffer=shm_tmp.buf[HEADER_SIZE:])
                return shm_tmp, img_data_tmp
            except FileNotFoundError:
                return None, None

        print("🟢 [Vision-QRCode] Vlákno aktivní, čekám na data COMBINED z kamer...")

        try:
            while not self.shutdown_event.is_set():
                socks = dict(poller.poll(500))
                
                if sub not in socks:
                    continue
                    
                parts = sub.recv_multipart()
                if len(parts) != 2:
                    continue
                    
                payload = parts[1].decode('utf-8').split(' ')
                if len(payload) != 2:
                    continue
                
                zmq_frame_seq = int(payload[0])
                
                if shm is None:
                    shm, img_data = try_connect_shm()
                    
                if shm is None:
                    continue
                
                # Lock-free čtení
                retries = 0
                read_success = False
                while retries < 20:
                    seq_before = struct.unpack_from('q', shm.buf, 0)[0]
                    
                    if seq_before == -1 or seq_before < zmq_frame_seq:
                        time.sleep(0.005)
                        retries += 1
                        continue
                        
                    if seq_before > zmq_frame_seq:
                        break
                        
                    raw_frame = img_data.copy()
                    
                    seq_after = struct.unpack_from('q', shm.buf, 0)[0]
                    if seq_after != seq_before:
                        retries += 1
                        continue
                        
                    read_success = True
                    break
                    
                if not read_success:
                    try:
                        shm.close()
                        new_shm, new_img_data = try_connect_shm()
                        if new_shm is not None:
                            shm, img_data = new_shm, new_img_data
                    except Exception:
                        pass
                    continue

                if self.u_map_L is None or self.v_map_L is None:
                    continue
                
                self.processed_frames += 1

                # Rozdělení na levý a pravý obraz
                raw_left = raw_frame[:, :W_IN]
                raw_right = raw_frame[:, W_IN:]

                # --- Transformace obrazu přes OpenCV ---
                transformed_left = cv2.remap(raw_left, self.u_map_L, self.v_map_L, cv2.INTER_LINEAR)
                transformed_right = cv2.remap(raw_right, self.u_map_R, self.v_map_R, cv2.INTER_LINEAR)
                
                # Ukládání snímků (jednou za vteřinu)
                current_time = time.monotonic()
                if current_time - self.last_log_time >= 1.0:
                    if self.log_dir:
                        cv2.imwrite(os.path.join(self.log_dir, f"left_{zmq_frame_seq}.jpg"), transformed_left)
                        cv2.imwrite(os.path.join(self.log_dir, f"right_{zmq_frame_seq}.jpg"), transformed_right)
                    self.last_log_time = current_time
                
                # --- Detekce QR kódu pomocí pyzbar ---
                decoded_left = decode(transformed_left, symbols=[ZBarSymbol.QRCODE])
                decoded_right = decode(transformed_right, symbols=[ZBarSymbol.QRCODE])
                
                found_texts = set()
                
                for obj in decoded_left + decoded_right:
                    qr_text = obj.data.decode("utf-8")
                    if qr_text in found_texts:
                        continue
                    found_texts.add(qr_text)
                    
                    self.found_qrcodes += 1
                    self.last_qrcode = qr_text
                    
                    # Určení prefixu
                    topic_out = "TEXT"
                    if ":" in qr_text:
                        possible_prefix = qr_text.split(":", 1)[0] + ":"
                        if len(possible_prefix) < 20:  # Rozumná délka prefixu
                            topic_out = possible_prefix
                    
                    print(f"✅ [Vision-QRCode] Nalezen QR kód: '{qr_text}' -> publikuji na topic: {topic_out}")
                    pub.send_multipart([topic_out.encode('utf-8'), qr_text.encode('utf-8')])

        except Exception as e:
            print(f"❌ [Vision-QRCode] Chyba smyčky: {e}")
        finally:
            if shm: shm.close()
            pub.close()
            sub.close()
            print("🛑 [Vision-QRCode] Smyčka ukončena.")

    def start(self) -> bool:
        if self.is_running: return False
        self.shutdown_event.clear()
        
        self.processed_frames = 0
        self.found_qrcodes = 0
        self.last_qrcode = ""
        
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H-%M-%S")
        self.log_dir = f"/data/robot/vision-qrcode/{date_str}/{time_str}"
        os.makedirs(self.log_dir, exist_ok=True)
        self.last_log_time = time.monotonic()
        
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
            return f"RUNNING processed:{self.processed_frames} found:{self.found_qrcodes} last:{self.last_qrcode}"
        return "IDLE"

service = VisionQRCodeService()
