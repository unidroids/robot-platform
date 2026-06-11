from ultralytics import YOLO
import numpy as np
import time

print("1. Načítám model YOLOv11 (přesun na GPU)...")
# Stáhne a načte nejmenší a nejrychlejší model
model = YOLO('yolo11n.pt') 

print("2. Vytvářím testovací obrazový šum (1280x720)...")
# Simulujeme snímek z kamery (náhodné pixely, RGB formát)
dummy_frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)

print("3. Spouštím inferenci pro zahřátí GPU (Warmup)...")
model.predict(source=dummy_frame, show=False, verbose=False)

print("4. Měřím rychlost na 50 snímcích...")
start_time = time.time()
for _ in range(50):
    model.predict(source=dummy_frame, show=False, verbose=False)
end_time = time.time()

fps = 50 / (end_time - start_time)
print(f"--- VÝSLEDEK ---")
print(f"✅ YOLO funguje! Průměrná rychlost: {fps:.1f} FPS")
print(f"Důkaz dokončen.")