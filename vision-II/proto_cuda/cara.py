import time
import torch
from ultralytics import YOLO

if __name__ == "__main__":
    print("🚀 Startuji čistý benchmark YOLOv8 Pose...")
    
    print("🧠 Načítám model best.pt...")
    model = YOLO('cara.engine', task='pose')
    #model.task = "pose"
    #model.fuse() # Optimalizace sítě
    
    # Vytvoření cvičného obrazu ROVNOU v paměti GPU (šum)
    # Tvar: (1 obrázek, 3 barvy, výška 480, šířka 640)
    dummy_input = torch.rand(1, 3, 480, 640, device='cuda')

    print("\n🔥 KROK 1: Zahřívání modelu (Warm-up)")
    for _ in range(10):
        _ = model.predict(dummy_input, verbose=False)
    torch.cuda.synchronize()
    print("   ✅ Model je zahřátý.")

    print("\n🔥 KROK 2: Ostrý Benchmark (100 průchodů)")
    iterations = 100
    
    torch.cuda.synchronize()
    t_start = time.perf_counter()
    
    for _ in range(iterations):
        _ = model.predict(dummy_input, verbose=False)
        
    torch.cuda.synchronize()
    t_end = time.perf_counter()
    
    total_time = (t_end - t_start) * 1000
    avg_time = total_time / iterations
    fps = 1000.0 / avg_time
    
    print("-" * 50)
    print("📊 VÝSLEDKY YOLO BENCHMARKU:")
    print(f"Rozlišení:         640x480")
    print(f"⏱️ ČAS NA 1 FRAME: {avg_time:.2f} ms")
    print(f"🚀 RYCHLOST (FPS): {fps:.1f} FPS")
    print("-" * 50)