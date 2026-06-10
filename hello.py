import platform
print("Hello from Jetson!", platform.uname())

import torch
import cv2
import numpy as np
import sys

try:
    import ultralytics
    ultralytics_version = ultralytics.__version__
except ImportError:
    ultralytics_version = "Not installed"

print("--- ENVIRONMENT DIAGNOSTICS ---")
print(f"Python version: {sys.version.split()[0]}")
print(f"CUDA available: {torch.cuda.is_available()}")

if torch.cuda.is_available():
    print(f"CUDA version (PyTorch): {torch.version.cuda}")
    print(f"GPU Device: {torch.cuda.get_device_name(0)}")
else:
    print("Warning: CUDA is NOT available. Check if --runtime nvidia was used.")

print(f"Torch version: {torch.__version__}")
print(f"OpenCV (cv2) version: {cv2.__version__}")
print(f"NumPy version: {np.__version__}")
print(f"Ultralytics (YOLO) version: {ultralytics_version}")
print("-------------------------------")