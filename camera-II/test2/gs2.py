import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Inicializace GStreameru
Gst.init(None)

def get_camera_frame(sensor_id):
    # Pipeline, která vynutí převod na BGR v RAM dříve, než se toho dotkne OpenCV
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGR ! "
        "appsink name=sink emit-signals=True max-buffers=1 drop=True"
    )
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"Chyba: Nelze otevrit pipeline pro senzor {sensor_id}")
        return None
        
    ret, frame = cap.read()
    cap.release()
    return frame