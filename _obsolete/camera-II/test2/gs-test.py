import cv2
import numpy as np
import gi
import sys
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Inicializace GStreameru
Gst.init(None)

def get_camera_frame(sensor_id):
    # Pipeline, která vynutí převod na BGR v RAM dříve, než se toho dotkne OpenCV
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGR ! videoconvert ! "
        "appsink"
    )
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"Chyba: Nelze otevrit pipeline pro senzor {sensor_id}")
        return None
        
    ret, frame = cap.read()
    cap.release()
    return frame

import gi
import numpy as np
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

Gst.init(None)

def get_frame_gst(sensor_id):
    # Pipeline: nvargus -> nvvidconv -> appsink
    # Důležité: 'appsink' nyní necháme vygenerovat buffer GStreamerem
    pipeline_str = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink name=sink sync=false"
    )
        
    pipeline = Gst.parse_launch(pipeline_str)
    sink = pipeline.get_by_name("sink")
    pipeline.set_state(Gst.State.PLAYING)
    
    # Čekání na vzorek
    sample = sink.emit("pull-sample")
    if sample:
        buf = sample.get_buffer()
        # Převod bufferu na numpy pole
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        if result:
            frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape((720, 1280, 3))
            buf.unmap(mapinfo)
            pipeline.set_state(Gst.State.NULL)
            return frame
    pipeline.set_state(Gst.State.NULL)
    return None

frame = get_frame_gst(0)
if frame is not None:
    print(f"ÚSPĚCH! Snímek má tvar: {frame.shape}")
else:
    print("Selhalo.")

    
print("Aruco modul: OK")

# Čtení kamer postupně
print("Načítám levou kameru...")
frame_left = get_camera_frame(0)
print("Načítám pravou kameru...")
frame_right = get_camera_frame(1)

if frame_left is None or frame_right is None:
    print("Error: Nepodařilo se načíst snímek z jedné nebo obou kamer.")
    sys.exit(1)

print(f"Format leve: {frame_left.shape}")
print(f"Format prave: {frame_right.shape}")

# Sloučení
combined = np.hstack([frame_left, frame_right])
cv2.imwrite("stereo_test.png", combined)
print("Sloučený snímek uložen jako: stereo_test.png")