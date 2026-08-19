import cv2
import zmq
import argparse
import time
import sys

def get_pipeline(camera):
    if camera == 'l':
        sensor_id = 0
        flip = 3
        aeregion = "86 320 722 950 1.0"
    else:
        sensor_id = 1
        flip = 1
        aeregion = "524 330 1148 954 1.0"
        
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} aeregion=\"{aeregion}\" exposuretimerange=\"34000 5000000\" gainrange=\"1.0 16.0\" ! "
        f"video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=10/1 ! "
        f"nvvidconv flip-method={flip} ! video/x-raw, width=1232, height=1640, format=BGRx ! "
        f"videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false max-buffers=1"
    )
    return pipeline

def main():
    parser = argparse.ArgumentParser(description="Remote Calibration Server (REP)")
    parser.add_argument('camera', choices=['l', 'r'], help="Select camera: 'l' for left, 'r' for right")
    parser.add_argument('--port', type=int, default=5555, help="ZMQ port to publish on")
    args = parser.parse_args()

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind(f"tcp://0.0.0.0:{args.port}")

    pipeline = get_pipeline(args.camera)
    print(f"Starting pipeline:\n{pipeline}")
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera.", file=sys.stderr)
        return
        
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"Server started on port {args.port}, serving camera '{args.camera}'")
    
    try:
        latest_frame = None
        while True:
            # Read frame to keep buffer empty
            ret, frame = cap.read()
            if ret:
                latest_frame = frame
            
            try:
                # Check if client is requesting an image
                msg = socket.recv(flags=zmq.NOBLOCK)
                
                if latest_frame is not None:
                    # Encode frame
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                    result, encimg = cv2.imencode('.jpg', latest_frame, encode_param)
                    if result:
                        socket.send(encimg.tobytes())
                    else:
                        socket.send(b"ERROR")
                else:
                    socket.send(b"WAIT")
            except zmq.Again:
                pass
            
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        cap.release()
        socket.close()
        context.term()

if __name__ == '__main__':
    main()
