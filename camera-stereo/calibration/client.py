import cv2
import zmq
import numpy as np
import argparse
import yaml

import os
import shutil

# Checkboard settings from user
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 8.2  # mm

def calibrate_fisheye(objpoints, imgpoints, img_shape):
    print("Starting fisheye calibration...")
    # Run standard calibration first to get a good initial guess
    try:
        objpoints_std = [o[0] for o in objpoints]
        imgpoints_std = [i[0] for i in imgpoints]
        rms_std, K_std, D_std, rvecs_std, tvecs_std = cv2.calibrateCamera(
            objpoints_std, imgpoints_std, img_shape, None, None)
        K_guess = K_std.copy()
        print(f"Standard calibration fallback successful. RMS: {rms_std:.4f}")
    except Exception as e:
        print(f"Standard calibration guess failed: {e}")
        K_guess = np.zeros((3, 3))
        K_guess[0, 0] = max(img_shape) * 0.8
        K_guess[1, 1] = max(img_shape) * 0.8
        K_guess[0, 2] = img_shape[0] / 2.0
        K_guess[1, 2] = img_shape[1] / 2.0
        K_guess[2, 2] = 1.0
        
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(len(objpoints))]
    
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_shape, K_guess, D, rvecs, tvecs, calibration_flags,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
    
    return rms, K, D

def save_calibration(filename, K, D, rms, img_shape):
    data = {
        'camera_matrix': K.tolist(),
        'dist_coeffs': D.tolist(),
        'rms': rms,
        'image_shape': img_shape
    }
    with open(filename, 'w') as f:
        yaml.dump(data, f)
    print(f"Calibration saved to {filename}")

def main():
    parser = argparse.ArgumentParser(description="Remote Calibration Client (REQ)")
    parser.add_argument('--ip', required=True, default='192.168.10.2', help="IP address of the server")
    parser.add_argument('--port', type=int, default=5555, help="ZMQ port of the server")
    parser.add_argument('--out', default='calibration.yaml', help="Output file for calibration results")
    args = parser.parse_args()

    # Clear and recreate captured directory
    captured_dir = "captured"
    if os.path.exists(captured_dir):
        shutil.rmtree(captured_dir)
    os.makedirs(captured_dir)
    print(f"Directory '{captured_dir}' is ready for saved frames.")

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{args.ip}:{args.port}")

    print(f"Connected to tcp://{args.ip}:{args.port}")
    print("Controls:")
    print("  'n' - Request new frame from server")
    print("  's' - Save current frame for calibration")
    print("  'c' - Run calibration with saved frames")
    print("  'q' - Quit")

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

    objpoints = []
    imgpoints = []

    collected = 0
    img_shape = None
    last_frame = None
    ret = False
    corners = None

    cv2.namedWindow('Calibration Client', cv2.WINDOW_NORMAL)

    def process_frame(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Try findChessboardCornersSB first (much more robust for fisheye)
        try:
            found, corn = cv2.findChessboardCornersSB(gray, CHECKERBOARD, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        except AttributeError:
            # Fallback for older OpenCV
            found, corn = cv2.findChessboardCorners(gray, CHECKERBOARD, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                corn = cv2.cornerSubPix(gray, corn, (11,11), (-1,-1), 
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                
        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, CHECKERBOARD, corn, found)
        return disp, found, corn

    display_frame = None

    # Initial frame request
    print("Requesting initial frame...")
    socket.send(b"REQ")
    msg = socket.recv()
    if msg not in (b"ERROR", b"WAIT"):
        npimg = np.frombuffer(msg, dtype=np.uint8)
        last_frame = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
        if last_frame is not None:
            img_shape = last_frame.shape[:2][::-1]
            display_frame, ret, corners = process_frame(last_frame)

    while True:
        if display_frame is not None:
            show_frame = display_frame.copy()
            cv2.putText(show_frame, f"Collected: {collected}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Calibration Client', show_frame)
        else:
            # Blank screen
            blank = np.zeros((100, 400, 3), dtype=np.uint8)
            cv2.putText(blank, "Press 'n' for new frame", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow('Calibration Client', blank)

        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):
            print("Requesting new frame...")
            socket.send(b"REQ")
            msg = socket.recv()
            if msg not in (b"ERROR", b"WAIT"):
                npimg = np.frombuffer(msg, dtype=np.uint8)
                last_frame = cv2.imdecode(npimg, cv2.IMREAD_COLOR)
                if last_frame is not None:
                    if img_shape is None:
                        img_shape = last_frame.shape[:2][::-1]
                    display_frame, ret, corners = process_frame(last_frame)
                    print("New frame received!")
            else:
                print("Server returned:", msg)
        elif key == ord('s'):
            if ret and corners is not None and last_frame is not None:
                # Save physically (as PNG to prevent JPEG artifacts which break fisheye optimization)
                save_path = os.path.join(captured_dir, f"frame_{collected:03d}.png")
                cv2.imwrite(save_path, last_frame)
                
                corners2 = corners.reshape(1, -1, 2).astype(np.float32)
                
                objpoints.append(objp)
                imgpoints.append(corners2)
                collected += 1
                print(f"Frame saved as {save_path}. Total collected: {collected}")
            else:
                print("Cannot save frame, chessboard not detected! (If you have a photo, please provide it.)")
                # Also save the physical frame for debugging even if not detected
                save_path = os.path.join(captured_dir, f"failed_frame_{collected:03d}.jpg")
                cv2.imwrite(save_path, last_frame)
                print(f"Saved failed frame for debugging: {save_path}")
        elif key == ord('c'):
            if collected < 5:
                print(f"Need at least 5 frames to calibrate! Currently have: {collected}")
            else:
                try:
                    rms, K, D = calibrate_fisheye(objpoints, imgpoints, img_shape)
                    print(f"Calibration successful! RMS error: {rms}")
                    print(f"Camera Matrix:\n{K}")
                    print(f"Distortion Coeffs:\n{D}")
                    save_calibration(args.out, K, D, rms, img_shape)
                except Exception as e:
                    print(f"\n❌ Calibration failed: {e}")
                    print("⚠️ The algorithm couldn't calculate the lens distortion.")
                    print("💡 TIP: You need more variety in your photos. Try capturing the chessboard:")
                    print("   - Closer to the edges of the image")
                    print("   - Tilted at different angles (up, down, left, right)")
                    print("▶️ You can CONTINUE capturing by pressing 'n' (new) and 's' (save) right now. Your existing points are kept in memory!\n")
                
    cv2.destroyAllWindows()
    socket.close()
    context.term()

if __name__ == '__main__':
    main()
