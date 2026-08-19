import cv2
import numpy as np
import glob
import os
import yaml
import argparse

CHECKERBOARD = (9, 6)
SQUARE_SIZE = 8.2  # mm

def save_calibration(filename, K, D, rms, img_shape, model="fisheye"):
    data = {
        'camera_model': model,
        'camera_matrix': K.tolist(),
        'dist_coeffs': D.tolist(),
        'rms': float(rms),
        'image_shape': img_shape
    }
    with open(filename, 'w') as f:
        yaml.dump(data, f)
    print(f"Calibration saved to {filename}")

def main():
    parser = argparse.ArgumentParser(description="Offline Calibration from captured images")
    parser.add_argument('--dir', default='captured', help="Directory with captured images")
    parser.add_argument('--out', default='calibration.yaml', help="Output file")
    args = parser.parse_args()

    images = glob.glob(os.path.join(args.dir, 'frame_*.jpg')) + glob.glob(os.path.join(args.dir, 'frame_*.png'))
    if not images:
        print(f"No images found in directory '{args.dir}'")
        return

    print(f"Found {len(images)} images in {args.dir}. Extracting corners...")

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

    objpoints = []
    imgpoints = []
    img_shape = None

    for fname in sorted(images):
        img = cv2.imread(fname)
        if img is None:
            continue
            
        if img_shape is None:
            img_shape = img.shape[:2][::-1]

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        try:
            ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        except AttributeError:
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), 
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        if ret:
            corners2 = corners.reshape(1, -1, 2).astype(np.float32)
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(f"  OK: {fname}")
        else:
            print(f"  FAILED: {fname}")

    success_count = len(objpoints)
    print(f"\nSuccessfully found corners in {success_count}/{len(images)} images.")
    
    if success_count < 5:
        print("Not enough valid images to perform calibration (need at least 5).")
        return

    print("\nRunning standard camera calibration first (to get a good initial guess)...")
    try:
        objpoints_std = [o[0] for o in objpoints]
        imgpoints_std = [i[0] for i in imgpoints]
        
        rms_std, K_std, D_std, rvecs_std, tvecs_std = cv2.calibrateCamera(
            objpoints_std, imgpoints_std, img_shape, None, None)
            
        print(f"Standard Calibration successful! RMS error: {rms_std:.4f}")
        print(f"Camera Matrix:\n{K_std}")
        print(f"Distortion Coeffs:\n{D_std}")
        
        # Save standard calibration as a backup
        save_calibration(args.out.replace('.yaml', '_std.yaml'), K_std, D_std, rms_std, img_shape, "standard")
        
        K_guess = K_std.copy()
    except Exception as e2:
        print(f"Standard calibration failed: {e2}")
        K_guess = np.zeros((3, 3))
        K_guess[0, 0] = max(img_shape) * 0.8
        K_guess[1, 1] = max(img_shape) * 0.8
        K_guess[0, 2] = img_shape[0] / 2.0
        K_guess[1, 2] = img_shape[1] / 2.0
        K_guess[2, 2] = 1.0

    print("\nStarting fisheye calibration using the initial guess...")
    
    D_fish = np.zeros((4, 1))
    rvecs_fish = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(success_count)]
    tvecs_fish = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(success_count)]
    
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    
    try:
        rms_f, K_f, D_f, rvecs_f, tvecs_f = cv2.fisheye.calibrate(
            objpoints, imgpoints, img_shape, K_guess, D_fish, rvecs_fish, tvecs_fish, calibration_flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        print(f"Fisheye Calibration successful! RMS error: {rms_f:.4f}")
        print(f"Camera Matrix:\n{K_f}")
        print(f"Distortion Coeffs:\n{D_f}")
        save_calibration(args.out, K_f, D_f, rms_f, img_shape, "fisheye")
    except Exception as e:
        print(f"Fisheye calibration failed: {e}")
        print("Note: The standard calibration result was saved to *_std.yaml")

if __name__ == '__main__':
    main()
