"""
calibrate_camera.py
────────────────────────────────────────────────────────────────────────────────
Step 1 of the vision pipeline for the Bras Youpi chess robot.

HOW TO USE:
  1. Print a checkerboard pattern (9x6 inner corners, 25mm squares recommended).
     Print one here: https://calib.io/pages/camera-calibration-pattern-generator
  2. Run this script: python calibrate_camera.py
  3. Hold the checkerboard in front of the camera at various angles.
  4. Press SPACE to capture when the board is detected (green corners shown).
  5. Collect at least 20 images from different angles/distances.
  6. The script saves camera_calibration.npz — used by vision_controller.py.

TIPS FOR GOOD CALIBRATION:
  - Tilt the board in X, Y, and Z — don't keep it flat/parallel to camera
  - Cover the full field of view (corners, center, edges)
  - RMS error < 0.5 is excellent, < 1.0 is acceptable, > 1.5 means redo it
────────────────────────────────────────────────────────────────────────────────
"""

import cv2
import numpy as np
import os
import time

# ── CONFIG ────────────────────────────────────────────────────────────────────
CHECKERBOARD       = (7, 7)         # inner corners (width, height) — NOT squares count
SQUARE_SIZE_MM     = 27.5           # physical size of each square in mm
NUM_IMAGES_NEEDED  = 20             # minimum good captures
OUTPUT_FILE        = "camera_calibration.npz"
CAMERA_INDEX       = 1              # 0 = default webcam
CAPTURE_COOLDOWN_S = 1.0            # seconds between captures to avoid duplicates
# ─────────────────────────────────────────────────────────────────────────────


def build_object_points():
    """3D points of checkerboard corners in the board's coordinate frame."""
    obj_p = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    obj_p[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    obj_p *= SQUARE_SIZE_MM
    return obj_p


def draw_ui(frame, count, found, rms=None):
    h, w = frame.shape[:2]

    # Status bar background
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h - 80), (w, h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

    font = cv2.FONT_HERSHEY_SIMPLEX

    if found:
        color = (0, 220, 80)
        msg = f"Board detected!  SPACE to capture  [{count}/{NUM_IMAGES_NEEDED}]"
    else:
        color = (40, 40, 220)
        msg = f"Searching for checkerboard...  [{count}/{NUM_IMAGES_NEEDED}]"

    cv2.putText(frame, msg,    (12, h - 50), font, 0.6, color, 2)
    cv2.putText(frame, "Q = quit early and calibrate with what you have",
                (12, h - 20), font, 0.45, (180, 180, 180), 1)

    if rms is not None:
        quality = "EXCELLENT" if rms < 0.5 else ("GOOD" if rms < 1.0 else "POOR — REDO")
        q_color = (0, 220, 80) if rms < 1.0 else (0, 80, 220)
        cv2.putText(frame, f"Calibration RMS: {rms:.4f}  [{quality}]",
                    (12, 35), font, 0.65, q_color, 2)


def calibrate():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_p     = build_object_points()

    obj_points = []   # 3D world points
    img_points = []   # 2D image points
    frame_size = None

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("=" * 60)
    print("  Camera Calibration — Bras Youpi Chess Robot")
    print("=" * 60)
    print(f"  Checkerboard: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners")
    print(f"  Square size:  {SQUARE_SIZE_MM} mm")
    print(f"  Target:       {NUM_IMAGES_NEEDED} images")
    print("=" * 60)
    print()

    count        = 0
    last_capture = 0.0

    while count < NUM_IMAGES_NEEDED:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_size = gray.shape[::-1]   # (width, height)

        found, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()

        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners_refined, found)

        draw_ui(display, count, found)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        now = time.time()

        if key == ord(' ') and found and (now - last_capture) > CAPTURE_COOLDOWN_S:
            obj_points.append(obj_p)
            img_points.append(corners_refined)
            count += 1
            last_capture = now
            print(f"  ✓ Captured {count}/{NUM_IMAGES_NEEDED}")

        elif key == ord('q'):
            print(f"\n  Stopping early with {count} images...")
            break

    cap.release()

    if count < 5:
        print("\n  ✗ Not enough images (need ≥ 5). Aborting.")
        cv2.destroyAllWindows()
        return

    print(f"\n  Computing calibration from {count} images...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, frame_size, None, None
    )

    # ── Show result ────────────────────────────────────────────────────────
    rms = ret
    print(f"\n  RMS reprojection error: {rms:.4f}")
    if rms > 1.0:
        print("  ⚠ RMS > 1.0 — consider redoing with more varied angles")
    else:
        print("  ✓ Calibration looks good!")

    print(f"\n  Camera matrix:\n{mtx}\n")
    print(f"  Distortion coefficients:\n{dist}\n")

    np.savez(OUTPUT_FILE, camera_matrix=mtx, dist_coeffs=dist)
    print(f"  Saved → {OUTPUT_FILE}")

    # ── Show undistorted preview ───────────────────────────────────────────
    cap2 = cv2.VideoCapture(CAMERA_INDEX)
    print("\n  Showing undistorted preview. Press any key to exit.")
    while True:
        ret, frame = cap2.read()
        if not ret:
            break
        undistorted = cv2.undistort(frame, mtx, dist)
        combined    = np.hstack([frame, undistorted])
        h, w = combined.shape[:2]
        combined = cv2.resize(combined, (min(w, 1280), min(h, 480)))
        cv2.putText(combined, "Original", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        cv2.putText(combined, "Undistorted", (combined.shape[1]//2 + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2)
        draw_ui(combined, count, False, rms)
        cv2.imshow("Calibration Result", combined)
        if cv2.waitKey(1) != -1:
            break

    cap2.release()
    cv2.destroyAllWindows()
    print("\n  Done. Run vision_controller.py next.")


if __name__ == "__main__":
    calibrate()
