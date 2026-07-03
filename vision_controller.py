"""
vision_controller.py
────────────────────────────────────────────────────────────────────────────────
Closed-loop visual correction for the Bras Youpi chess robot.

PIPELINE:
  1. Open-loop IK move → arm reaches approximately the right square
  2. Camera detects ArUco on gripper tip and fixed board reference marker
  3. Computes gripper position error in board frame (mm)
  4. Sends small correction steps to microcontroller over serial
  5. Repeats until error < threshold OR max iterations reached
  6. Sends GRAB command

MARKER SETUP:
  - Print marker ID 0 (small, ~20mm) → glue to gripper tip face-up
  - Print marker ID 1 (large, ~40mm) → fix to board corner A1 (never moves)
  - Generate markers: https://chev.me/arucogen/
    Dictionary: 4x4_50, IDs: 0 and 1

SERIAL PROTOCOL (sent to Pi Pico / ESP32):
  "COR,dx,dy\n"     → correction in mm (your firmware converts to steps)
  "GRAB\n"          → close gripper
  "RELEASE\n"       → open gripper
  "HOME\n"          → go to home position

WIRING (serial):
  Pi Pico:   USB serial (COM port on Windows, /dev/ttyACM0 on Linux)
  ESP32:     USB serial or /dev/ttyUSB0

DEPENDENCIES:
  pip install opencv-contrib-python numpy pyserial
────────────────────────────────────────────────────────────────────────────────
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import serial
import serial.tools.list_ports
import time
import os
import json
from dataclasses import dataclass, field
from typing import Optional, Tuple


# ══════════════════════════════════════════════════════════════════════════════
#   CONFIGURATION  —  edit this section to match your setup
# ══════════════════════════════════════════════════════════════════════════════

CALIBRATION_FILE        = "camera_calibration.npz"
CAMERA_INDEX            = 1
SERIAL_PORT             = "AUTO"          # "AUTO" to scan, or e.g. "COM4" / "/dev/ttyUSB0"
SERIAL_BAUD             = 115200

# ArUco
ARUCO_DICT_NAME         = aruco.DICT_4X4_50
GRIPPER_MARKER_ID       = 0               # Marker on gripper tip
BOARD_REF_MARKER_ID     = 1               # Fixed marker on board corner A1
GRIPPER_MARKER_SIZE_M   = 0.020           # 20 mm → meters
BOARD_MARKER_SIZE_M     = 0.040           # 40 mm → meters

# Closed-loop parameters
POSITION_THRESHOLD_MM   = 3.0             # "close enough" tolerance
MAX_CORRECTION_ITERS    = 8               # max correction attempts per move
CORRECTION_WAIT_S       = 0.6            # seconds to wait after sending correction
MIN_CORRECTION_MM       = 0.5            # ignore error smaller than this (noise floor)

# Chess board geometry (measured from board reference marker at A1 corner)
# These are the XY offsets in mm from the A1 ArUco marker center to each square center
# Tune these to YOUR board after measuring
SQUARE_SIZE_MM          = 50.0            # standard chess square size
BOARD_OFFSET_X_MM       = 25.0           # offset from A1 marker to center of square A1
BOARD_OFFSET_Y_MM       = 25.0

# ══════════════════════════════════════════════════════════════════════════════


# ── Chess coordinate helpers ───────────────────────────────────────────────────

CHESS_FILES = "ABCDEFGH"

def square_to_xy(square: str) -> Tuple[float, float]:
    """
    Convert chess notation (e.g. "E4") to XY position in board frame (mm).
    Origin = center of A1 square, measured from board reference marker.
    """
    col = CHESS_FILES.index(square[0].upper())   # A=0 … H=7
    row = int(square[1]) - 1                      # 1=0 … 8=7
    x   = BOARD_OFFSET_X_MM + col * SQUARE_SIZE_MM
    y   = BOARD_OFFSET_Y_MM + row * SQUARE_SIZE_MM
    return x, y


# ── Serial helpers ─────────────────────────────────────────────────────────────

def find_serial_port() -> Optional[str]:
    """Try to auto-detect the Pi Pico / ESP32 serial port."""
    candidates = [
        p.device for p in serial.tools.list_ports.comports()
        if any(kw in (p.description or "").lower()
               for kw in ["usb", "uart", "pico", "esp", "ch340", "cp210", "ftdi"])
    ]
    return candidates[0] if candidates else None


def open_serial(port: str, baud: int) -> Optional[serial.Serial]:
    if port == "AUTO":
        port = find_serial_port()
        if port is None:
            print("  ⚠ No serial port found. Running in SIMULATION mode.")
            return None
        print(f"  Auto-detected serial port: {port}")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)   # wait for microcontroller to boot
        print(f"  ✓ Serial connected → {port} @ {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"  ✗ Serial error: {e}")
        print("  Running in SIMULATION mode (no commands sent).")
        return None


def send_cmd(ser: Optional[serial.Serial], cmd: str):
    """Send a newline-terminated command string."""
    full = cmd.strip() + "\n"
    if ser:
        ser.write(full.encode())
    print(f"  → TX: {full.strip()}")


# ── Camera calibration loader ──────────────────────────────────────────────────

def load_calibration(path: str) -> Tuple[np.ndarray, np.ndarray]:
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Calibration file not found: {path}\n"
            "Run calibrate_camera.py first!"
        )
    data = np.load(path)
    mtx  = data["camera_matrix"]
    dist = data["dist_coeffs"]
    print(f"  ✓ Loaded calibration from {path}")
    return mtx, dist


# ── ArUco detection & pose estimation ─────────────────────────────────────────

def make_detector() -> aruco.ArucoDetector:
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    params     = aruco.DetectorParameters()
    # Improve detection for small markers
    params.minMarkerPerimeterRate = 0.02
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return aruco.ArucoDetector(dictionary, params)


def marker_object_points(size_m: float) -> np.ndarray:
    """3D corners of a square marker centred at origin."""
    h = size_m / 2
    return np.array([
        [-h,  h, 0],
        [ h,  h, 0],
        [ h, -h, 0],
        [-h, -h, 0],
    ], dtype=np.float32)


def solve_marker_pose(
    corners: np.ndarray,
    size_m:  float,
    cam_mtx: np.ndarray,
    dist:    np.ndarray
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Returns (rvec, tvec) or (None, None) on failure."""
    obj_pts = marker_object_points(size_m)
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, corners, cam_mtx, dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    return (rvec, tvec) if ok else (None, None)


def to_transform(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert rvec + tvec → 4×4 homogeneous transform matrix."""
    R, _ = cv2.Rodrigues(rvec)
    T     = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3]  = tvec.flatten()
    return T


@dataclass
class FrameResult:
    gripper_T:   Optional[np.ndarray] = None   # 4×4 cam→gripper
    board_ref_T: Optional[np.ndarray] = None   # 4×4 cam→board_ref
    gripper_pos_in_board: Optional[np.ndarray] = None   # XYZ mm in board frame
    annotated_frame: Optional[np.ndarray] = None


def process_frame(
    frame:   np.ndarray,
    detector: aruco.ArucoDetector,
    cam_mtx: np.ndarray,
    dist:    np.ndarray
) -> FrameResult:
    result = FrameResult()
    result.annotated_frame = frame.copy()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners_list, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return result

    ids_flat = ids.flatten()
    aruco.drawDetectedMarkers(result.annotated_frame, corners_list, ids)

    for i, mid in enumerate(ids_flat):
        c = corners_list[i][0]   # shape (4,2)

        if mid == GRIPPER_MARKER_ID:
            rv, tv = solve_marker_pose(c, GRIPPER_MARKER_SIZE_M, cam_mtx, dist)
            if rv is not None:
                result.gripper_T = to_transform(rv, tv)
                cv2.drawFrameAxes(result.annotated_frame, cam_mtx, dist,
                                  rv, tv, GRIPPER_MARKER_SIZE_M * 0.6)

        elif mid == BOARD_REF_MARKER_ID:
            rv, tv = solve_marker_pose(c, BOARD_MARKER_SIZE_M, cam_mtx, dist)
            if rv is not None:
                result.board_ref_T = to_transform(rv, tv)
                cv2.drawFrameAxes(result.annotated_frame, cam_mtx, dist,
                                  rv, tv, BOARD_MARKER_SIZE_M * 0.6)

    # Compute gripper position in board frame if both markers visible
    if result.gripper_T is not None and result.board_ref_T is not None:
        T_board_cam    = np.linalg.inv(result.board_ref_T)
        T_board_gripper = T_board_cam @ result.gripper_T
        result.gripper_pos_in_board = T_board_gripper[:3, 3] * 1000.0   # m → mm

    return result


def grab_stable_measurement(
    cap:     cv2.VideoCapture,
    detector: aruco.ArucoDetector,
    cam_mtx: np.ndarray,
    dist:    np.ndarray,
    n_frames: int = 5
) -> FrameResult:
    """
    Average gripper position over N frames for a stable measurement.
    Falls back to first successful frame if averaging fails.
    """
    positions = []
    last_result = FrameResult()

    for _ in range(n_frames + 5):   # extra attempts in case of bad frames
        ret, frame = cap.read()
        if not ret:
            continue

        r = process_frame(frame, detector, cam_mtx, dist)
        last_result = r

        if r.gripper_pos_in_board is not None:
            positions.append(r.gripper_pos_in_board.copy())

        if len(positions) >= n_frames:
            break

    if positions:
        last_result.gripper_pos_in_board = np.mean(positions, axis=0)

    return last_result


# ── Visualisation overlay ──────────────────────────────────────────────────────

def draw_hud(
    frame:       np.ndarray,
    gripper_pos: Optional[np.ndarray],
    target_pos:  Optional[np.ndarray],
    iteration:   int,
    status:      str,
    error_mm:    Optional[float] = None
):
    h, w = frame.shape[:2]
    font  = cv2.FONT_HERSHEY_SIMPLEX

    # Semi-transparent bottom bar
    ov = frame.copy()
    cv2.rectangle(ov, (0, h - 110), (w, h), (15, 15, 15), -1)
    cv2.addWeighted(ov, 0.65, frame, 0.35, 0, frame)

    def put(text, y, color=(220, 220, 220), scale=0.55):
        cv2.putText(frame, text, (12, y), font, scale, color, 1, cv2.LINE_AA)

    if gripper_pos is not None:
        put(f"Gripper : X={gripper_pos[0]:6.1f}  Y={gripper_pos[1]:6.1f} mm",
            h - 88, (80, 220, 80))

    if target_pos is not None:
        put(f"Target  : X={target_pos[0]:6.1f}  Y={target_pos[1]:6.1f} mm",
            h - 62, (80, 180, 255))

    if error_mm is not None:
        color = (0, 220, 80) if error_mm < POSITION_THRESHOLD_MM else (0, 100, 255)
        put(f"Error   : {error_mm:5.1f} mm  (threshold {POSITION_THRESHOLD_MM:.1f} mm)  iter {iteration}",
            h - 36, color)

    put(f"Status  : {status}", h - 10, (200, 200, 200))

    # Marker status indicator top-right
    def dot(label, present, x, y):
        col = (0, 200, 80) if present else (0, 60, 220)
        cv2.circle(frame, (x, y), 7, col, -1)
        cv2.putText(frame, label, (x + 12, y + 5), font, 0.45, col, 1)

    dot(f"Gripper  (ID {GRIPPER_MARKER_ID})",  gripper_pos is not None,  w - 200, 25)
    dot(f"Board ref (ID {BOARD_REF_MARKER_ID})", gripper_pos is not None and target_pos is not None, w - 200, 50)


# ── Main controller class ──────────────────────────────────────────────────────

class VisionController:

    def __init__(self):
        print("\n╔══════════════════════════════════════════════╗")
        print("║   Bras Youpi — Vision Controller              ║")
        print("╚══════════════════════════════════════════════╝\n")

        self.cam_mtx, self.dist = load_calibration(CALIBRATION_FILE)
        self.detector            = make_detector()
        self.ser                 = open_serial(SERIAL_PORT, SERIAL_BAUD)
        self.cap                 = cv2.VideoCapture(CAMERA_INDEX)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {CAMERA_INDEX}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)      # disable autofocus for stable pose

        self.target_xy: Optional[np.ndarray] = None
        self.status = "IDLE"
        print("\n  ✓ Vision controller ready.\n")

    # ── Target management ──────────────────────────────────────────────────

    def set_target_mm(self, x_mm: float, y_mm: float):
        """Set target directly in board frame mm."""
        self.target_xy = np.array([x_mm, y_mm], dtype=np.float64)
        print(f"  Target set → ({x_mm:.1f}, {y_mm:.1f}) mm")

    def set_target_square(self, square: str):
        """Set target by chess notation e.g. 'E4'."""
        x, y = square_to_xy(square)
        print(f"  Target square {square.upper()} → ({x:.1f}, {y:.1f}) mm")
        self.set_target_mm(x, y)

    # ── Core correction loop ───────────────────────────────────────────────

    def correction_loop(self, verbose: bool = True) -> bool:
        """
        Run closed-loop correction. Call AFTER the open-loop move completes.
        Returns True if gripper reached target within threshold.
        """
        if self.target_xy is None:
            print("  ✗ No target set. Call set_target_square() first.")
            return False

        self.status = "CORRECTING"
        print(f"\n  ── Correction loop (threshold={POSITION_THRESHOLD_MM}mm) ──")

        for iteration in range(1, MAX_CORRECTION_ITERS + 1):
            # Grab stable measurement
            result = grab_stable_measurement(
                self.cap, self.detector, self.cam_mtx, self.dist
            )

            frame = result.annotated_frame
            gpos  = result.gripper_pos_in_board

            if gpos is None:
                if verbose:
                    print(f"  Iter {iteration}: Markers not visible — skipping")
                self._show(frame, None, iteration, "Markers not visible")
                time.sleep(0.2)
                continue

            # Compute XY error
            error_xy  = self.target_xy - gpos[:2]
            error_dist = float(np.linalg.norm(error_xy))

            if verbose:
                print(f"  Iter {iteration}: "
                      f"pos=({gpos[0]:.1f},{gpos[1]:.1f})  "
                      f"err=({error_xy[0]:.1f},{error_xy[1]:.1f})  "
                      f"dist={error_dist:.1f}mm")

            self._show(frame, gpos, iteration,
                       f"Correcting…  err={error_dist:.1f}mm", error_dist)

            # ── On target? ──────────────────────────────────────────────
            if error_dist < POSITION_THRESHOLD_MM:
                print(f"\n  ✓ On target!  ({error_dist:.1f}mm < {POSITION_THRESHOLD_MM}mm)")
                self.status = "ON_TARGET"
                return True

            # ── Send correction if error is meaningful ──────────────────
            if error_dist > MIN_CORRECTION_MM:
                send_cmd(self.ser, f"COR,{error_xy[0]:.2f},{error_xy[1]:.2f}")
                time.sleep(CORRECTION_WAIT_S)

        print(f"\n  ⚠ Max iterations reached. Final error may exceed threshold.")
        self.status = "CORRECTION_TIMEOUT"
        return False

    # ── High-level move commands ───────────────────────────────────────────

    def move_and_correct(self, target_square: str, move_speed_s: float = 2.5) -> bool:
        """
        Coarse open-loop move to a square, then close-loop correction.
        Returns True if gripper reached target within threshold.
        """
        self.set_target_square(target_square)
        send_cmd(self.ser, f"MOVE,{target_square.upper()}")
        print(f"  Waiting for open-loop move to complete…")
        time.sleep(move_speed_s)
        return self.correction_loop()

    def grab(self):
        send_cmd(self.ser, "GRAB")
        time.sleep(0.8)

    def release(self):
        send_cmd(self.ser, "RELEASE")
        time.sleep(0.8)

    def descend(self):
        """Tell arm to go down to PICK_Z at current XY."""
        send_cmd(self.ser, "DOWN")
        time.sleep(1.5)

    def rise(self):
        """Tell arm to go up to SAFE_Z at current XY."""
        send_cmd(self.ser, "UP")
        time.sleep(1.5)

    def home(self):
        send_cmd(self.ser, "HOME")
        self.status = "HOMING"
        time.sleep(2.5)

    def execute_chess_move(self, src_square: str, dst_square: str) -> bool:
        """
        Full vision-corrected chess move:
          src_square  e.g. "E2"  — where to pick up
          dst_square  e.g. "E4"  — where to place

        Sequence:
          1. Fly to above src → vision correct → descend → grab → rise
          2. Fly to above dst → vision correct → descend → release → rise
          3. Return home
        """
        print(f"\n{'='*50}")
        print(f"  CHESS MOVE  {src_square.upper()} → {dst_square.upper()}")
        print(f"{'='*50}")

        # ── Phase 1: Pick ──────────────────────────────────────
        print("\n  Phase 1: Pick from", src_square.upper())
        if not self.move_and_correct(src_square):
            print("  ✗ Could not reach pick square accurately. Aborting.")
            self.home()
            return False

        self.descend()
        self.grab()
        self.rise()

        # ── Phase 2: Place ─────────────────────────────────────
        print("\n  Phase 2: Place on", dst_square.upper())
        if not self.move_and_correct(dst_square):
            print("  ✗ Could not reach place square accurately.")
            print("  Releasing piece at best-effort position.")

        self.descend()
        self.release()
        self.rise()

        # ── Phase 3: Home ──────────────────────────────────────
        print("\n  Phase 3: Returning home")
        self.home()

        print(f"\n  ✓ Move {src_square.upper()} → {dst_square.upper()} complete.")
        return True

    # ── Live monitor mode ──────────────────────────────────────────────────

    def run_monitor(self):
        """
        Live camera view with ArUco overlay.
        Keys:
          Q       — quit
          H       — send HOME
          G       — grab
          R       — release
          C       — run correction loop (if target set)
          1-8     — quick target: A1…A8 column test
        """
        print("\n  Monitor mode active.")
        print("  Keys: Q=quit  H=home  G=grab  R=release  C=correct")
        self.status = "MONITOR"
        iteration   = 0

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            result   = process_frame(frame, self.detector, self.cam_mtx, self.dist)
            gpos     = result.gripper_pos_in_board
            err_dist = None

            if gpos is not None and self.target_xy is not None:
                err_dist = float(np.linalg.norm(self.target_xy - gpos[:2]))

            draw_hud(
                result.annotated_frame,
                gpos,
                self.target_xy,
                iteration,
                self.status,
                err_dist
            )

            cv2.imshow("Bras Youpi — Vision Controller", result.annotated_frame)
            key = cv2.waitKey(1) & 0xFF

            if   key == ord('q'):  break
            elif key == ord('h'):  self.home()
            elif key == ord('g'):  self.grab()
            elif key == ord('r'):  self.release()
            elif key == ord('d'):  self.descend()
            elif key == ord('u'):  self.rise()
            elif key == ord('c'):  self.correction_loop()

        self.cleanup()

    # ── Internal helpers ───────────────────────────────────────────────────

    def _show(self, frame, gpos, iteration, status_msg, error_mm=None):
        if frame is None:
            return
        draw_hud(frame, gpos, self.target_xy, iteration, status_msg, error_mm)
        cv2.imshow("Bras Youpi — Vision Controller", frame)
        cv2.waitKey(1)

    def cleanup(self):
        self.cap.release()
        if self.ser:
            self.ser.close()
        cv2.destroyAllWindows()
        print("\n  Goodbye.")


# ══════════════════════════════════════════════════════════════════════════════
#   ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    vc = VisionController()

    # ── Example: move piece from E2 to E4 ──────────────────────────────────
    # Uncomment once hardware is wired up:
    #
    # vc.execute_chess_move("E2", "E4")

    # Default: open the live monitor
    # Use keys in the window:
    #   Q = quit
    #   H = home
    #   G = grab
    #   R = release
    #   D = descend (DOWN)
    #   U = rise (UP)
    #   C = run correction loop (if target set)
    vc.run_monitor()
