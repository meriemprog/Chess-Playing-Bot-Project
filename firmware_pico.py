"""
firmware_pico.py  (MicroPython — for Pi Pico / ESP32)
────────────────────────────────────────────────────────────────────────────────
Receives serial commands from vision_controller.py and drives the stepper motors.

SERIAL PROTOCOL (received from PC):
  MOVE,E4       → open-loop IK move to square E4
  COR,dx,dy     → small correction in mm (dx/dy floats)
  GRAB          → close gripper
  RELEASE       → open gripper
  HOME          → go to home position

ADAPT THIS TO YOUR SPECIFIC MOTOR WIRING AND STEPS/MM VALUES.

Runs on MicroPython (Pi Pico) or CircuitPython.
Flash MicroPython firmware first: https://micropython.org/download/rp2-pico/
────────────────────────────────────────────────────────────────────────────────
"""

from machine import Pin, UART
import time
import math

# ── UART config (USB serial on Pico = UART0 via USB) ─────────────────────────
# On Pi Pico, sys.stdin/stdout go over USB — just use sys for simplicity.
import sys

# ── Motor config — TUNE THESE ─────────────────────────────────────────────────
# Steps per revolution for your stepper
STEPS_PER_REV = 200

# Gear ratios on each Bras Youpi joint (measure or look up for your model)
GEAR_RATIO_J1  = 1.0      # base rotation
GEAR_RATIO_J2  = 1.0      # shoulder
GEAR_RATIO_J3  = 1.0      # elbow

# Steps per mm of end-effector movement (calibrate empirically)
# You'll calculate this from geometry, then fine-tune experimentally
STEPS_PER_MM_X = 10.0
STEPS_PER_MM_Y = 10.0

# Stepper driver pins (STEP, DIR) for each joint
# Adjust to your actual GPIO wiring
STEP_PINS = {
    "J1": (Pin(2, Pin.OUT), Pin(3, Pin.OUT)),   # base
    "J2": (Pin(4, Pin.OUT), Pin(5, Pin.OUT)),   # shoulder
    "J3": (Pin(6, Pin.OUT), Pin(7, Pin.OUT)),   # elbow
}

# Gripper servo or motor pin
GRIPPER_PIN = Pin(14, Pin.OUT)

# Step timing (microseconds)
STEP_DELAY_US = 800       # pulse width — increase if missing steps
DIR_SETUP_US  = 10        # direction setup time

# ── IK geometry — Bras Youpi arm lengths (mm, measure yours) ─────────────────
L1 = 200.0    # shoulder to elbow length
L2 = 180.0    # elbow to wrist length


# ── Stepper helpers ───────────────────────────────────────────────────────────

def step_motor(joint_key: str, steps: int, forward: bool = True):
    """Send N steps to a joint motor. Negative steps = reverse."""
    step_pin, dir_pin = STEP_PINS[joint_key]
    dir_pin.value(1 if forward else 0)
    time.sleep_us(DIR_SETUP_US)

    for _ in range(abs(steps)):
        step_pin.value(1)
        time.sleep_us(STEP_DELAY_US)
        step_pin.value(0)
        time.sleep_us(STEP_DELAY_US)


def steps_for_angle(angle_deg: float, gear_ratio: float) -> int:
    """Convert joint angle (degrees) to step count."""
    return int(round(angle_deg / 360.0 * STEPS_PER_REV * gear_ratio))


# ── Inverse kinematics (2D planar for 2-joint arm, extend for 3D) ─────────────

def ik_2d(x_mm: float, y_mm: float):
    """
    Simple 2-link planar IK.
    Returns (theta1_deg, theta2_deg) or None if unreachable.
    Adapt for the Bras Youpi's specific geometry.
    """
    d = math.sqrt(x_mm**2 + y_mm**2)

    if d > (L1 + L2) or d < abs(L1 - L2):
        return None   # unreachable

    cos_t2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_t2 = max(-1.0, min(1.0, cos_t2))   # clamp for floating-point safety
    theta2  = math.acos(cos_t2)

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y_mm, x_mm) - math.atan2(k2, k1)

    return math.degrees(theta1), math.degrees(theta2)


# ── Chess square → XY position (mirror of Python side) ───────────────────────

FILES = "ABCDEFGH"
SQUARE_SIZE_MM = 50.0
OFFSET_X = 25.0
OFFSET_Y = 25.0

def square_to_xy(square: str):
    col = FILES.index(square[0].upper())
    row = int(square[1]) - 1
    return OFFSET_X + col * SQUARE_SIZE_MM, OFFSET_Y + row * SQUARE_SIZE_MM


# ── Command handlers ───────────────────────────────────────────────────────────

current_angles = [0.0, 0.0, 0.0]   # track current joint angles

def handle_move(square: str):
    print(f"MOVE {square}")
    x, y = square_to_xy(square)
    angles = ik_2d(x, y)
    if angles is None:
        print(f"ERR: {square} unreachable")
        return

    t1, t2 = angles
    dt1 = t1 - current_angles[0]
    dt2 = t2 - current_angles[1]

    s1 = steps_for_angle(abs(dt1), GEAR_RATIO_J1)
    s2 = steps_for_angle(abs(dt2), GEAR_RATIO_J2)

    step_motor("J1", s1, dt1 >= 0)
    step_motor("J2", s2, dt2 >= 0)

    current_angles[0] = t1
    current_angles[1] = t2
    print(f"OK: moved to ({x:.1f},{y:.1f}) angles=({t1:.1f},{t2:.1f})")


def handle_correction(dx_mm: float, dy_mm: float):
    """
    Translate small XY correction (mm) into steps.
    This is simplified — for a proper solution, differentiate IK at current pose.
    """
    print(f"COR dx={dx_mm:.2f} dy={dy_mm:.2f}")

    # Simple open-loop: convert mm directly to steps
    # For better accuracy, use Jacobian of IK at current joint angles
    sx = int(round(dx_mm * STEPS_PER_MM_X))
    sy = int(round(dy_mm * STEPS_PER_MM_Y))

    if sx != 0:
        step_motor("J1", abs(sx), sx > 0)
    if sy != 0:
        step_motor("J2", abs(sy), sy > 0)

    print(f"OK: correction steps ({sx},{sy})")


def handle_grab():
    print("GRAB")
    GRIPPER_PIN.value(1)    # adapt to your gripper mechanism
    time.sleep(0.5)
    print("OK")


def handle_release():
    print("RELEASE")
    GRIPPER_PIN.value(0)
    time.sleep(0.3)
    print("OK")


def handle_home():
    print("HOME")
    # Drive back to home — adapt to your homing routine
    # Simplest: step back to known zero using limit switch or step counting
    step_motor("J1", steps_for_angle(abs(current_angles[0]), GEAR_RATIO_J1),
               current_angles[0] < 0)
    step_motor("J2", steps_for_angle(abs(current_angles[1]), GEAR_RATIO_J2),
               current_angles[1] < 0)
    current_angles[0] = 0.0
    current_angles[1] = 0.0
    print("OK: home reached")


# ── Main serial loop ───────────────────────────────────────────────────────────

def main():
    print("Bras Youpi firmware ready. Waiting for commands...")

    buf = ""
    while True:
        # Read one character at a time from USB serial
        if sys.stdin in []:     # non-blocking check
            c = sys.stdin.read(1)
        else:
            try:
                c = sys.stdin.read(1)
            except Exception:
                continue

        if c in ('\n', '\r'):
            line = buf.strip()
            buf  = ""

            if not line:
                continue

            parts = line.split(',')
            cmd   = parts[0].upper()

            try:
                if cmd == "MOVE" and len(parts) >= 2:
                    handle_move(parts[1])
                elif cmd == "COR" and len(parts) >= 3:
                    handle_correction(float(parts[1]), float(parts[2]))
                elif cmd == "GRAB":
                    handle_grab()
                elif cmd == "RELEASE":
                    handle_release()
                elif cmd == "HOME":
                    handle_home()
                else:
                    print(f"UNKNOWN: {line}")
            except Exception as e:
                print(f"ERR: {e}")
        else:
            buf += c


main()
