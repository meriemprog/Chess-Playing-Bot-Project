"""
mouhawelt_omar_vision.py  —  Bras Youpi Pico Firmware (Vision-Integrated)
────────────────────────────────────────────────────────────────────────────────
Adapted from mouhawelt_omar.py to work with vision_controller.py on the PC.

WHAT CHANGED vs the original:
  ✓ Main loop now speaks the vision protocol (MOVE / COR / DOWN / UP / GRAB / RELEASE / HOME)
  ✓ Added chess board → robot XY coordinate mapping  (tune BOARD_* constants)
  ✓ COR command reuses your existing IK — no STEPS_PER_MM approximation needed
  ✓ Pick sequence split into individual steps so vision can correct between each one
  ✓ Fixed out-of-scope t1/t2/t3 bug in the original pick_sequence
  ✓ Original pick_sequence kept as PICK command for legacy/manual testing

SERIAL PROTOCOL  (sent from vision_controller.py over USB):
  MOVE,E4           → fly to above square E4 at SAFE_Z  (open-loop positioning)
  COR,dx,dy         → small XY correction in mm  (uses real IK — very accurate)
  DOWN              → descend from SAFE_Z to PICK_Z  (at current XY)
  UP                → rise from current Z back to SAFE_Z
  GRAB              → close gripper  (stepper +1024 steps)
  RELEASE           → open gripper  (stepper -1024 steps)
  HOME              → return to saved home position
  POS               → print current XYZ (for debugging)
  PICK,E2,E4        → full legacy pick & place between two squares  (no vision)

REPLIES (sent back to PC):
  OK                → command succeeded
  OK:...            → command succeeded with info
  ERR:...           → command failed with reason

HOW THE VISION LOOP USES THIS  (orchestrated from vision_controller.py):
  1. PC sends  MOVE,E2          → arm flies to above E2 at safe height
  2. Camera reads gripper ArUco → computes error
  3. PC sends  COR,dx,dy        → arm corrects using IK  (repeated until < 3mm)
  4. PC sends  DOWN             → arm descends to PICK_Z
  5. PC sends  GRAB             → gripper closes
  6. PC sends  UP               → arm rises back to SAFE_Z
  7. Repeat steps 1–6 for destination square but with RELEASE instead of GRAB
  8. PC sends  HOME             → arm returns home

────────────────────────────────────────────────────────────────────────────────
  ██████╗  ██████╗  █████╗ ██████╗ ██████╗     ██╗  ██╗███████╗██████╗ ███████╗
  ██╔══██╗██╔═══██╗██╔══██╗██╔══██╗██╔══██╗    ██║  ██║██╔════╝██╔══██╗██╔════╝
  ██████╔╝██║   ██║███████║██████╔╝██║  ██║    ███████║█████╗  ██████╔╝█████╗
  ██╔══██╗██║   ██║██╔══██║██╔══██╗██║  ██║    ██╔══██║██╔══╝  ██╔══██╗██╔══╝
  ██████╔╝╚██████╔╝██║  ██║██║  ██║██████╔╝    ██║  ██║███████╗██║  ██║███████╗
  ╚═════╝  ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝     ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚══════╝
────────────────────────────────────────────────────────────────────────────────
"""

from machine import Pin
import math
import utime
import sys

# ==================== PIN DEFINES ====================

MOTOR1_CLK = 10
MOTOR1_CW  = 11
MOTOR2_CLK = 13
MOTOR2_CW  = 12
MOTOR3_CLK = 14
MOTOR3_CW  = 15

CLK_PINS = [MOTOR1_CLK, MOTOR2_CLK, MOTOR3_CLK]
CW_PINS  = [MOTOR1_CW,  MOTOR2_CW,  MOTOR3_CW ]

step_pins = [Pin(p, Pin.OUT) for p in CLK_PINS]
dir_pins  = [Pin(p, Pin.OUT) for p in CW_PINS ]

# Gripper stepper (4-wire half-step)
gripper_pins = [Pin(6, Pin.OUT), Pin(7, Pin.OUT), Pin(8, Pin.OUT), Pin(9, Pin.OUT)]
gripper_sequence = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1]
]

# ==================== ARM GEOMETRY ====================

YOUPI_L1      = 162.0
YOUPI_L2      = 162.0
SHOULDER_H    = 80.0
MAX_ARM_RANGE = YOUPI_L1 + YOUPI_L2

T1_MIN = -math.pi / 2.0
T1_MAX =  math.pi / 2.0
T2_MIN = -math.pi
T2_MAX =  2.0 * math.pi
T3_MIN = -math.pi
T3_MAX =  math.pi

# ==================== PICK & PLACE CONFIG ====================

SAFE_Z        = 180
PICK_Z        = 35
GRIP_DELAY_MS = 500      # reduced — vision controls timing now

# ==================== STEPS PER REVOLUTION ====================

STEPS_PER_REV = [11800, 11800, 12600]
STEP_DELAY_US = 950

# ==================== CHESS BOARD GEOMETRY ====================
# These define where the chessboard is in the robot's coordinate frame.
#
# The robot origin is at its shoulder/base pivot.
# BOARD_ORIGIN_X/Y = robot XY (mm) of the CENTER of square A1.
#
# HOW TO CALIBRATE:
#   1. Manually jog the arm to hover over the center of square A1.
#   2. Note the X, Y values — put them below.
#   3. Do the same for H8. The difference / 7 should equal SQUARE_SIZE_MM.
#
# AXIS ORIENTATION (adjust FLIP_X / FLIP_Y if your board is mirrored):
#   Files A→H  increase along the +X axis of the robot by default
#   Ranks 1→8  increase along the +Y axis of the robot by default

BOARD_ORIGIN_X  = 80.0    # ← X of A1 center in robot frame (mm)  — TUNE THIS
BOARD_ORIGIN_Y  = 50.0    # ← Y of A1 center in robot frame (mm)  — TUNE THIS
SQUARE_SIZE_MM  = 50.0    # standard chess square (mm) — measure yours
FLIP_X          = False   # set True if files go A→H in −X direction
FLIP_Y          = False   # set True if ranks go 1→8 in −Y direction

CHESS_FILES = "ABCDEFGH"

def square_to_xy(square: str):
    """
    Convert chess notation (e.g. "E4") to robot XY coordinates (mm).
    Returns (x, y) or raises ValueError for invalid input.
    """
    square = square.strip().upper()
    if len(square) != 2 or square[0] not in CHESS_FILES or square[1] not in "12345678":
        raise ValueError(f"Invalid square: '{square}'")

    col = CHESS_FILES.index(square[0])   # A=0 … H=7
    row = int(square[1]) - 1             # 1=0 … 8=7

    x = BOARD_ORIGIN_X + ((-1 if FLIP_X else 1) * col * SQUARE_SIZE_MM)
    y = BOARD_ORIGIN_Y + ((-1 if FLIP_Y else 1) * row * SQUARE_SIZE_MM)
    return x, y

# ==================== GLOBALS ====================

current_steps = [0, 0, 0]
current_phase = 0           # gripper stepper phase

home_angles = None
home_saved  = False
home_x = home_y = home_z = 0.0
current_x = current_y = current_z = 0.0

# ==================== HELPERS ====================

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def sign_f(v):
    return 1 if v >= 0.0 else -1

def angle_to_steps(angle_rad, motor):
    return int((angle_rad / (2.0 * math.pi)) * STEPS_PER_REV[motor])

def reply_ok(msg=""):
    out = "OK:" + msg if msg else "OK"
    print(out)

def reply_err(msg):
    print("ERR:" + msg)

# ==================== STEP ====================

def step_once(motor, direction):
    dir_pins[motor].value(1 if direction > 0 else 0)
    utime.sleep_us(2)
    step_pins[motor].value(1)
    utime.sleep_us(10)
    step_pins[motor].value(0)

# ==================== MOTION ====================

def move_to_angles(t1, t2, t3, delay_us=STEP_DELAY_US):
    global current_steps

    target = [angle_to_steps(t1, 0), angle_to_steps(t2, 1), angle_to_steps(t3, 2)]
    delta  = [target[i] - current_steps[i] for i in range(3)]
    dirs   = [1 if d >= 0 else -1 for d in delta]
    absd   = [abs(d) for d in delta]
    max_s  = max(absd)
    acc    = [0, 0, 0]

    if max_s == 0:
        return   # already there — silent in vision mode

    for _ in range(max_s):
        for i in range(3):
            acc[i] += absd[i]
            if acc[i] >= max_s:
                acc[i] -= max_s
                step_once(i, dirs[i])
                current_steps[i] += dirs[i]
        utime.sleep_us(delay_us)

# ==================== INVERSE KINEMATICS ====================

def solve_ik(x, y, z, ch="home"):
    horiz    = math.sqrt(x*x + y*y)
    z4       = z - SHOULDER_H
    sphere_r = math.sqrt(horiz*horiz + z4*z4)

    if sphere_r > MAX_ARM_RANGE:
        return None, f"Too far (r={sphere_r:.1f} max={MAX_ARM_RANGE})"
    if sphere_r < 1e-3:
        return None, "Target at shoulder pivot"

    alpha_num = horiz*horiz + z4*z4 + YOUPI_L1*YOUPI_L1 - YOUPI_L2*YOUPI_L2
    acos_arg  = alpha_num / (2.0 * YOUPI_L1 * sphere_r)

    if acos_arg < -1.0 or acos_arg > 1.0:
        return None, "acos out of domain"

    beta  = math.atan2(z4, horiz)
    alpha = math.acos(acos_arg)
    t1    = (math.pi / 2.0) * sign_f(y) if x == 0.0 else math.atan2(y, x)

    def try_sol(t2c):
        elbow_x = YOUPI_L1 * math.cos(t2c)
        elbow_z = YOUPI_L1 * math.sin(t2c)
        t3c     = math.atan2(z4 - elbow_z, horiz - elbow_x)
        if (T1_MIN <= t1 <= T1_MAX and
                T2_MIN <= t2c <= T2_MAX and
                T3_MIN <= t3c <= T3_MAX):
            return t1, t2c, t3c
        return None

    candidates = ([(beta + alpha, "elbow-down"), (beta + alpha, "elbow-up")]
                  if ch == "move"
                  else [(beta + alpha, "elbow-up"), (beta - alpha, "elbow-down")])

    for t2c, label in candidates:
        sol = try_sol(t2c)
        if sol:
            return sol, None

    return None, "Joint limits violated"

# ==================== MOVE XYZ ====================

def move_xyz(x, y, z, ch="home"):
    global current_x, current_y, current_z
    sol, err = solve_ik(x, y, z, ch)
    if sol is None:
        return False, err
    move_to_angles(*sol)
    current_x, current_y, current_z = x, y, z
    return True, None

# ==================== HOME ====================

def save_home(x, y, z):
    global home_angles, home_saved, home_x, home_y, home_z
    global current_x, current_y, current_z

    sol, err = solve_ik(x, y, z, ch="home")
    if sol is None:
        print(f"[HOME ERROR] {err}")
        return False

    home_x, home_y, home_z    = x, y, z
    current_x, current_y, current_z = x, y, z
    home_angles = sol
    home_saved  = True
    return True

def return_home():
    global current_x, current_y, current_z
    if not home_saved:
        return False, "No home saved"
    move_to_angles(*home_angles)
    current_x, current_y, current_z = home_x, home_y, home_z
    return True, None

# ==================== GRIPPER ====================

def gripper_move(steps):
    """Positive steps = close/grip, negative = open/release."""
    global current_phase
    direction = 1 if steps > 0 else -1
    for _ in range(abs(steps)):
        current_phase = (current_phase + direction) % 4
        for i in range(4):
            gripper_pins[i].value(gripper_sequence[current_phase][i])
        utime.sleep_ms(5)
    for p in gripper_pins:
        p.value(0)

GRIPPER_STEPS = 1024    # steps for full open/close travel

# ==================== VISION COMMAND HANDLERS ====================

def cmd_move(square):
    """
    MOVE,E4  →  fly to above that square at SAFE_Z.
    This is the open-loop coarse move. Vision corrects afterward with COR.
    """
    try:
        x, y = square_to_xy(square)
    except ValueError as e:
        reply_err(str(e))
        return

    ok, err = move_xyz(x, y, SAFE_Z, ch="home")
    if ok:
        reply_ok(f"above {square} X={x:.1f} Y={y:.1f} Z={SAFE_Z}")
    else:
        reply_err(f"IK failed for {square}: {err}")


def cmd_correction(dx_mm, dy_mm):
    """
    COR,dx,dy  →  small XY correction in mm.

    KEY INSIGHT: We don't need STEPS_PER_MM here.
    We just add the correction to current XY and re-run the IK.
    Your existing IK is perfectly accurate for small deltas.
    The arm stays at the same Z height.
    """
    new_x = current_x + dx_mm
    new_y = current_y + dy_mm
    ok, err = move_xyz(new_x, new_y, current_z, ch="move")
    if ok:
        reply_ok(f"corrected to X={new_x:.1f} Y={new_y:.1f}")
    else:
        reply_err(f"correction IK failed: {err}")


def cmd_down():
    """DOWN  →  descend to PICK_Z at current XY."""
    ok, err = move_xyz(current_x, current_y, PICK_Z, ch="move")
    if ok:
        reply_ok(f"descended to Z={PICK_Z}")
    else:
        reply_err(f"descent IK failed: {err}")


def cmd_up():
    """UP  →  rise to SAFE_Z at current XY."""
    ok, err = move_xyz(current_x, current_y, SAFE_Z, ch="home")
    if ok:
        reply_ok(f"risen to Z={SAFE_Z}")
    else:
        reply_err(f"rise IK failed: {err}")


def cmd_grab():
    """GRAB  →  close gripper."""
    gripper_move(GRIPPER_STEPS)
    utime.sleep_ms(GRIP_DELAY_MS)
    reply_ok("gripper closed")


def cmd_release():
    """RELEASE  →  open gripper."""
    gripper_move(-GRIPPER_STEPS)
    utime.sleep_ms(GRIP_DELAY_MS)
    reply_ok("gripper opened")


def cmd_home():
    """HOME  →  return to saved home position."""
    ok, err = return_home()
    if ok:
        reply_ok("home reached")
    else:
        reply_err(err)


def cmd_pos():
    """POS  →  report current position (for debugging)."""
    reply_ok(f"X={current_x:.1f} Y={current_y:.1f} Z={current_z:.1f}")


# ==================== LEGACY FULL PICK SEQUENCE ====================
# Kept for manual testing without vision.
# PICK,E2,E4  →  full pick & place from square E2 to E4 with no vision feedback.

def cmd_pick_legacy(src_sq, dst_sq):
    try:
        px, py = square_to_xy(src_sq)
        lx, ly = square_to_xy(dst_sq)
    except ValueError as e:
        reply_err(str(e))
        return

    print(f"[PICK] {src_sq}({px},{py}) → {dst_sq}({lx},{ly})")

    def safe_move(x, y, z, ch, step_label):
        ok, err = move_xyz(x, y, z, ch)
        if not ok:
            print(f"[PICK ABORT] Step {step_label}: {err}")
            return_home()
            return False
        return True

    # 1. Fly to above pick square
    if not safe_move(px, py, SAFE_Z, "home", "1-fly-to-pick"):   return
    # 2. Descend
    if not safe_move(px, py, PICK_Z, "move", "2-descend-pick"):  return
    # 3. Grip
    gripper_move(GRIPPER_STEPS)
    utime.sleep_ms(GRIP_DELAY_MS)
    # 4. Rise
    if not safe_move(px, py, SAFE_Z, "home", "4-rise-pick"):     return
    # 5. Fly to above place square
    if not safe_move(lx, ly, SAFE_Z, "home", "5-fly-to-place"):  return
    # 6. Descend
    if not safe_move(lx, ly, PICK_Z + 30, "move", "6-descend-place"): return
    # 7. Release
    gripper_move(-GRIPPER_STEPS)
    utime.sleep_ms(GRIP_DELAY_MS)
    # 8. Rise and home
    move_xyz(lx, ly, SAFE_Z, "home")
    return_home()

    reply_ok(f"pick&place {src_sq}→{dst_sq} complete")


# ==================== COMMAND DISPATCHER ====================

def dispatch(line: str):
    """Parse and execute one command line."""
    line   = line.strip()
    if not line:
        return

    parts  = line.split(",")
    cmd    = parts[0].upper()

    try:
        if cmd == "MOVE" and len(parts) >= 2:
            cmd_move(parts[1].strip())

        elif cmd == "COR" and len(parts) >= 3:
            dx = float(parts[1])
            dy = float(parts[2])
            cmd_correction(dx, dy)

        elif cmd == "DOWN":
            cmd_down()

        elif cmd == "UP":
            cmd_up()

        elif cmd == "GRAB":
            cmd_grab()

        elif cmd == "RELEASE":
            cmd_release()

        elif cmd == "HOME":
            cmd_home()

        elif cmd == "POS":
            cmd_pos()

        elif cmd == "PICK" and len(parts) >= 3:
            cmd_pick_legacy(parts[1].strip(), parts[2].strip())

        else:
            reply_err(f"Unknown command: '{line}'")
            print("  Valid: MOVE,E4 | COR,dx,dy | DOWN | UP | GRAB | RELEASE | HOME | POS | PICK,E2,E4")

    except (ValueError, IndexError) as e:
        reply_err(f"Parse error: {e}")
    except Exception as e:
        reply_err(f"Runtime error: {e}")


# ==================== SETUP ====================

for p in step_pins:    p.value(0)
for p in dir_pins:     p.value(0)
for p in gripper_pins: p.value(0)

utime.sleep_ms(1000)

# Save and move to home position
if save_home(80.0, 0.0, SAFE_Z):
    print("[STARTUP] Moving to home position...")
    move_to_angles(*home_angles)
    print("[STARTUP] Home reached.")
else:
    print("[STARTUP ERROR] Home position not reachable — check geometry constants!")

print("=" * 50)
print("  YOUPI ARM — VISION MODE READY")
print(f"  Steps/rev : M1={STEPS_PER_REV[0]}  M2={STEPS_PER_REV[1]}  M3={STEPS_PER_REV[2]}")
print(f"  safe_z    : {SAFE_Z} mm    pick_z: {PICK_Z} mm")
print(f"  Board A1  : X={BOARD_ORIGIN_X}  Y={BOARD_ORIGIN_Y}  sq={SQUARE_SIZE_MM}mm")
print("  Protocol  : MOVE,E4 | COR,dx,dy | DOWN | UP | GRAB | RELEASE | HOME | POS")
print("=" * 50)

# ==================== MAIN LOOP ====================

buf = ""

while True:
    try:
        # Read character by character — compatible with both
        # blocking USB serial (Thonny) and pyserial from PC vision script
        c = sys.stdin.read(1)

        if c in ('\n', '\r'):
            dispatch(buf)
            buf = ""
        else:
            buf += c

    except Exception as e:
        print(f"[LOOP ERROR] {e}")
        buf = ""
