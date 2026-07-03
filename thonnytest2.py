from machine import Pin
import math
import utime

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

# ==================== ARM GEOMETRY ====================

YOUPI_L1    = 162.0
YOUPI_L2    = 162.0
SHOULDER_H  = 80.0
MAX_ARM_RANGE = YOUPI_L1 + YOUPI_L2

T1_MIN = -math.pi / 2.0
T1_MAX =  math.pi / 2.0
T2_MIN =  0.0
T2_MAX =  2.0 * math.pi
T3_MIN = -math.pi
T3_MAX =  math.pi

# ==================== PICK & PLACE CONFIG ====================

SAFE_Z       = 40.0    # travel height (mm)
PICK_Z       = 40.0    # picking height (mm)
GRIP_DELAY_MS = 4000   # gripper hold time (ms)

# ==================== STEPS PER REVOLUTION ====================

STEPS_PER_REV = [11800, 11800, 12600]  # M1, M2, M3
STEP_DELAY_US = 2000

# ==================== WAYPOINT LOG (RAM) ====================
# Replaces EEPROM — stored in a list of (t1, t2, t3) tuples

MAX_WAYPOINTS = 100
waypoint_log  = []

def waypoint_push(t1, t2, t3):
    if len(waypoint_log) >= MAX_WAYPOINTS:
        print("[WARN] Waypoint log full — send 'home' to clear.")
        return
    waypoint_log.append((t1, t2, t3))

def waypoint_clear():
    waypoint_log.clear()

# ==================== GLOBALS ====================

current_steps = [0, 0, 0]

home_angles = None   # (t1, t2, t3)
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

    for _ in range(max_s):
        for i in range(3):
            acc[i] += absd[i]
            if acc[i] >= max_s:
                acc[i] -= max_s
                step_once(i, dirs[i])
                current_steps[i] += dirs[i]
        utime.sleep_us(delay_us)

# ==================== INVERSE KINEMATICS ====================

def solve_ik(x, y, z):
    print("----------------------------------")
    print(f"[IK] Solving for X={x} Y={y} Z={z}")

    horiz    = math.sqrt(x*x + y*y)
    z4       = z - SHOULDER_H
    sphere_r = math.sqrt(horiz*horiz + z4*z4)

    if sphere_r > MAX_ARM_RANGE:
        print(f"[IK ERROR] Point too far! sphere_r={sphere_r:.2f} MAX={MAX_ARM_RANGE}")
        return None
    if sphere_r < 1e-3:
        print("[IK ERROR] Target coincides with shoulder pivot!")
        return None

    alpha_num = horiz*horiz + z4*z4 + YOUPI_L1*YOUPI_L1 - YOUPI_L2*YOUPI_L2
    acos_arg  = alpha_num / (2.0 * YOUPI_L1 * sphere_r)

    if acos_arg < -1.0 or acos_arg > 1.0:
        print("[IK ERROR] acos out of domain — point unreachable!")
        return None

    beta = math.atan2(z4, horiz)

    t1 = (math.pi / 2.0) * sign_f(y) if x == 0.0 else math.atan2(y, x)
    t2 = math.acos(acos_arg) + beta

    elbow_arg = clamp((horiz - YOUPI_L1 * math.cos(t2)) / YOUPI_L2, -1.0, 1.0)
    t3 = math.acos(elbow_arg) * sign_f(z4)

    print(f"[IK] t1={math.degrees(t1):.2f}  t2={math.degrees(t2):.2f}  t3={math.degrees(t3):.2f}")

    ok = True
    if not (T1_MIN <= t1 <= T1_MAX): print("[LIMIT ERROR] t1 out of range"); ok = False
    if not (T2_MIN <= t2 <= T2_MAX): print("[LIMIT ERROR] t2 out of range"); ok = False
    if not (T3_MIN <= t3 <= T3_MAX): print("[LIMIT ERROR] t3 out of range"); ok = False

    if not ok:
        print("[IK ERROR] Joint limit violated — move aborted")
        return None

    print("[IK OK] All joints within limits")
    return t1, t2, t3

# ==================== MOVE XYZ ====================

def move_xyz(x, y, z):
    global current_x, current_y, current_z
    res = solve_ik(x, y, z)
    if res is None:
        return False
    t1, t2, t3 = res
    waypoint_push(t1, t2, t3)
    move_to_angles(t1, t2, t3)
    current_x, current_y, current_z = x, y, z
    return True

# ==================== HOME ====================

def save_home(x, y, z):
    global home_angles, home_saved, home_x, home_y, home_z
    global current_x, current_y, current_z

    res = solve_ik(x, y, z)
    if res is None:
        print("[HOME ERROR] Position not reachable!")
        return

    home_x, home_y, home_z = x, y, z
    current_x, current_y, current_z = x, y, z
    home_angles = res
    home_saved  = True
    waypoint_clear()

    print("[HOME SAVED]")
    print(f"  X={x} Y={y} Z={z}")
    print(f"  Max moves before home required: {MAX_WAYPOINTS}")

def return_home():
    global current_x, current_y, current_z

    if not home_saved:
        print("[HOME ERROR] No home saved!")
        return
    if len(waypoint_log) == 0:
        print("[HOME] Already at home.")
        return

    print(f"[RETURNING HOME] Retracing {len(waypoint_log)} waypoints...")

    # Retrace in reverse, then go to home_angles as final step
    for i in range(len(waypoint_log) - 1, -2, -1):
        if i == -1:
            t1, t2, t3 = home_angles
            print("[HOME] Final step — home angles")
        else:
            t1, t2, t3 = waypoint_log[i]
            print(f"[HOME] Step to waypoint {i}")
        move_to_angles(t1, t2, t3)

    current_x, current_y, current_z = home_x, home_y, home_z
    waypoint_clear()
    print("[HOME REACHED]")

# ==================== PICK SEQUENCE ====================

def pick_sequence(x, y):
    print("==================================")
    print("[PICK] Starting pick sequence")
    print(f"  Target    : X={x}  Y={y}")
    print(f"  safe_z    : {SAFE_Z}")
    print(f"  pick_z    : {PICK_Z}")
    print(f"  grip delay: {GRIP_DELAY_MS} ms")

    # 1. Fly to XY at safe height
    print("[PICK] 1/6 — moving to safe height above target")
    if not move_xyz(x, y, SAFE_Z):
        print("[PICK ERROR] Cannot reach safe position — aborted")
        return

    # 2. Descend to pick height
    print("[PICK] 2/6 — descending to pick_z")
    if not move_xyz(x, y, PICK_Z):
        print("[PICK ERROR] Cannot reach pick position — aborted")
        return

    # 3. Grip delay
    print(f"[PICK] 3/6 — gripping ({GRIP_DELAY_MS} ms) ...")
    utime.sleep_ms(GRIP_DELAY_MS)

    # 4. Rise to safe height
    print("[PICK] 4/6 — rising to safe_z")
    if not move_xyz(x, y, SAFE_Z):
        print("[PICK ERROR] Cannot rise — aborted")
        return

    # 5. Return to home
    print("[PICK] 5/6 — returning to home")
    return_home()

    # 6. Parking position (home but base rotated +90 deg)
    print("[PICK] 6/6 — moving to parking position (base +90 deg)")
    t1, t2, t3 = home_angles
    park_t1 = min(t1 + math.pi / 2.0, T1_MAX)
    move_to_angles(park_t1, t2, t3)

    print("==================================")
    print("[DONE]")
    print("  Send next X Y to pick again.")
    print("  Send 'home' to reset to start position.")
    print("==================================")

# ==================== SETUP ====================

for p in step_pins: p.value(0)
for p in dir_pins:  p.value(0)

utime.sleep_ms(1000)

# Physically place the arm at X=80, Y=0, Z=SAFE_Z before powering on.
save_home(80.0, 0.0, SAFE_Z)

print("==================================")
print("  YOUPI ARM — READY")
print(f"  Max moves   : {MAX_WAYPOINTS}")
print(f"  Steps/rev   : M1={STEPS_PER_REV[0]}  M2={STEPS_PER_REV[1]}  M3={STEPS_PER_REV[2]}")
print(f"  safe_z      : {SAFE_Z} mm")
print(f"  pick_z      : {PICK_Z} mm")
print(f"  grip delay  : {GRIP_DELAY_MS} ms")
print("----------------------------------")
print("  Send: X Y   — full pick sequence")
print("  Send: home  — retrace path back to start")
print("  Send: pos   — show current position")
print("==================================")

# ==================== MAIN LOOP ====================

while True:
    try:
        line = input().strip().lower()

        # ---- home ----
        if line == "home":
            return_home()
            print("----------------------------------")
            print("Ready — send X Y")
            continue

        # ---- pos ----
        if line == "pos":
            print("----------------------------------")
            print(f"[POS] X={current_x}  Y={current_y}  Z={current_z}")
            print("----------------------------------")
            continue

        # ---- X Y input ----
        parts = line.split()

        if len(parts) != 2:
            print("[INPUT ERROR] Format: X Y  (e.g. 120 50)")
            continue

        tx = float(parts[0])
        ty = float(parts[1])

        print(f"[INPUT] Pick at X={tx}  Y={ty}")
        pick_sequence(tx, ty)

        print("----------------------------------")
        print("Ready — send X Y")

    except Exception as e:
        print(f"[ERROR] {e}")

