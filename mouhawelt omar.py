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

pins = [Pin(6, Pin.OUT), Pin(7, Pin.OUT), Pin(8, Pin.OUT), Pin(9, Pin.OUT)]
sequence = [
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
PICK_Z        = 49
GRIP_DELAY_MS = 1000

# ==================== STEPS PER REVOLUTION ====================

STEPS_PER_REV = [11800, 11800, 12600]
STEP_DELAY_US = 950

# ==================== GLOBALS ====================

current_steps = [0, 0, 0]
current_phase = 0

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
        print("[MOVE] Already at target angles — no steps needed.")
        return

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
    print("----------------------------------")
    print(f"[IK] Solving for X={x} Y={y} Z={z}  mode={ch}")

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

    beta  = math.atan2(z4, horiz)
    alpha = math.acos(acos_arg)
    t1    = (math.pi / 2.0) * sign_f(y) if x == 0.0 else math.atan2(y, x)

    def try_solution(t2_candidate, label):
        elbow_x      = YOUPI_L1 * math.cos(t2_candidate)
        elbow_z      = YOUPI_L1 * math.sin(t2_candidate)
        dx           = horiz - elbow_x
        dz           = z4    - elbow_z
        t3_candidate = math.atan2(dz, dx)

        in_t1 = T1_MIN <= t1            <= T1_MAX
        in_t2 = T2_MIN <= t2_candidate  <= T2_MAX
        in_t3 = T3_MIN <= t3_candidate  <= T3_MAX

        status = "OK" if (in_t1 and in_t2 and in_t3) else "FAIL"
        print(f"[IK {label}] t1={math.degrees(t1):.2f}  "
              f"t2={math.degrees(t2_candidate):.2f}  "
              f"t3={math.degrees(t3_candidate):.2f}  "
              f"limits={status}")

        if in_t1 and in_t2 and in_t3:
            return t1, t2_candidate, t3_candidate
        return None

    if ch == "move":
        order = [(beta + alpha, "elbow-down"), (beta + alpha, "elbow-up")]
    else:
        order = [(beta + alpha, "elbow-up"), (beta - alpha, "elbow-down")]

    for t2_candidate, label in order:
        res = try_solution(t2_candidate, label)
        if res:
            print(f"[IK OK] Using {label} solution")
            return res

    print("[IK ERROR] Both solutions violated joint limits — move aborted")
    return None

# ==================== MOVE XYZ ====================

def move_xyz(x, y, z, ch="home"):
    global current_x, current_y, current_z
    res = solve_ik(x, y, z, ch)
    if res is None:
        return False
    t1, t2, t3 = res
    move_to_angles(t1, t2, t3)
    current_x, current_y, current_z = x, y, z
    return True

# ==================== HOME ====================

def save_home(x, y, z):
    global home_angles, home_saved, home_x, home_y, home_z
    global current_x, current_y, current_z

    res = solve_ik(x, y, z, ch="home")
    if res is None:
        print("[HOME ERROR] Position not reachable!")
        return

    home_x, home_y, home_z = x, y, z
    current_x, current_y, current_z = x, y, z
    home_angles = res
    home_saved  = True

    print("[HOME SAVED]")
    print(f"  X={x} Y={y} Z={z}")

def return_home():
    global current_x, current_y, current_z

    if not home_saved:
        print("[HOME ERROR] No home saved!")
        return

    print("[RETURNING HOME]...")
    t1, t2, t3 = home_angles
    move_to_angles(t1, t2, t3)
    current_x, current_y, current_z = home_x, home_y, home_z
    print("[HOME REACHED]")

# ==================== STEPPER MOTOR ====================

def step_motor(steps):
    global current_phase
    direction = 1 if steps > 0 else -1
    steps = abs(steps)

    for _ in range(steps):
        current_phase = (current_phase + direction) % 4
        for i in range(4):
            pins[i].value(sequence[current_phase][i])
        utime.sleep_ms(5)

    for p in pins: p.value(0)

# ==================== PICK SEQUENCE ====================

def pick_sequence(pick_x, pick_y, place_x, place_y):
    print("==================================")
    print("[PICK] Starting pick & place sequence")
    print(f"  Pick      : X={pick_x}  Y={pick_y}")
    print(f"  Place     : X={place_x}  Y={place_y}")
    print(f"  safe_z    : {SAFE_Z}")
    print(f"  pick_z    : {PICK_Z}")
    print(f"  grip delay: {GRIP_DELAY_MS} ms")

    # 1. Fly to pick XY at safe height
    print("[PICK] 1/7 — moving to safe height above pick target")
    if not move_xyz(pick_x, pick_y, SAFE_Z, ch="home"):
        print("[PICK ERROR] Cannot reach safe position above pick — aborted")
        # Rotate back +90° to parking before aborting
        print("[ABORT] Rotating +90° back to parking position...")
        step_motor(1024)
        return

    # 2. Descend to pick height
    print("[PICK] 2/7 — descending to pick_z")
    if not move_xyz(pick_x, pick_y, PICK_Z, ch="move"):
        print("[PICK ERROR] Cannot reach pick position — aborted")
        move_xyz(pick_x, pick_y, SAFE_Z, ch="home")
        return_home()
        print("[ABORT] Rotating +90° back to parking position...")
        step_motor(1024)
        return

    # 3. Grip
    print(f"[PICK] 3/7 — gripping ({GRIP_DELAY_MS} ms) ...")
    utime.sleep_ms(GRIP_DELAY_MS)
    step_motor(512)

    # 4. Rise back to safe height
    print("[PICK] 4/7 — rising to safe_z")
    if not move_xyz(pick_x, pick_y, SAFE_Z, ch="home"):
        print("[PICK ERROR] Cannot rise after pick — aborted")
        return_home()
        print("[ABORT] Rotating +90° back to parking position...")
        step_motor(1024)
        return

    # 5. Fly to place XY at safe height
    print("[PICK] 5/7 — moving to safe height above place target")
    if not move_xyz(place_x, place_y, SAFE_Z, ch="home"):
        print("[PICK ERROR] Cannot reach safe position above place — aborted")
        return_home()
        print("[ABORT] Rotating +90° back to parking position...")
        step_motor(1024)
        return

    # 6. Descend and release
    print("[PICK] 6/7 — descending to place_z and releasing")
    if not move_xyz(place_x, place_y, PICK_Z + 30, ch="move"):
        print("[PICK ERROR] Cannot reach place position — aborted")
        return_home()
        print("[ABORT] Rotating +90° back to parking position...")
        step_motor(1024)
        return
    step_motor(-1024)
    utime.sleep_ms(GRIP_DELAY_MS)

    # 7. Rise and return home
    print("[PICK] 7/7 — rising and returning home")
    move_xyz(place_x, place_y, SAFE_Z, ch="home")
    return_home()

    # Rotate +90° back to parking position — facing away from chessboard
    print("[DONE] Rotating +90° back to parking position...")
    park_t1 = min(t1 + math.pi / 6.0, T1_MAX)
    move_to_angles(park_t1, t2, t3)    
    print("==================================")
    print("[DONE] Cycle complete — parked and waiting.")
    print("==================================")

# ==================== SETUP ====================

for p in step_pins: p.value(0)
for p in dir_pins:  p.value(0)

utime.sleep_ms(1000)

save_home(80.0, 0.0, SAFE_Z)

# Move physically to home on startup
print("[STARTUP] Moving to home position...")
t1, t2, t3 = home_angles
move_to_angles(t1, t2, t3)
print("[STARTUP] Home position reached.")

print("==================================")
print("  YOUPI ARM — READY")
print(f"  Steps/rev   : M1={STEPS_PER_REV[0]}  M2={STEPS_PER_REV[1]}  M3={STEPS_PER_REV[2]}")
print(f"  safe_z      : {SAFE_Z} mm")
print(f"  pick_z      : {PICK_Z} mm")
print(f"  grip delay  : {GRIP_DELAY_MS} ms")
print("----------------------------------")
print("  Send: Xpick Ypick Xplace Yplace — full pick & place cycle")
print("  Send: home                      — return to start")
print("  Send: pos                       — show current position")
print("==================================")

# ==================== MAIN LOOP ====================

while True:
    try:
        line = input(">> ").strip().lower()

        if line == "home":
            return_home()
            print("----------------------------------")
            print("Ready — send Xpick Ypick Xplace Yplace")
            continue

        if line == "pos":
            print("----------------------------------")
            print(f"[POS] X={current_x}  Y={current_y}  Z={current_z}")
            print("----------------------------------")
            continue

        parts = line.split()

        if len(parts) != 4:
            print("[INPUT ERROR] Format: Xpick Ypick Xplace Yplace  (e.g. 80 50 120 -30)")
            continue

        px, py, lx, ly = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])



        print(f"[INPUT] Pick at X={px} Y={py} — Place at X={lx} Y={ly}")
        pick_sequence(px, py, lx, ly)

        print("----------------------------------")
        print("Ready — send Xpick Ypick Xplace Yplace")

    except Exception as e:
        print(f"[ERROR] {e}")