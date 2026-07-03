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

step_pins = [Pin(p, Pin.OUT) for p in CLK_PINS]  #configuration des clkpins en sorties 
dir_pins  = [Pin(p, Pin.OUT) for p in CW_PINS ]  #configuration des CWpins en sorties (1 ou -1)

# ==================== ARM GEOMETRY ====================

YOUPI_L1      = 162.0
YOUPI_L2      = 162.0
SHOULDER_H    = 80.0
MAX_ARM_RANGE = YOUPI_L1 + YOUPI_L2



T1_MIN = -math.pi / 2.0   # -90° < t1 < 90°
T1_MAX =  math.pi / 2.0
T2_MIN = -math.pi     # -45° < t2 < 90°
T2_MAX =  math.pi * 2.0
T3_MIN = -math.pi    # -90° < t3 < 90°
T3_MAX =  math.pi 

# ==================== PICK & PLACE CONFIG ====================

SAFE_Z        = 180    # travel height (mm)
PICK_Z        = 25    # picking height (mm)    dima position mtaa gripper 15 mm % base
GRIP_DELAY_MS = 1000  # gripper hold time (ms)

# ==================== STEPS PER REVOLUTION ====================

STEPS_PER_REV = [5900, 3150, 3250]   # M1(1/2 tour"180°"), M2(1/4 tour"90°"), M3(1/4 tour "90°")
STEP_DELAY_US = 2000

# ==================== WAYPOINT LOG (RAM) ====================

MAX_WAYPOINTS = 100   #retenir 100 positions successives (securité lel RAM mta l pico)
waypoint_log  = []    #liste ou laquelle on va stocker les parametres t1, t2, t3 de chaque position

def waypoint_push(t1, t2, t3):    #nakraw f les positions w nhottou fel parametres fel liste 
    if len(waypoint_log) >= MAX_WAYPOINTS:     
        print("[WARN] Waypoint log full — send 'home' to clear.")   
        return
    waypoint_log.append((t1, t2, t3))

def waypoint_clear():
    waypoint_log.clear()

# ==================== GLOBALS ==================== (initialisation)

current_steps = [0, 0, 0]   #position de chaque moteur en nombre de steps

home_angles = None  #angles de la position home related bel save-home()
home_saved  = False
home_x = home_y = home_z = 0.0
current_x = current_y = current_z = 0.0

# ==================== HELPERS ====================

def clamp(v, lo, hi):    #protection contre le depassement des limites 
    return lo if v < lo else hi if v > hi else v

def sign_f(v):      #donner les signes (monter/descendre / droite /gauche) (faciliter le travail f 3ousdh mannaa3mlou if w elif w else )
    return 1 if v >= 0.0 else -1

def angle_to_steps(angle_rad, motor):
    return int((angle_rad / (2.0 * math.pi)) * STEPS_PER_REV[motor])

# ==================== STEP ====================

def step_once(motor, direction):
    dir_pins[motor].value(1 if direction > 0 else 0) #definition de sens de rotation 
    utime.sleep_us(2)   #ettente pour que le moteur ne rate pas les premiers steps lors du changemlent de direction
    
    #Génération de l'impulsion de mouvement.
    step_pins[motor].value(1)  #passage de 0v at 5V sur la broche clk  (detection d un front montant )
    utime.sleep_us(10)         
    step_pins[motor].value(0)   #passage de 5v at 0V sur la broche clk  

# ==================== MOTION ====================
   #synchronisation entre les moteurs pour qu'ils start et finissent au meme temps( le moteur ayant le ,nbre de steps le plus grands a executer
#va guider le mouvement(ken M1 lezmou yaamel 100 steps w M2 50 steps el M2 chyaaml steps e yerteh(vitesse M2<vitesseM1)bech ykamlou fard wakt )
def move_to_angles(t1, t2, t3, delay_us=STEP_DELAY_US):   #. L'algorithme de Bresenham (L'interpolation)
    global current_steps

    target = [angle_to_steps(t1, 0), angle_to_steps(t2, 1), angle_to_steps(t3, 2)]
    delta  = [target[i] - current_steps[i] for i in range(3)]  #difference entre target w pos actuelle
    dirs   = [1 if d >= 0 else -1 for d in delta] #ntalou e direction elli chnemchiw feha selon e delta
    absd   = [abs(d) for d in delta]
    max_s  = max(absd)  #ntalou el moteur elli andou akther steps bech yaamalhom (elli ch yguidi el mouvements)
    acc    = [0, 0, 0]

    if max_s == 0:   # deja wselna le target 
        print("[MOVE] Already at target angles — no steps needed.")
        return

    for _ in range(max_s):
        for i in range(3): # accumulation lkol moteur 
            acc[i] += absd[i]
            if acc[i] >= max_s:# decision (yaaml step tawa walle (selon l calcul mtaa l vitesse))
                acc[i] -= max_s #ynakasha mel total khater deja chyaaml step
                step_once(i, dirs[i])  #execution (yaaml e step heki)
                current_steps[i] += dirs[i]
        utime.sleep_us(delay_us)

# ==================== INVERSE KINEMATICS ====================
#
#   ch = "move" → picking move: try elbow-down first, fallback elbow-up
#   ch = "home" → homing/safe moves: try elbow-up first, fallback elbow-down
#
def solve_ik(x, y, z, ch="home"):
    print("----------------------------------")
    print(f"[IK] Solving for X={x} Y={y} Z={z}  mode={ch}")

    horiz    = math.sqrt(x*x + y*y)    #raon du plan(distance horizontale entre el base w target)
    z4       = z - SHOULDER_H          #nahina el hauteur mtaa l epaule(repere yabda reference z mel coude mouch mel base )
    sphere_r = math.sqrt(horiz*horiz + z4*z4)   #calcul distancve entre l epaule et target

    if sphere_r > MAX_ARM_RANGE:#securite(haja non atteignable)
        print(f"[IK ERROR] Point too far! sphere_r={sphere_r:.2f} MAX={MAX_ARM_RANGE}")
        return None
    if sphere_r < 1e-3:  #secutrite (pas de devision a 0)
        print("[IK ERROR] Target coincides with shoulder pivot!") 
        return None
    
   #theoreme el kashi calcule de t1(angle de rotation % oz
    alpha_num = horiz*horiz + z4*z4 + YOUPI_L1*YOUPI_L1 - YOUPI_L2*YOUPI_L2
    acos_arg  = alpha_num / (2.0 * YOUPI_L1 * sphere_r)

    if acos_arg < -1.0 or acos_arg > 1.0:
        print("[IK ERROR] acos out of domain — point unreachable!")
        return None

    beta  = math.atan2(z4, horiz)
    alpha = math.acos(acos_arg)
    t1    = (math.pi / 2.0) * sign_f(y) if x == 0.0 else math.atan2(y, x)
    
#-----------------------------vérification des contraintes d'espace de travail(haja physiquement realisable wale)-----------------------
    
    def try_solution(t2_candidate, label):
        elbow_x      = YOUPI_L1 * math.cos(t2_candidate)  #calcule de la position x du coude 
        elbow_z      = YOUPI_L1 * math.sin(t2_candidate)  #calcule de la position z du coude 
        dx           = horiz - elbow_x       #delta x binet actuel et target
        dz           = z4    - elbow_z       #delta z binet actuel et target
        t3_candidate = math.atan2(dz, dx)    #calcul de l angle que doit L2 faire pour atteindre la cible t3
    #remarque: atan2(taaml el arctan ama prend en consideratio,n e signe exp arctan(1,1)=45° et arctan(-1,-1)=-135° mouch 45°)
        in_t1 = T1_MIN <= t1            <= T1_MAX #boolen yverifie elli t1 eli lkineha f limite mta t1 max w t1 min
        in_t2 = T2_MIN <= t2_candidate  <= T2_MAX #boolen yverifie elli t2 eli lkineha f limite mta t2 max w t2 min
        in_t3 = T3_MIN <= t3_candidate  <= T3_MAX #boolen yverifie elli t3 eli lkineha f limite mta t3 max w t3 min

        status = "OK" if (in_t1 and in_t2 and in_t3) else "FAIL"
        print(f"[IK {label}] t1={math.degrees(t1):.2f}  "
              f"t2={math.degrees(t2_candidate):.2f}  "
              f"t3={math.degrees(t3_candidate):.2f}  "
              f"limits={status}")

        if in_t1 and in_t2 and in_t3:
            return t1, t2_candidate, t3_candidate   #jawna behi :))  renvoie les 3 angles au controleur de mvt
        return None  

    # ---- solution priority depends on mode ----
    if ch == "move":
        # picking: elbow-down is the natural downward reach
        order = [(beta + alpha, "elbow-down"), (beta + alpha, "elbow-up")]
    else:
        # homing / safe travel: elbow-up keeps arm clear of obstacles
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
    waypoint_push(t1, t2, t3)
    move_to_angles(t1, t2, t3)
    current_x, current_y, current_z = x, y, z
    return True

# ==================== HOME ====================

def save_home(x, y, z):#kbaal maysavi positiuon aala ases home ychoufha reachable wale selon, les limites
    global home_angles, home_saved, home_x, home_y, home_z
    global current_x, current_y, current_z

    res = solve_ik(x, y, z, ch="home")
    if res is None:
        print("[HOME ERROR] Home Position not reachable!")
        return

    home_x, home_y, home_z = x, y, z
    current_x, current_y, current_z = x, y, z
    home_angles = res
    home_saved  = True
    waypoint_clear()

    print("[HOME SAVED]")
    print(f"  X={x} Y={y} Z={z}")
    print(f"  Max moves before home required: {MAX_WAYPOINTS}")

def return_home():  #execution bech yarjaa lel home
    global current_x, current_y, current_z

    if not home_saved:
        print("[HOME ERROR] No home saved!") #masavinech home maaandou win yarjaa
        return
    if len(waypoint_log) == 0:
        print("[HOME] Already at home.")  #deja fel home
        return

    print(f"[RETURNING HOME] Retracing {len(waypoint_log)} waypoints...")
#mayarjaach mestwi  , yarjaa aala jortou(y3awed e chemain eli aamlou ama bel aaks)
    for i in range(len(waypoint_log) - 1, -2, -1):   #range(start: ekher point , stop: position -1  , step: bel aaks heka aaleh-1)
        if i == -1: #c est bon lkina les angles
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

    # 1. Rise to safe height at current position
    print("[PICK] 1/7 — rising to safe height at current position")
    if not move_xyz(current_x, current_y, SAFE_Z, ch="home"):       #cas d erreur 
        print("[PICK ERROR] Cannot rise to safe height — aborted")
        return

    # 2. Fly to target XY at safe height
    print("[PICK] 2/7 — moving to safe height above target")
    if not move_xyz(x, y, SAFE_Z, ch="home"):
        print("[PICK ERROR] Cannot reach safe position — aborted")
        return

    # 3. Descend to pick height  ← "move" mode: elbow-down preferred
    print("[PICK] 3/7 — descending to pick_z")
    if not move_xyz(x, y, PICK_Z, ch="move"):
        print("[PICK ERROR] Cannot reach pick position — aborted")
        return

    # 4. Grip delay
    print(f"[PICK] 4/7 — gripping ({GRIP_DELAY_MS} ms) ...")
    utime.sleep_ms(GRIP_DELAY_MS)

    # 5. Rise back to safe height
    print("[PICK] 5/7 — rising to safe_z")
    if not move_xyz(x, y, SAFE_Z, ch="home"):
        print("[PICK ERROR] Cannot rise — aborted")
        return
    utime.sleep_ms(GRIP_DELAY_MS)
    print("----------------------------------")
    print("Give next X Y to place, or press Enter to return home:")
    next_input = input().strip().lower()

    if next_input == "" or next_input == "home":
        pass  # continue to step 6 normally
    else:
        parts = next_input.split()
        if len(parts) == 2:
            nx, ny = float(parts[0]), float(parts[1])
            move_xyz(nx, ny, SAFE_Z, ch="home")
            move_xyz(nx, ny, PICK_Z, ch="move")
            utime.sleep_ms(GRIP_DELAY_MS)
            move_xyz(nx, ny, SAFE_Z, ch="home")

    # 6. Return to home
    print("[PICK] 6/7 — returning to home")
    return_home()

    # 7. Parking position (home but base rotated +90 deg)
    print("[PICK] 7/7 — moving to parking position (base +90 deg)")
    t1, t2, t3 = home_angles
    park_t1 = min(t1 + math.pi / 6.0, T1_MAX)
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

        if line == "home":        #si on texte "home" il decle,nche immediatement la fonction return home
            return_home()
            print("----------------------------------")
            print("Ready — send X Y")
            continue

        if line == "pos":       #si on texte "pos" le robot renvoie sa position XYZ
            print("----------------------------------")
            print(f"[POS] X={current_x}  Y={current_y}  Z={current_z}")
            print("----------------------------------")
            continue

        parts = line.split()   #si on ecrit "80 0" => va etre decouypé en "80" et "0" ki nektboulou les cordonnée X Y

        if len(parts) != 2:    #erreur ken nektbou haja feha akther men 2 parties
            print("[INPUT ERROR] Format: X Y  (e.g. 80 0)")
            continue

        tx = float(parts[0])  #partie loula eli hiya X (kbal el espace)
        ty = float(parts[1])  #partie thenya eli hiya Y(baed el espace)

        print(f"[INPUT] Pick at X={tx}  Y={ty}")
        pick_sequence(tx, ty)

        print("----------------------------------")
        print("Ready — send X Y")

    except Exception as e:
        print(f"[ERROR] {e}")