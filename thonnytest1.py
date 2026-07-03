from machine import Pin, Timer
import math
import utime
import struct

# ==================== CONFIGURATION DES PINS ====================
# Format: [Step_Pin, Dir_Pin]
PINS = {
    "m1": [10, 11], # Base
    "m2": [12, 7],  # Épaule
    "m3": [3, 4]    # Coude
}

# Objets Pin
step_pins = [Pin(PINS["m1"][0], Pin.OUT), Pin(PINS["m2"][0], Pin.OUT), Pin(PINS["m3"][0], Pin.OUT)]
dir_pins  = [Pin(PINS["m1"][1], Pin.OUT), Pin(PINS["m2"][1], Pin.OUT), Pin(PINS["m3"][1], Pin.OUT)]

# ==================== GÉOMÉTRIE DU BRAS ====================
YOUPI_L1, YOUPI_L2 = 163.0, 163.0
SHOULDER_H = 80.0
GEAR = [10.0, 20.0, 10.0]
STEPS_REV = 1600
STEP_DELAY_US = 2000

# Limites (Radians)
T1_LIMITS = (-math.pi, math.pi)
T2_LIMITS = (-math.pi/2, math.pi/2)
T3_LIMITS = (-5*math.pi/6, 5*math.pi/6)

# ==================== VARIABLES D'ÉTAT ====================
current_steps = [0, 0, 0]
home_pos = {"x": 300.0, "y": 0.0, "z": 150.0}
current_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
waypoint_log = [] # Liste pour remplacer l'EEPROM (mémoire vive du Pico)

# ==================== FONCTIONS DE MOUVEMENT ====================

def angle_to_steps(angle_rad, motor_idx):
    return int((angle_rad / (2 * math.pi)) * GEAR[motor_idx] * STEPS_REV)

def move_to_angles(t1, t2, t3):
    global current_steps
    target_steps = [angle_to_steps(t1, 0), angle_to_steps(t2, 1), angle_to_steps(t3, 2)]
    
    delta = [target_steps[i] - current_steps[i] for i in range(3)]
    directions = [1 if d >= 0 else 0 for d in delta]
    abs_delta = [abs(d) for d in delta]
    max_s = max(abs_delta)
    
    # Appliquer les directions
    for i in range(3):
        dir_pins[i].value(directions[i])
    
    # Mouvement synchrone
    acc = [0, 0, 0]
    for _ in range(max_s):
        for i in range(3):
            acc[i] += abs_delta[i]
            if acc[i] >= max_s:
                acc[i] -= max_s
                step_pins[i].value(1)
        utime.sleep_us(10) # Largeur d'impulsion
        for p in step_pins: p.value(0)
        utime.sleep_us(STEP_DELAY_US)
    
    current_steps = target_steps

# ==================== CINÉMATIQUE INVERSE ====================

def solve_ik(x, y, z):
    z_min = SHOULDER_H - YOUPI_L1 - YOUPI_L2
    if z < z_min: return None
    
    r = math.sqrt(x**2 + y**2)
    z4 = z - SHOULDER_H
    D = math.sqrt(r**2 + z4**2)
    
    if D > (YOUPI_L1 + YOUPI_L2) or D < abs(YOUPI_L1 - YOUPI_L2):
        return None
        
    cos_t3 = (D**2 - YOUPI_L1**2 - YOUPI_L2**2) / (2 * YOUPI_L1 * YOUPI_L2)
    cos_t3 = max(-1, min(1, cos_t3))
    t3 = math.acos(cos_t3)
    
    alpha = math.atan2(z4, r)
    beta = math.atan2(YOUPI_L2 * math.sin(t3), YOUPI_L1 + YOUPI_L2 * math.cos(t3))
    t2 = alpha - beta
    t1 = math.atan2(y, x)
    
    # Vérification des limites
    if not (T1_LIMITS[0] <= t1 <= T1_LIMITS[1]): return None
    if not (T2_LIMITS[0] <= t2 <= T2_LIMITS[1]): return None
    if not (T3_LIMITS[0] <= t3 <= T3_LIMITS[1]): return None
    
    return t1, t2, t3

# ==================== BOUCLE PRINCIPALE ====================

print("YOUPI ARM READY - Mode Python")

while True:
    try:
        line = input("Entrez X Y Z (ex: 10 0 -20) ou 'home': ").strip().lower()
        
        if line == "home":
            print("Retour à la base...")
            angles = solve_ik(home_pos["x"], home_pos["y"], home_pos["z"])
            move_to_angles(*angles)
            current_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
            continue
            
        parts = line.split()
        if len(parts) == 3:
            dx, dy, dz = map(float, parts)
            tx = home_pos["x"] + current_offset["x"] + dx
            ty = home_pos["y"] + current_offset["y"] + dy
            tz = home_pos["z"] + current_offset["z"] + dz
            
            res = solve_ik(tx, ty, tz)
            if res:
                print(f"Déplacement vers: {tx}, {ty}, {tz}")
                move_to_angles(*res)
                current_offset["x"] += dx
                current_offset["y"] += dy
                current_offset["z"] += dz
            else:
                print("Erreur: Position hors de portée ou limites violées.")
    except Exception as e:
        print(f"Erreur de saisie: {e}")