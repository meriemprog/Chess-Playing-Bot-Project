from machine import Pin
import utime

# ==================== CONFIGURATION DES PINS ====================
# Motor 1 Pins (Base)
MOTOR1_CLK = Pin(10, Pin.OUT)
MOTOR1_CW  = Pin(11, Pin.OUT)

# Motor 2 Pins (Épaule)
MOTOR2_CLK = Pin(13, Pin.OUT)
MOTOR2_CW  = Pin(12, Pin.OUT)

# Motor 3 Pins (Coude)
MOTOR3_CLK = Pin(14, Pin.OUT)
MOTOR3_CW  = Pin(15, Pin.OUT)

# ==================== PARAMÈTRES DE TEST ====================
STEPS = 1500            # Nombre de pas (correspond au réglage du driver)
STEP_DELAY_US = 500    # Microsecondes entre les impulsions
PAUSE_MS = 1000        # Pause entre les rotations

# Initialisation à l'état bas
for p in [MOTOR1_CLK, MOTOR1_CW, MOTOR2_CLK, MOTOR2_CW, MOTOR3_CLK, MOTOR3_CW]:
    p.value(0)

# ==================== FONCTION ROTATION ====================
def rotate_motor(clk_pin, cw_pin, steps, clockwise):
    # Définir la direction
    cw_pin.value(1 if clockwise else 0)
    utime.sleep_us(10) # Petit délai pour le driver TB6560

    for _ in range(steps):
        clk_pin.value(1)
        utime.sleep_us(STEP_DELAY_US)
        clk_pin.value(0)
        utime.sleep_us(STEP_DELAY_US)

# ==================== BOUCLE PRINCIPALE ====================
print("=== Test des 3 Moteurs Starting ===")
utime.sleep(2)

while True:
    # --- Moteur 1 ---
    print(">> Motor 1: Rotating CLOCKWISE...")
    rotate_motor(MOTOR1_CLK, MOTOR1_CW, STEPS, True)
    utime.sleep_ms(PAUSE_MS)
    
    print(">> Motor 1: Rotating COUNTER-CLOCKWISE...")
    rotate_motor(MOTOR1_CLK, MOTOR1_CW, STEPS, False)
    utime.sleep_ms(PAUSE_MS)

    # --- Moteur 2 ---
    print(">> Motor 2: Rotating CLOCKWISE...")
    rotate_motor(MOTOR2_CLK, MOTOR2_CW, STEPS, True)
    utime.sleep_ms(PAUSE_MS)
    
    print(">> Motor 2: Rotating COUNTER-CLOCKWISE...")
    rotate_motor(MOTOR2_CLK, MOTOR2_CW, STEPS, False)
    utime.sleep_ms(PAUSE_MS)

    # --- Moteur 3 ---
    print(">> Motor 3: Rotating CLOCKWISE...")
    rotate_motor(MOTOR3_CLK, MOTOR3_CW, STEPS, True)
    utime.sleep_ms(PAUSE_MS)
    
    print(">> Motor 3: Rotating COUNTER-CLOCKWISE...")
    rotate_motor(MOTOR3_CLK, MOTOR3_CW, STEPS, False)
    utime.sleep_ms(PAUSE_MS)