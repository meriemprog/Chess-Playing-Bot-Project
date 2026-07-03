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


def rotate_motor(motor_index, steps, direction, delay_us=2000):
    """
    motor_index : 0 (M1), 1 (M2), ou 2 (M3)
    steps       : nombre de pas à effectuer
    direction   : 1 (CW) ou -1 (CCW)
    delay_us    : vitesse (plus c'est petit, plus c'est rapide)
    """
    # Configuration de la direction
    dir_pins[motor_index].value(1 if direction > 0 else 0)
    
    # Exécution des pas
    for _ in range(steps):
        step_pins[motor_index].value(1)
        utime.sleep_us(10)  # Largeur d'impulsion minimale
        step_pins[motor_index].value(0)
        utime.sleep_us(delay_us)

# --- EXEMPLES D'UTILISATION ---

# Faire tourner M1 (base) d'un tour complet (11800 pas)
print("Rotation base...")
rotate_motor(0, 11800, 1)

# Faire tourner M2 (epaule) d'un demi-tour dans sens 1
print("Rotation epaule...")
rotate_motor(1, 3150, 1)

# Faire tourner M3 (coude) d'un demi-tour dans sens
print("Rotation coude...")
rotate_motor(2, 3250, 1)