from machine import Pin
import utime

# --- Configuration du moteur (Pins 6, 7, 8, 9) ---
# Dans ton code Arduino : IN1=6, IN2=7, IN3=8, IN4=9
pins = [Pin(6, Pin.OUT), Pin(7, Pin.OUT), Pin(8, Pin.OUT), Pin(9, Pin.OUT)]

# Séquence de pas (Full step - 4 étapes) 
# Note : C'est l'ordre classique pour correspondre à la librairie Stepper d'Arduino
sequence = [
    [1, 0, 0, 0], # IN1
    [0, 1, 0, 0], # IN2
    [0, 0, 1, 0], # IN3
    [0, 0, 0, 1]  # IN4
]

def step_motor(steps):
    direction = 1 if steps > 0 else -1
    steps = abs(steps)
    
    # On utilise une variable globale pour garder trace de la phase actuelle
    # Cela évite les secousses entre deux appels de fonction    global current_phase
    if 'current_phase' not in globals():
        current_phase = 0

    for _ in range(steps):
        current_phase = (current_phase + direction) % 4
        for i in range(4):
            pins[i].value(sequence[current_phase][i])
        
        # Vitesse : 10ms correspond environ à setSpeed(6) en Arduino
        utime.sleep_ms(10)
    
    # Très important : Éteindre les bobines après le mouvement
    for p in pins: p.value(0)

# --- Boucle principale (Loop) ---
while True:
    print("Avance de 512 pas...")
    step_motor(512)
    
    print("Attente de 6 secondes...")
    utime.sleep(6)
    
    print("Recul de 512 pas...")
    step_motor(-512)
    
    print("Attente de 6 secondes...")
    utime.sleep(6)