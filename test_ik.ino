#include <Arduino.h>
#include <math.h>

// =============================================
// Pin Definitions
// =============================================
#define PIN_STEP_J1   6
#define PIN_DIR_J1    7
#define PIN_STEP_J2   3
#define PIN_DIR_J2    4
#define PIN_STEP_J3   10
#define PIN_DIR_J3    11

// =============================================
// Robot Dimensions
// =============================================
#define YOUPI_L1    135.0f
#define YOUPI_L2    115.0f
#define L_HORIZ      80.0f
#define L_VERT       60.0f
#define SHOULDER_H  295.0f

// =============================================
// Joint Angle Limits
// =============================================
#define T1_MIN  (-PI)
#define T1_MAX  ( PI)
#define T2_MIN  (-PI / 2.0f)
#define T2_MAX  ( PI / 2.0f)
#define T3_MIN  (-5.0f * PI / 6.0f)
#define T3_MAX  ( 5.0f * PI / 6.0f)

// =============================================
// Motor & Motion Settings
// =============================================
#define STEPS_REV       1600
#define STEP_DELAY_US   2000
#define SAFE_Z          120.0f
#define PICK_Z           15.0f

// =============================================
// TARGET POSITION — change these to test
// =============================================
#define TARGET_X   140.0f
#define TARGET_Y     145.0f
#define TARGET_Z   150.0f

// =============================================
// Motor Arrays
// =============================================
const float GEAR[3]    = {10.0f, 20.0f, 10.0f};
const int STEP_PINS[3] = {6, 10, 3};
const int DIR_PINS[3]  = {7, 11, 4};

// =============================================
// State Tracking
// =============================================
static long  current_steps[3] = {0, 0, 0};
static float current_x = 0, current_y = 0, current_z = SAFE_Z;

// =============================================
// Structs
// =============================================
struct Angles3DOF {
  float t1;
  float t2;
  float t3;
  float t4;
};

// =============================================
// Helpers
// =============================================
float clamp(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

long angle_to_steps(float angle_rad, int motor) {
  return (long)((angle_rad / (2.0f * PI)) * GEAR[motor] * STEPS_REV);
}

// =============================================
// Motor Control
// =============================================
void step_once(int motor, int dir) {
  digitalWrite(DIR_PINS[motor], dir > 0 ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(STEP_PINS[motor], HIGH);
  delayMicroseconds(10);
  digitalWrite(STEP_PINS[motor], LOW);
}

void move_to_angles(Angles3DOF target, int delay_us) {
  long target_steps[3] = {
    angle_to_steps(target.t1, 0),
    angle_to_steps(target.t2, 1),
    angle_to_steps(target.t3, 2)
  };

  long delta[3], absd[3], acc[3] = {0, 0, 0};
  int  dir[3];
  long max_steps = 0;

  for (int i = 0; i < 3; i++) {
    delta[i] = target_steps[i] - current_steps[i];
    dir[i]   = (delta[i] >= 0) ? 1 : -1;
    absd[i]  = abs(delta[i]);
    if (absd[i] > max_steps) max_steps = absd[i];
  }

  for (long s = 0; s < max_steps; s++) {
    for (int i = 0; i < 3; i++) {
      acc[i] += absd[i];
      if (acc[i] >= max_steps) {
        acc[i] -= max_steps;
        step_once(i, dir[i]);
        current_steps[i] += dir[i];
      }
    }
    delayMicroseconds(delay_us);
  }
}

// =============================================
// Inverse Kinematics
// =============================================
bool solve_ik(float x, float y, float z, Angles3DOF* a) {

  if (z < 0) return false;

  float r  = sqrt(x*x + y*y);
  float r4 = r - L_HORIZ;
  float z4 = z + L_VERT - SHOULDER_H;

  a->t1 = atan2(y, x);

  float D = sqrt(r4*r4 + z4*z4);

  if (D > YOUPI_L1 + YOUPI_L2)        return false;
  if (D < fabs(YOUPI_L1 - YOUPI_L2))  return false;

  float cos_t3 = (D*D - YOUPI_L1*YOUPI_L1 - YOUPI_L2*YOUPI_L2)
                 / (2.0f * YOUPI_L1 * YOUPI_L2);
  cos_t3       = clamp(cos_t3, -1.0f, 1.0f);
  float sin_t3 = sqrt(1.0f - cos_t3 * cos_t3);
  a->t3        = atan2(sin_t3, cos_t3);

  float alpha = atan2(z4, r4);
  float beta  = atan2(YOUPI_L2 * sin_t3, YOUPI_L1 + YOUPI_L2 * cos_t3);
  a->t2       = alpha - beta;

  a->t4 = -(a->t2 + a->t3);

  if (a->t1 < T1_MIN || a->t1 > T1_MAX) return false;
  if (a->t2 < T2_MIN || a->t2 > T2_MAX) return false;
  if (a->t3 < T3_MIN || a->t3 > T3_MAX) return false;

  return true;
}

// =============================================
// Setup
// =============================================
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 3; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i],  OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    digitalWrite(DIR_PINS[i],  LOW);
  }

  delay(1000);

  Serial.println("=== IK Test Starting ===");
  Serial.print("Target -> X: "); Serial.print(TARGET_X);
  Serial.print("  Y: ");         Serial.print(TARGET_Y);
  Serial.print("  Z: ");         Serial.println(TARGET_Z);

  Angles3DOF a;
  if (solve_ik(TARGET_X, TARGET_Y, TARGET_Z, &a)) {
    Serial.print("  Base     (t1): "); Serial.print(degrees(a.t1)); Serial.println("°");
    Serial.print("  Shoulder (t2): "); Serial.print(degrees(a.t2)); Serial.println("°");
    Serial.print("  Elbow    (t3): "); Serial.print(degrees(a.t3)); Serial.println("°");
    Serial.print("  Wrist    (t4): "); Serial.print(degrees(a.t4)); Serial.println("° (not driven)");
    move_to_angles(a, STEP_DELAY_US);
    Serial.println("=== Move Complete ===");
  } else {
    Serial.println("ERROR: Target unreachable or out of joint limits");
  }
}

// =============================================
// Loop — nothing, test runs once in setup
// =============================================
void loop() {
  // intentionally empty
}