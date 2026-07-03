#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// ==================== PIN DEFINES ====================

#define MOTOR1_CLK  10
#define MOTOR1_CW   11
#define MOTOR2_CLK  12
#define MOTOR2_CW   7
#define MOTOR3_CLK  3
#define MOTOR3_CW   4

// ==================== ARM GEOMETRY ====================

#define YOUPI_L1    163.0f
#define YOUPI_L2    163.0f
#define L_HORIZ      0.0f
#define L_VERT       0.0f
#define SHOULDER_H  80.0f

#define T1_MIN  (-PI)
#define T1_MAX  ( PI)
#define T2_MIN  (-PI / 2.0f)
#define T2_MAX  ( PI / 2.0f)
#define T3_MIN  (-5.0f * PI / 6.0f)
#define T3_MAX  ( 5.0f * PI / 6.0f)

#define STEPS_REV      1600
#define STEP_DELAY_US  2000

// ==================== STRUCTS ====================

struct Angles3DOF {
  float t1, t2, t3, t4;
};

struct Position {
  float x, y, z;
};

// ==================== WAYPOINT LOG (EEPROM) ====================

#define EEPROM_COUNT_ADDR   0
#define EEPROM_LOG_ADDR     2
#define BYTES_PER_WAYPOINT  12
#define MAX_WAYPOINTS       ((EEPROM.length() - EEPROM_LOG_ADDR) / BYTES_PER_WAYPOINT)

static uint16_t waypoint_count = 0;

void eeprom_write_float(int addr, float val) {
  byte* p = (byte*)&val;
  for (int i = 0; i < 4; i++) EEPROM.update(addr + i, p[i]);
}

float eeprom_read_float(int addr) {
  float val;
  byte* p = (byte*)&val;
  for (int i = 0; i < 4; i++) p[i] = EEPROM.read(addr + i);
  return val;
}

void waypoint_save_count() {
  EEPROM.update(EEPROM_COUNT_ADDR,     (byte)(waypoint_count & 0xFF));
  EEPROM.update(EEPROM_COUNT_ADDR + 1, (byte)(waypoint_count >> 8));
}

void waypoint_load_count() {
  waypoint_count = (uint16_t)EEPROM.read(EEPROM_COUNT_ADDR)
                 | ((uint16_t)EEPROM.read(EEPROM_COUNT_ADDR + 1) << 8);
  if (waypoint_count > MAX_WAYPOINTS) waypoint_count = 0;
}

void waypoint_push(Angles3DOF a) {
  if (waypoint_count >= MAX_WAYPOINTS) {
    Serial.println("[WARN] Waypoint log full! Send 'home' to clear.");
    return;
  }
  int addr = EEPROM_LOG_ADDR + waypoint_count * BYTES_PER_WAYPOINT;
  eeprom_write_float(addr,     a.t1);
  eeprom_write_float(addr + 4, a.t2);
  eeprom_write_float(addr + 8, a.t3);
  waypoint_count++;
  waypoint_save_count();
}

Angles3DOF waypoint_read(uint16_t i) {
  int addr = EEPROM_LOG_ADDR + i * BYTES_PER_WAYPOINT;
  Angles3DOF a;
  a.t1 = eeprom_read_float(addr);
  a.t2 = eeprom_read_float(addr + 4);
  a.t3 = eeprom_read_float(addr + 8);
  a.t4 = -(a.t2 + a.t3);
  return a;
}

void waypoint_clear() {
  waypoint_count = 0;
  waypoint_save_count();
}

// ==================== FORWARD DECLARATIONS ====================

float      clamp(float v, float lo, float hi);
long       angle_to_steps(float angle_rad, int motor);
void       step_once(int motor, int dir);
void       move_to_angles(Angles3DOF target, int delay_us);
bool       solve_ik(float x, float y, float z, Angles3DOF* a);
void       save_home(float x, float y, float z);
void       return_home(void);

// ==================== CONSTANTS ====================

const float GEAR[3]     = {10.0f, 20.0f, 10.0f};
const int   CLK_PINS[3] = {MOTOR1_CLK, MOTOR2_CLK, MOTOR3_CLK};
const int   CW_PINS[3]  = {MOTOR1_CW,  MOTOR2_CW,  MOTOR3_CW };

// ==================== GLOBALS ====================

static long       current_steps[3] = {0, 0, 0};
static Angles3DOF home_angles      = {0.0f, 0.0f, 0.0f, 0.0f};
static bool       home_saved       = false;

// Real-world home coordinates
static float home_x = 0.0f;
static float home_y = 0.0f;
static float home_z = 0.0f;

// Accumulated offset from home (current_x/y/z = 0 means at home)
static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;

// ==================== HELPERS ====================

float clamp(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

long angle_to_steps(float angle_rad, int motor) {
  return (long)((angle_rad / (2.0f * PI)) * GEAR[motor] * STEPS_REV);
}

// ==================== STEP ====================

void step_once(int motor, int dir) {
  digitalWrite(CW_PINS[motor], dir > 0 ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(CLK_PINS[motor], HIGH);
  delayMicroseconds(10);
  digitalWrite(CLK_PINS[motor], LOW);
}

// ==================== MOTION ====================

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

// ==================== INVERSE KINEMATICS ====================

bool solve_ik(float x, float y, float z, Angles3DOF* a) {
  Serial.println("----------------------------------");
  Serial.print("[IK] Solving for X="); Serial.print(x);
  Serial.print(" Y="); Serial.print(y);
  Serial.print(" Z="); Serial.println(z);

  float z_min = SHOULDER_H - YOUPI_L1 - YOUPI_L2;
  if (z < z_min) {
    Serial.print("[IK ERROR] Z too low! Min="); Serial.print(z_min);
    Serial.print(" Requested="); Serial.println(z);
    return false;
  }

  float r  = sqrt(x*x + y*y);
  float r4 = r - L_HORIZ;
  float z4 = z + L_VERT - SHOULDER_H;
  float D  = sqrt(r4*r4 + z4*z4);

  if (D > YOUPI_L1 + YOUPI_L2) {
    Serial.println("[IK ERROR] Point too far");
    Serial.print("  D="); Serial.print(D);
    Serial.print(" Max="); Serial.println(YOUPI_L1 + YOUPI_L2);
    return false;
  }
  if (D < fabs(YOUPI_L1 - YOUPI_L2)) {
    Serial.println("[IK ERROR] Point too close");
    return false;
  }

  float cos_t3 = (D*D - YOUPI_L1*YOUPI_L1 - YOUPI_L2*YOUPI_L2)
                 / (2.0f * YOUPI_L1 * YOUPI_L2);
  cos_t3       = clamp(cos_t3, -1.0f, 1.0f);
  float sin_t3 = sqrt(1.0f - cos_t3 * cos_t3);
  a->t3        = atan2(sin_t3, cos_t3);

  float alpha  = atan2(z4, r4);
  float beta   = atan2(YOUPI_L2 * sin_t3, YOUPI_L1 + YOUPI_L2 * cos_t3);
  a->t2        = alpha - beta;
  a->t1        = atan2(y, x);
  a->t4        = -(a->t2 + a->t3);

  Serial.print("[IK] t1="); Serial.print(degrees(a->t1));
  Serial.print("° t2="); Serial.print(degrees(a->t2));
  Serial.print("° t3="); Serial.println(degrees(a->t3));

  bool ok = true;
  if (a->t1 < T1_MIN || a->t1 > T1_MAX) { Serial.println("[LIMIT ERROR] t1 out of range"); ok = false; }
  if (a->t2 < T2_MIN || a->t2 > T2_MAX) { Serial.println("[LIMIT ERROR] t2 out of range"); ok = false; }
  if (a->t3 < T3_MIN || a->t3 > T3_MAX) { Serial.println("[LIMIT ERROR] t3 out of range"); ok = false; }

  if (!ok) {
    Serial.println("[IK ERROR] Joint limit violated — move aborted");
    return false;
  }

  Serial.println("[IK OK] All joints within limits");
  return true;
}

// ==================== HOME ====================

void save_home(float x, float y, float z) {
  Angles3DOF a;
  if (!solve_ik(x, y, z, &a)) {
    Serial.println("[HOME ERROR] Position not reachable!");
    return;
  }

  home_x = x;
  home_y = y;
  home_z = z;

  // Offset starts at 0 — arm is at home
  current_x = 0.0f;
  current_y = 0.0f;
  current_z = 0.0f;

  home_angles = a;
  home_saved  = true;

  waypoint_clear();

  Serial.println("[HOME SAVED]");
  Serial.print("  Home real-world: X="); Serial.print(x);
  Serial.print(" Y="); Serial.print(y);
  Serial.print(" Z="); Serial.println(z);
  Serial.print("  Max moves before home required: ");
  Serial.println(MAX_WAYPOINTS);
}

void return_home(void) {
  if (!home_saved) {
    Serial.println("[HOME ERROR] No home position saved!");
    return;
  }
  if (waypoint_count == 0) {
    Serial.println("[HOME] Already at home.");
    return;
  }

  Serial.print("[RETURNING HOME] Retracing ");
  Serial.print(waypoint_count); Serial.println(" waypoints...");

  for (int i = (int)waypoint_count - 2; i >= -1; i--) {
    Angles3DOF target;
    if (i == -1) {
      target = home_angles;
      Serial.println("[HOME] Final step — home angles");
    } else {
      target = waypoint_read((uint16_t)i);
      Serial.print("[HOME] Step to waypoint "); Serial.println(i);
    }
    move_to_angles(target, STEP_DELAY_US);
  }

  // Offset resets to 0 — back at home
  current_x = 0.0f;
  current_y = 0.0f;
  current_z = 0.0f;

  waypoint_clear();
  Serial.println("[HOME REACHED] Offset reset to (0, 0, 0)");
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 3; i++) {
    pinMode(CLK_PINS[i], OUTPUT);
    pinMode(CW_PINS[i],  OUTPUT);
    digitalWrite(CLK_PINS[i], LOW);
    digitalWrite(CW_PINS[i],  LOW);
  }

  delay(1000);

  waypoint_load_count();
  Serial.print("[BOOT] Waypoints in EEPROM: "); Serial.println(waypoint_count);

  save_home(300.0f, 0.0f, 150.0f);

  Serial.println("==================================");
  Serial.println("  YOUPI ARM — READY");
  Serial.print("  EEPROM size  : "); Serial.print(EEPROM.length()); Serial.println(" bytes");
  Serial.print("  Max moves    : "); Serial.println(MAX_WAYPOINTS);
  Serial.println("  Each X Y Z = mm to move from CURRENT position");
  Serial.println("  Send: X Y Z to move");
  Serial.println("  Send: home  to return to start");
  Serial.println("  Send: pos   to print current offset");
  Serial.println("==================================");
}

// ==================== LOOP ====================

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // Return to home
    if (line == "home") {
      return_home();
      Serial.println("----------------------------------");
      Serial.println("Ready — send X Y Z or 'home' or 'pos'");
      return;
    }

    // Print current offset from home
    if (line == "pos") {
      Serial.println("----------------------------------");
      Serial.print("[POS] Offset from home : X="); Serial.print(current_x);
      Serial.print(" Y="); Serial.print(current_y);
      Serial.print(" Z="); Serial.println(current_z);
      Serial.print("[POS] Real-world pos   : X="); Serial.print(home_x + current_x);
      Serial.print(" Y="); Serial.print(home_y + current_y);
      Serial.print(" Z="); Serial.println(home_z + current_z);
      Serial.println("----------------------------------");
      return;
    }

    // Parse X Y Z
    int s1 = line.indexOf(' ');
    int s2 = line.indexOf(' ', s1 + 1);

    if (s1 == -1 || s2 == -1) {
      Serial.println("[INPUT ERROR] Format: X Y Z  (e.g. 50 0 -20)");
      return;
    }

    // What you type = how many mm to move from current position
    float dx = line.substring(0, s1).toFloat();
    float dy = line.substring(s1 + 1, s2).toFloat();
    float dz = line.substring(s2 + 1).toFloat();

    // Real-world target = home + current offset + this new delta
    float tx = home_x + current_x + dx;
    float ty = home_y + current_y + dy;
    float tz = home_z + current_z + dz;

    Serial.print("[INPUT] Move by   : ("); Serial.print(dx);
    Serial.print(", "); Serial.print(dy);
    Serial.print(", "); Serial.print(dz); Serial.println(") mm");
    Serial.print("[INPUT] From offset: ("); Serial.print(current_x);
    Serial.print(", "); Serial.print(current_y);
    Serial.print(", "); Serial.print(current_z); Serial.println(")");
    Serial.print("[INPUT] Real target: ("); Serial.print(tx);
    Serial.print(", "); Serial.print(ty);
    Serial.print(", "); Serial.print(tz); Serial.println(")");

    Angles3DOF a;
    if (solve_ik(tx, ty, tz, &a)) {
      waypoint_push(a);
      Serial.println("[MOVING]");
      move_to_angles(a, STEP_DELAY_US);

      // Accumulate offset only after successful move
      current_x += dx;
      current_y += dy;
      current_z += dz;

      Serial.println("[DONE]");
      Serial.print("  New offset : X="); Serial.print(current_x);
      Serial.print(" Y="); Serial.print(current_y);
      Serial.print(" Z="); Serial.println(current_z);
      Serial.print("  Real pos   : X="); Serial.print(home_x + current_x);
      Serial.print(" Y="); Serial.print(home_y + current_y);
      Serial.print(" Z="); Serial.println(home_z + current_z);
      Serial.print("  Waypoints  : "); Serial.print(waypoint_count);
      Serial.print("/"); Serial.println(MAX_WAYPOINTS);
    }

    Serial.println("----------------------------------");
    Serial.println("Ready — send X Y Z or 'home' or 'pos'");
  }
}

