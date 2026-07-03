#include <Arduino.h>

// --- Motor 1 Pins ---
#define MOTOR1_CLK 10
#define MOTOR1_CW   11


// --- Motor 2 Pins ---
#define MOTOR2_CLK  12
#define MOTOR2_CW   13

// --- Motor 3 Pins ---
#define MOTOR3_CLK  14
#define MOTOR3_CW   15


// --- Test Parameters ---
#define STEPS       400       // Steps per rotation
#define STEP_DELAY  500      // Microseconds between pulses
#define PAUSE       1000      // Pause between rotations

// =============================================
// Rotate Motor Function
// =============================================
void rotateMotor(int clkPin, int cwPin, int steps, bool clockwise) {
  digitalWrite(cwPin, clockwise ? HIGH : LOW);
  delayMicroseconds(10);

  for (int i = 0; i < steps; i++) {
    digitalWrite(clkPin, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(clkPin, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}

// =============================================
// Setup
// =============================================
void setup() {
  Serial.begin(9600);

  pinMode(MOTOR1_CLK, OUTPUT);
  pinMode(MOTOR1_CW, OUTPUT);

  digitalWrite(MOTOR1_CLK, LOW);
  digitalWrite(MOTOR1_CW, LOW);

  pinMode(MOTOR2_CLK, OUTPUT);
  pinMode(MOTOR2_CW, OUTPUT);

  digitalWrite(MOTOR2_CLK, LOW);
  digitalWrite(MOTOR2_CW, LOW);

  pinMode(MOTOR3_CLK, OUTPUT);
  pinMode(MOTOR3_CW, OUTPUT);

  digitalWrite(MOTOR3_CLK, LOW);
  digitalWrite(MOTOR3_CW, LOW);

  Serial.println("=== Motor 1 Test Starting ===");
  delay(2000);
}

// =============================================
// Loop
// =============================================
void loop() {
  Serial.println(">> Motor 1: Rotating CLOCKWISE...");
  rotateMotor(MOTOR1_CLK, MOTOR1_CW, STEPS, true);
  delay(PAUSE);

  Serial.println(">> Motor 1: Rotating COUNTER-CLOCKWISE...");
  rotateMotor(MOTOR1_CLK, MOTOR1_CW, STEPS, false);
  delay(PAUSE);


    Serial.println(">> Motor 2: Rotating CLOCKWISE...");
  rotateMotor(MOTOR2_CLK, MOTOR2_CW, STEPS, true);
  delay(PAUSE);

  Serial.println(">> Motor 2: Rotating COUNTER-CLOCKWISE...");
  rotateMotor(MOTOR2_CLK, MOTOR2_CW, STEPS, false);
  delay(PAUSE);

  
    Serial.println(">> Motor 3: Rotating CLOCKWISE...");
  rotateMotor(MOTOR3_CLK, MOTOR3_CW, STEPS, true);
  delay(PAUSE);

  Serial.println(">> Motor 3: Rotating COUNTER-CLOCKWISE...");
  rotateMotor(MOTOR3_CLK, MOTOR3_CW, STEPS, false);
  delay(PAUSE);
}