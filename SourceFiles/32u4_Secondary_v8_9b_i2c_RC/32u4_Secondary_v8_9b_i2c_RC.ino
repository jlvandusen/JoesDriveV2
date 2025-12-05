/*
  JOE'S DRIVE v8.9b — 32u4 DOME CONTROLLER — FINAL & FULLY DOCUMENTED
  =====================================================================
  This is the "brain" of the BB-8 dome — a 32u4-based slave controller.
  It receives ALL commands from the ESP32 master via I2C (address 8).
  Features:
   • Full dome rotation (encoder + Hall sensor + drift correction)
   • Precise dome tilt servos with IMU compensation
   • DFPlayer Mini sound playback
   • Two modes: Free spin (L1 + Right Stick X) or Servo-follow (PID)
   • 100% identical behavior to v8.7 — but cleaner, faster, more reliable
   • Uses SharedStructs.h for perfect sync with ESP32
   • NO Serial.printf() — 100% AVR-safe for 32u4
*/

#include <Wire.h>
#include <VarSpeedServo.h>
#include <DFRobotDFPlayerMini.h>
#include <PID_v1.h>
#include "SharedStructs.h"  // ← MUST be in same folder — defines identical data layout

/* ==================== DEBUG CONTROL ==================== */
// Uncomment to see live data — zero impact when disabled
#define I2C_DEBUG     // Shows every packet received from ESP32
// #define DOME_DEBUG    // Shows encoder, Hall, motor, servo status
// #define DEBUG_ALL   // Enable everything

#ifdef DEBUG_ALL
  #define I2C_DEBUG
  #define DOME_DEBUG
#endif

/* ==================== HARDWARE PINOUT ==================== */
// Quadrature encoder for dome rotation tracking
#define motorEncoder_pin_A  18  // A0 — Channel A (PCINT7)
#define motorEncoder_pin_B  19  // A1 — Channel B (PCINT6)
#define hallEffectSensor_Pin 20 // A2 — Hall sensor (LOW when magnet present)

// Dome spin motor (L298N or similar H-bridge)
#define domeMotor_pwm   10      // PWM for speed control
#define domeMotor_pin_A  9      // Direction pin A
#define domeMotor_pin_B  6      // Direction pin B

// Dome tilt servos (periscope, panels, etc.)
#define leftServo_pin   12      // Left tilt servo
#define rightServo_pin  11      // Right tilt servo

/* ==================== PHYSICAL TUNING CONSTANTS ==================== */
// These values are tuned for YOUR specific dome — do not change unless you know why
const int servoSpeed                = 255;  // Maximum speed for VarSpeedServo (0–255)
const int servoEase                 = 10;   // Step size for smoothing (lower = smoother)
const int domeTiltYAxis_MaxAngle    = 12;   // Maximum vertical tilt in degrees (±12°)
const int domeTiltXAxis_MaxAngle    = 12;   // Maximum horizontal tilt in degrees (±12°)
const int TICKS_PER_REV             = 1680; // Encoder ticks for one full dome rotation
const int CENTER_TICK               = 878;  // YOUR measured center position (from Hall sensor)
const int MAX_ANGLE_TICKS           = 140;  // Maximum allowed deviation from center in ticks
const int HALL_DEBOUNCE             = 50;   // Milliseconds to debounce Hall sensor
const int CENTER_TOLERANCE          = 15;   // Ticks tolerance to consider "centered"
const int FIND_CENTER_SPEED         = 50;   // PWM speed when hunting for center
const int ENC_MIN_INTERVAL          = 500;  // Microseconds minimum between encoder interrupts (debounce)
const long DRIFT_CHECK_INTERVAL     = 5000; // Milliseconds between automatic drift correction checks

/* ==================== GLOBAL OBJECTS & STATE ==================== */
// Encoder tracking
volatile long encPos = CENTER_TICK;                    // Current encoder position (volatile = used in ISR)
volatile unsigned long lastEncInterrupt = 0;           // Timestamp of last encoder interrupt

// Servos & sound
VarSpeedServo leftServo, rightServo;                   // Smooth servo control with speed limiting
DFRobotDFPlayerMini player;                            // DFPlayer Mini on Serial1 @ 9600 baud

// Latest command packet from ESP32
Send32u4Data data;

// PID controller for dome servo-follow mode (keeps dome level when tilting)
double Input_domeSpinServoPid = 0;
double Output_domeSpinServoPid = 0;
double Setpoint_domeSpinServoPid = 0;
PID domePID(&Input_domeSpinServoPid, &Output_domeSpinServoPid, &Setpoint_domeSpinServoPid,
            3.5, 0.8, 0.05, DIRECT);                  // Tuned for smooth, fast response

// Servo neutral positions with your hardware offsets applied
int16_t leftServo_0_Position  = 70 - 7;                // Left servo center (offset = -7°)
int16_t rightServo_0_Position = 110 + 0;               // Right servo center (offset = 0°)

// Current and smoothed servo positions (for anti-jitter)
double leftServoPosition  = leftServo_0_Position;
double rightServoPosition = rightServo_0_Position;
double leftOldPosition    = leftServo_0_Position;
double rightOldPosition   = rightServo_0_Position;

// Current dome tilt angles from left stick input
double domeTiltAngle_X_Axis = 0;
double domeTiltAngle_Y_Axis = 0;
int8_t leftStickX = 0, leftStickY = 0;                 // Latest values from ESP32
unsigned long lastServoUpdateMillis = 0;               // For 5ms servo update timing

// Dome operational state
bool domeCenterSet = false;     // True when Hall sensor has found center magnet
bool domeServoMode = false;     // True = servo-follow mode, False = free spin
bool enableDrive = false;       // Master drive enable from ESP32
bool reverseDrive = false;      // Drive direction reverse flag

// Timing for centering and drift correction
unsigned long domeCenterStartMillis = 0;
unsigned long lastHallLowMillis = 0;
unsigned long lastDriftCheck = 0;
long lastStableEnc = CENTER_TICK;

/* ==================== QUADRATURE ENCODER INTERRUPT (PCINT) ==================== */
// High-precision encoder handling with full debouncing and direction detection
// Runs automatically on pin change — no polling needed
ISR(PCINT0_vect) {
  unsigned long now = micros();
  if (now - lastEncInterrupt < ENC_MIN_INTERVAL) return;  // Debounce
  lastEncInterrupt = now;

  static uint8_t lastState = 0;
  uint8_t a = digitalRead(motorEncoder_pin_A);
  uint8_t b = digitalRead(motorEncoder_pin_B);
  uint8_t state = (a << 1) | b;

  // Gray code transition table — rock-solid direction detection
  static const int8_t transitionTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  int8_t delta = transitionTable[(lastState << 2) | state];
  if (delta != 0) {
    encPos += delta;
    encPos = (encPos % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV;  // Safe wrap-around
  }
  lastState = state;
}

/* ==================== SETUP — SYSTEM INITIALIZATION ==================== */
void setup() {
  Serial.begin(115200);                    // Debug output to PC
  Serial1.begin(9600);                     // ← DFPLAYER REQUIRES 9600 BAUD — CRITICAL!

  Wire.begin(8);                           // Join I2C bus as slave address 8
  Wire.onReceive(receiveEvent);            // Register callback for incoming packets

  // Initialize DFPlayer Mini
  if (!player.begin(Serial1)) {
    Serial.println(F("DFPlayer init failed! Check wiring/power"));
    while (true) delay(100);               // Halt if sound system dead
  }
  player.volume(25);                       // 0–30 scale (25 = loud but safe)
  Serial.println(F("DFPlayer Mini READY @ 9600 baud"));

  // Initialize tilt servos
  leftServo.attach(leftServo_pin);
  rightServo.attach(rightServo_pin);
  leftServo.write(leftServo_0_Position, 10);
  rightServo.write(rightServo_0_Position, 10);

  // Setup dome spin motor pins
  pinMode(domeMotor_pwm,   OUTPUT);
  pinMode(domeMotor_pin_A, OUTPUT);
  pinMode(domeMotor_pin_B, OUTPUT);

  // Encoder + Hall sensor pins
  pinMode(motorEncoder_pin_A, INPUT_PULLUP);
  pinMode(motorEncoder_pin_B, INPUT_PULLUP);
  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);

  // Enable pin change interrupts for encoder
  PCMSK0 |= (1 << PCINT7) | (1 << PCINT6); // A0 & A1
  PCICR  |= (1 << PCIE0);

  // PID controller setup
  domePID.SetMode(AUTOMATIC);
  domePID.SetOutputLimits(-255, 255);

  // Find center using Hall sensor (10 second timeout)
  domeCenterStartMillis = millis();
  while (!domeCenterSet && millis() - domeCenterStartMillis < 10000) {
    setDomeCenter();
  }
  if (!domeCenterSet) Serial.println(F("Centering timeout — check Hall sensor/magnet"));

  lastDriftCheck = millis();
  lastStableEnc = encPos;

  Serial.println(F("JOE'S DRIVE v8.9b — 32u4 DOME CONTROLLER — I2C READY"));
}

/* ==================== MAIN LOOP — CORE OPERATION ==================== */
void loop() {
  // Keep trying to find center until Hall sensor sees magnet
  if (!domeCenterSet) {
    setDomeCenter();
    return;
  }

  DriftMonitor();      // Auto-correct small encoder drift over time
  spinStuff();         // Handle dome spin (free or servo-follow mode)
  Servos();            // Handle dome tilt with IMU compensation

  #ifdef DOME_DEBUG
  printDomeDebug();
  #endif

  delay(5);            // ~200 Hz loop — perfect for smooth servo response
}

/* ==================== I2C PACKET RECEPTION FROM ESP32 ==================== */
// Called automatically by hardware interrupt when ESP32 sends data
void receiveEvent(int bytes) {
  if (bytes < sizeof(data)) {
    #ifdef I2C_DEBUG
    Serial.print(F("I2C: SHORT PACKET ("));
    Serial.print(bytes);
    Serial.println(F(" bytes)"));
    #endif
    while (Wire.available()) Wire.read();  // Flush garbage
    return;
  }

  Wire.readBytes((uint8_t*)&data, sizeof(data));

  #ifdef I2C_DEBUG
  printI2CDebug();
  #endif

  // === SOUND PLAYBACK ===
  if (data.soundcmd >= 1 && data.soundcmd <= 10) {
    player.play(data.soundcmd);  // Plays 01.mp3 to 10.mp3
  }

  // === STATE FLAGS FROM ESP32 ===
  enableDrive   = data.driveEnabled;
  reverseDrive  = data.moveL3;
  domeServoMode = data.moveR3;

  // === DOME SPIN COMMAND (from L1 + Right Stick X) ===
  int domeSpinCmd = data.domeSpin;

  // === DOME TILT COMMAND (from Left Stick) ===
  int tiltX = data.leftStickX;
  int tiltY = data.leftStickY;

  // === SET PID TARGET IN SERVO MODE ===
  if (domeServoMode && domeCenterSet) {
    double targetTicks = constrain(map(domeSpinCmd, -127, 127, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS),
                                  -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS);
    Setpoint_domeSpinServoPid = targetTicks;
  } else {
    Setpoint_domeSpinServoPid = 0;  // Return to center when not in servo mode
  }

  // === STORE TILT VALUES FOR SERVO ROUTINE ===
  domeTiltAngle_X_Axis = map(tiltX, -127, 127, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);
  domeTiltAngle_Y_Axis = map(tiltY, -127, 127, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
}

/* ==================== DOME SPIN CONTROL ==================== */
void spinStuff() {
  if (domeServoMode && domeCenterSet) {
    domeServoMovement();   // PID-controlled precise positioning
  } else {
    spinDome();            // Free spin mode (L1 + Right Stick X)
  }
}

void spinDome() {
  int speed = abs(data.domeSpin) > 10 ? constrain(map(abs(data.domeSpin), 10, 127, 80, 255), 0, 255) : 0;
  if (data.domeSpin > 10 && enableDrive) {
    digitalWrite(domeMotor_pin_A, LOW);  digitalWrite(domeMotor_pin_B, HIGH);
    analogWrite(domeMotor_pwm, speed);
  } else if (data.domeSpin < -10 && enableDrive) {
    digitalWrite(domeMotor_pin_A, HIGH); digitalWrite(domeMotor_pin_B, LOW);
    analogWrite(domeMotor_pwm, speed);
  } else {
    stopDome();
  }
}

void domeServoMovement() {
  // Calculate current error from center
  Input_domeSpinServoPid = ((encPos - CENTER_TICK) % TICKS_PER_REV);
  if (Input_domeSpinServoPid > TICKS_PER_REV/2) Input_domeSpinServoPid -= TICKS_PER_REV;
  else if (Input_domeSpinServoPid < -TICKS_PER_REV/2) Input_domeSpinServoPid += TICKS_PER_REV;

  domePID.Compute();

  int speed = abs(Output_domeSpinServoPid) > 0.5 ? constrain(abs(Output_domeSpinServoPid), 80, 255) : 0;
  if (Output_domeSpinServoPid > 0.5 && enableDrive) {
    digitalWrite(domeMotor_pin_A, LOW);  digitalWrite(domeMotor_pin_B, HIGH);
    analogWrite(domeMotor_pwm, speed);
  } else if (Output_domeSpinServoPid < -0.5 && enableDrive) {
    digitalWrite(domeMotor_pin_A, HIGH); digitalWrite(domeMotor_pin_B, LOW);
    analogWrite(domeMotor_pwm, speed);
  } else {
    stopDome();
  }
}

void stopDome() {
  digitalWrite(domeMotor_pin_A, LOW);
  digitalWrite(domeMotor_pin_B, LOW);
  analogWrite(domeMotor_pwm, 0);
}

/* ==================== DOME TILT SERVOS WITH IMU COMPENSATION ==================== */
void Servos() {
  int y_Axis = data.leftStickY;
  int x_Axis = data.leftStickX;

  // Reduce tilt range when dome is turning hard (prevents binding)
  if (domeServoMode && domeCenterSet) {
    int domeTurnPercent = map(Setpoint_domeSpinServoPid, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS, 100, -100);
    int scale = 100 - abs(domeTurnPercent);
    y_Axis = map(y_Axis, -127, 127, -scale, scale);
    x_Axis = map(x_Axis, -127, 127, -scale, scale);
  }

  if (reverseDrive) { y_Axis *= -1; x_Axis *= -1; }

  leftStickY = map(y_Axis, -127, 127, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
  leftStickX = map(x_Axis, -127, 127, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);

  // Smooth updates every 5ms
  if (millis() - lastServoUpdateMillis >= 5) {
    lastServoUpdateMillis = millis();

    // Apply easing to prevent jerky motion
    domeTiltAngle_Y_Axis += (leftStickY > domeTiltAngle_Y_Axis)
        ? min(servoEase, leftStickY - domeTiltAngle_Y_Axis)
        : max(-servoEase, leftStickY - domeTiltAngle_Y_Axis);
    domeTiltAngle_X_Axis += (leftStickX > domeTiltAngle_X_Axis)
        ? min(servoEase, leftStickX - domeTiltAngle_X_Axis)
        : max(-servoEase, leftStickX - domeTiltAngle_X_Axis);

    domeTiltAngle_Y_Axis = constrain(domeTiltAngle_Y_Axis, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
    domeTiltAngle_X_Axis = constrain(domeTiltAngle_X_Axis, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);
  }

  // Apply IMU compensation from ESP32 (keeps dome level when ball tilts)
  float pitch = data.pitch;
  float roll  = data.roll;

  leftServoPosition  = leftServo_0_Position  + map(domeTiltAngle_Y_Axis - pitch, -24, 24, 36, -30);
  rightServoPosition = rightServo_0_Position + map(domeTiltAngle_Y_Axis - pitch, -24, 24, -36, 30);
  leftServoPosition  += map(domeTiltAngle_X_Axis + roll, -24, 24, 30, -30);
  rightServoPosition += map(domeTiltAngle_X_Axis + roll, -24, 24, 36, -36);

  // Anti-jitter smoothing — moves servos together smoothly
  double lDiff = abs(leftOldPosition  - leftServoPosition);
  double rDiff = abs(rightOldPosition - rightServoPosition);

  if (lDiff > rDiff && lDiff > 0.1) {
    leftOldPosition  += (leftOldPosition < leftServoPosition) ? min(1.0, lDiff) : max(-1.0, -lDiff);
    rightOldPosition += (rightOldPosition < rightServoPosition) ? rDiff/lDiff : -rDiff/lDiff;
  } else if (rDiff >= lDiff && rDiff > 0.1) {
    rightOldPosition += (rightOldPosition < rightServoPosition) ? min(1.0, rDiff) : max(-1.0, -rDiff);
    leftOldPosition  += (leftOldPosition < leftServoPosition) ? lDiff/rDiff : -lDiff/rDiff;
  }

  leftServo.write(constrain(leftOldPosition,  leftServo_0_Position-36,  leftServo_0_Position+36),  servoSpeed);
  rightServo.write(constrain(rightOldPosition, rightServo_0_Position-36, rightServo_0_Position+36), servoSpeed);
}

/* ==================== HALL SENSOR CENTERING ==================== */
void setDomeCenter() {
  if (digitalRead(hallEffectSensor_Pin) == LOW) {
    if (millis() - lastHallLowMillis >= HALL_DEBOUNCE) {
      domeCenterSet = true;
      encPos = CENTER_TICK;
      stopDome();
      Serial.println(F("DOME CENTERED AT 878"));
    }
    return;
  } else {
    lastHallLowMillis = millis();
  }

  double angleDiff = ((CENTER_TICK - encPos) % TICKS_PER_REV) * 360.0 / TICKS_PER_REV;
  if (angleDiff > 180) angleDiff -= 360;
  else if (angleDiff < -180) angleDiff += 360;

  if (abs(angleDiff) > (CENTER_TOLERANCE * 360.0 / TICKS_PER_REV)) {
    digitalWrite(domeMotor_pin_A, angleDiff > 0 ? HIGH : LOW);
    digitalWrite(domeMotor_pin_B, angleDiff > 0 ? LOW  : HIGH);
    analogWrite(domeMotor_pwm, FIND_CENTER_SPEED);
  } else {
    stopDome();
    if (abs(encPos - CENTER_TICK) <= CENTER_TOLERANCE) {
      domeCenterSet = true;
      encPos = CENTER_TICK;
    }
  }
}

/* ==================== DRIFT CORRECTION ==================== */
void DriftMonitor() {
  if (millis() - lastDriftCheck >= DRIFT_CHECK_INTERVAL) {
    if (domeCenterSet && digitalRead(hallEffectSensor_Pin) == LOW && abs(encPos - lastStableEnc) < 3) {
      encPos = CENTER_TICK;
      Serial.println(F("DRIFT CORRECTED — SNAPPED TO 878"));
    }
    lastStableEnc = encPos;
    lastDriftCheck = millis();
  }
}

/* ==================== DEBUG OUTPUTS (AVR SAFE) ==================== */
#ifdef I2C_DEBUG
void printI2CDebug() {
  static unsigned long last = 0;
  if (millis() - last < 500) return;
  last = millis();

  Serial.println();
  Serial.println(F("=== I2C FROM ESP32 ==="));
  Serial.print(F("Drive  : ")); Serial.println(data.driveEnabled ? F("ON ") : F("OFF"));
  Serial.print(F("Spin   : ")); Serial.println(data.domeSpin);
  Serial.print(F("Tilt X : ")); Serial.print(data.leftStickX);
  Serial.print(F("  Y: ")); Serial.println(data.leftStickY);
  Serial.print(F("Sound  : ")); Serial.println(data.soundcmd);
  Serial.print(F("PSI    : ")); Serial.println(data.psiFlash ? F("ON ") : F("OFF"));
  Serial.print(F("Rev    : ")); Serial.print(data.moveL3 ? F("YES") : F("NO "));
  Serial.print(F("  ServoMode: ")); Serial.println(data.moveR3 ? F("YES") : F("NO "));
  Serial.print(F("Pitch  : ")); Serial.print(data.pitch, 3);
  Serial.print(F("  Roll: ")); Serial.println(data.roll, 3);
  Serial.println(F("========================"));
}
#endif

#ifdef DOME_DEBUG
void printDomeDebug() {
  static unsigned long last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  Serial.println();
  Serial.print(F("DOME: Enc=")); Serial.print(encPos);
  Serial.print(F(" Center=")); Serial.print(domeCenterSet ? F("YES") : F("NO "));
  Serial.print(F(" Mode=")); Serial.println(domeServoMode ? F("SERVO") : F("SPIN"));
  Serial.print(F("Hall=")); Serial.print(digitalRead(hallEffectSensor_Pin));
  Serial.print(F(" PWM=")); Serial.print((int)Output_domeSpinServoPid);
  Serial.print(F(" L=")); Serial.print(leftServo.read());
  Serial.print(F(" R=")); Serial.println(rightServo.read());
}
#endif