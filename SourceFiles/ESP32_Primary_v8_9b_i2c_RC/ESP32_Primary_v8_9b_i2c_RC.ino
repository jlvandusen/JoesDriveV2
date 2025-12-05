/*
  JOE'S DRIVE v8.9b — ESP32 MASTER — FINAL & COMPLETE
  - Serial1 → IMU Board (unchanged, EasyTransfer)
  - I2C → DOME CONTROLLER (32u4 with DFPlayer + ALL servos)
  - Uses SharedStructs.h for identical data layout
  - 100% identical behavior to v8.7
*/

#include <Arduino.h>
#include <Wire.h>
#include <EasyTransfer.h>
#include <analogWrite.h>
#include <Preferences.h>
#include <PSController.h>
#include <Ticker.h>
#include <PID_v1.h>
#include "SharedStructs.h"  // ← NEW: shared struct definition

/* ==================== DEBUG CONTROL ==================== */
#define DEBUG_CALIBRATION      // Pot + saved offsets
// #define IMUDEBUG               // ← NEW: Raw + Adjusted Pitch/Roll + Saved Offsets
// #define DEBUG_JOYSTICK
// #define DEBUG_S2S_MODE
// #define DEBUG_ALL              // Uncomment to enable everything

/* ------------------- CONFIG ------------------- */
const bool ENABLE_ESPNOW = false;
const bool REVERSE_S2S   = true;
const int  MAX_S2S_TILT  = 300;
const int  JOYSTICK_DEADZONE = 25;
const int  S2S_POT_DEF   = 1708; // 1708 close to center physically of the potentiometer with the slot horizontal
int  S2S_POT_MIN   = (S2S_POT_DEF-400); // 1300
int  S2S_POT_MAX   = (S2S_POT_DEF+400); // 2150

const int  POT_FILTER_SIZE = 10;
const char* MASTER_NAV   = "7c:9e:bd:d7:63:c6";
const uint8_t DOME_CONTROLLER_ADDR = 8;

/* ----- RETURN-TO-CENTER TUNING ----- */
const int  RETURN_DEADBAND   = 25;
const int  MIN_RETURN_PWM    = 30;
const int  MAX_RETURN_PWM    = 180;
const bool ENABLE_ROLL_BIAS  = false;
const float ROLL_BIAS_GAIN   = 10.0;


/* ----- PID BALANCE TUNING (DRIVE) ----- */
float KP_PITCH = 45.0;
float KI_PITCH = 0.8;
float KD_PITCH = 12.0;
const float IMU_DEADZONE = 1.45;
const unsigned long IMU_TIMEOUT = 500;

/* ----- PID BALANCE TUNING (ROLL) ----- */
const float INTEGRAL_LIMIT_ROLL = 100.0;

/* ----- CALIBRATION HOLD ----- */
const unsigned long CALIB_HOLD_TIME = 3000;  // 3 seconds
unsigned long calibHoldStart = 0;
bool calibSaveActive = false;
bool calibResetActive = false;

/* LIVE TUNING MODE */
bool tuningMode = false;
enum class TuneState { 
  IDLE,
  PITCH_KP, PITCH_KD, PITCH_KI,
  ROLL_KP,  ROLL_KD,  ROLL_IK,
  DONE
} tuneState = TuneState::IDLE;

unsigned long psHoldStart = 0;
const unsigned long PS_HOLD_TIME = 3000;

float  tempKP_PITCH = 45.0f;
float  tempKD_PITCH = 0.0f;
float  tempKI_PITCH = 0.0f;
double tempPk2 = 45.0;
double tempDk2 = 0.0;
double tempIk2 = 0.0;

/* ------------------- PINS ------------------- */
const uint8_t S2S_PWM      = 33;
const uint8_t S2S_PIN_1    = 26;
const uint8_t S2S_PIN_2    = 25;
const uint8_t DRIVE_PWM    = 21;
const uint8_t DRIVE_PIN_1  = 4;
const uint8_t DRIVE_PIN_2  = 27;
const uint8_t S2S_POT_PIN  = 34;
const uint8_t FLYWHEEL_PWM = 15;
const uint8_t FLYWHEEL_PIN_A = 32;
const uint8_t FLYWHEEL_PIN_B = 14;

/* ------------------- STRUCTS ------------------- */
//  These are all called from sharedstructs.h

/* ------------------- GLOBALS ------------------- */
EasyTransfer recIMU;
Preferences preferences;
PSController driveController(nullptr), domeController(nullptr);
Ticker mainLoopTicker;
IMUData receiveIMUData;
Send32u4Data sendTo32u4Data;
ControllerButtons buttonsL, buttonsR, prevR, prevL;
bool IMUconnected = false, controllerConnected = false;
bool drivecontrollerConnected = false, domecontrollerConnected = false;
bool DomeServoMode = false, enableDrive = false, reverseDrive = false, EnableFlywheel = false;
bool autoBalance = false;
float flywheel = 0;
unsigned long lastIMUMillis = 0;
int potOffset = S2S_POT_DEF;
float pitchOffset = 0.0, rollOffset = 0.0;
int potValues[POT_FILTER_SIZE];
int potValueIndex = 0;

unsigned long lastCompactDebug = 0;
const unsigned long COMPACT_DEBUG_INTERVAL = 100;


/* S2S Debug */
int lastPwmS2S = 0;
enum class S2SMode { JOY, RTN, STOP };
S2SMode s2sMode = S2SMode::STOP;

/* PID Balance (Pitch) */
float pitchIntegral = 0.0;
float prevPitchError = 0.0;
unsigned long lastPIDTime = 0;
const float INTEGRAL_LIMIT = 100.0;

/* PID Balance (Roll) - S2S */
double Setpoint2 = 0.0;           // We want roll = 0
double Input2 = 0.0;              // Current roll angle
double Output2 = 0.0;             // PID output (-255 to +255)
double Output2_S2S_pwm = 0;       // Final PWM

// PID gains - START HERE and tune!
double Pk2 = 45.0;   // Start with pitch KP (was working)
double Ik2 = 1.0;    // Small I to eliminate steady-state error
double Dk2 = 15.0;   // Damping - critical to stop oscillation

PID PID2_S2S(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

/* PID Balance (Roll) */
float rollIntegral = 0.0;
float prevRollError = 0.0;
unsigned long lastS2STime = 0;

/* ------------------- SETUP ------------------- */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // ← IMU Board (unchanged)
  Wire.begin(18, 19);     // ← DOME CONTROLLER (I2C)

  preferences.begin("JoeDriveV2", false);
  potOffset   = preferences.getInt("potOffset", S2S_POT_DEF);
  pitchOffset = preferences.getFloat("pitchOffset", 0.0);
  rollOffset  = preferences.getFloat("rollOffset", 0.0);

  Serial.printf("T: %lu INFO: Loaded Cal - Pot:%d Pitch:%.2f Roll:%.2f\n",
                millis(), potOffset, pitchOffset, rollOffset);

  for (int i = 0; i < POT_FILTER_SIZE; i++) potValues[i] = potOffset;

  pinMode(S2S_PWM, OUTPUT);
  pinMode(S2S_PIN_1, OUTPUT);
  pinMode(S2S_PIN_2, OUTPUT);
  pinMode(DRIVE_PWM, OUTPUT);
  pinMode(DRIVE_PIN_1, OUTPUT);
  pinMode(DRIVE_PIN_2, OUTPUT);
  pinMode(FLYWHEEL_PWM, OUTPUT);
  pinMode(FLYWHEEL_PIN_A, OUTPUT);
  pinMode(FLYWHEEL_PIN_B, OUTPUT);
  pinMode(S2S_POT_PIN, INPUT);

  if (!PSController::startListening(MASTER_NAV))
    Serial.printf("T: %lu BT failed\n", millis());
  else
    Serial.printf("T: %lu BT MAC: %s\n", millis(), PSController::getDeviceAddress().c_str());

  recIMU.begin(details(receiveIMUData), &Serial1);

  for (int i = 0; i < 10000; i++) {
    if (recIMU.receiveData()) { IMUconnected = true; lastIMUMillis = millis(); break; }
    delay(1);
  }

  PID2_S2S.SetMode(AUTOMATIC);
  PID2_S2S.SetOutputLimits(-255, 255);
  PID2_S2S.SetSampleTime(10);

  lastPIDTime = millis();
  mainLoopTicker.attach_ms(10, mainLoop);
  Serial.println("JOE'S DRIVE v8.9b — ESP32 MASTER — SERIAL1 IMU + I2C DOME CONTROLLER — READY");
}

/* ------------------- MAIN LOOP ------------------- */
void mainLoop() {
  updateControllerStates();
  handleTuningModeEntry();

  if (tuningMode) {
    handleTuningMode();
    printTuningDebug();
    return; // BLOCKS ALL NORMAL OPERATION
  }

  receiveIMU();
  handleCalibrationHold();
  handleButtonActions();
  S2S_Movement();
  drive_Movement();
  spinFlywheel();
  sendToDomeController();
  printDebugInfo();
}

/* ------------------- I2C TO DOME CONTROLLER ------------------- */
void sendToDomeController() {
  Wire.beginTransmission(DOME_CONTROLLER_ADDR);
  Wire.write((uint8_t*)&sendTo32u4Data, sizeof(sendTo32u4Data));
  Wire.endTransmission();
}

/* ------------------- IMU FROM SERIAL1 ------------------- */
void receiveIMU() {
  unsigned long now = millis();
  if (recIMU.receiveData()) {
    float rawPitch = receiveIMUData.pitch - pitchOffset;
    float rawRoll  = receiveIMUData.roll  - rollOffset;

    sendTo32u4Data.pitch = abs(rawPitch) > IMU_DEADZONE ? rawPitch : 0;
    sendTo32u4Data.roll  = abs(rawRoll)  > IMU_DEADZONE ? rawRoll  : 0;

    IMUconnected = true;
    lastIMUMillis = now;
  } else if (now - lastIMUMillis > IMU_TIMEOUT) {
    IMUconnected = false;
    if (autoBalance) {
      autoBalance = false;
      Serial.printf("T: %lu IMU Lost → Balance OFF\n", now);
    }
  }
}

/* ------------------- 3-SEC CALIBRATION HOLD ------------------- */
void handleCalibrationHold() {
  bool bothUp   = buttonsL.up && buttonsR.up;
  bool bothDown = buttonsL.down && buttonsR.down;

  unsigned long now = millis();

  // --- SAVE (UP + UP) ---
  if (bothUp && !calibSaveActive) {
    calibHoldStart = now;
    calibSaveActive = true;
    calibResetActive = false;
  } else if (!bothUp && calibSaveActive) {
    calibSaveActive = false;
  }

  if (calibSaveActive && (now - calibHoldStart >= CALIB_HOLD_TIME)) {
    saveCalibration();
    calibSaveActive = false;
  }

  // --- RESET (DOWN + DOWN) ---
  if (bothDown && !calibResetActive) {
    calibHoldStart = now;
    calibResetActive = true;
    calibSaveActive = false;
  } else if (!bothDown && calibResetActive) {
    calibResetActive = false;
  }

  if (calibResetActive && (now - calibHoldStart >= CALIB_HOLD_TIME)) {
    resetCalibration();
    calibResetActive = false;
  }
}

/* ------------------- SAVE / RESET ------------------- */
void saveCalibration() {
  int pot = filterPotValue(analogRead(S2S_POT_PIN));
  potOffset = pot;
  pitchOffset = receiveIMUData.pitch;
  rollOffset  = receiveIMUData.roll;

  preferences.putInt("potOffset", potOffset);
  preferences.putFloat("pitchOffset", pitchOffset);
  preferences.putFloat("rollOffset", rollOffset);
  S2S_POT_MIN   = (potOffset-400);
  S2S_POT_MAX   = (potOffset+400);
  //add bounds check (purely defensive)
  // S2S_POT_MIN = max(potOffset - 600, 500);   // prevent absurd values
  // S2S_POT_MAX = min(potOffset + 350, 700);

  Serial.printf("T: %lu INFO: CAL SAVED → Pot:%d PotMin:%d PotMax:%d Pitch:%.2f Roll:%.2f\n",
                millis(), potOffset, S2S_POT_MIN, S2S_POT_MAX, pitchOffset, rollOffset);
}

void resetCalibration() {
  potOffset = 0;
  pitchOffset = 0.0;
  rollOffset = 0.0;

  preferences.putInt("potOffset", 0);
  preferences.putFloat("pitchOffset", 0.0);
  preferences.putFloat("rollOffset", 0.0);

  Serial.printf("T: %lu INFO: CAL RESET → All offsets set to 0\n", millis());
}

/* ------------------- CONTROLLER ------------------- */
void updateControllerStates() {
  drivecontrollerConnected = driveController.isConnected();
  domecontrollerConnected = domeController.isConnected();
  controllerConnected = drivecontrollerConnected || domecontrollerConnected;

  if (controllerConnected) {
    buttonsR.l1 = driveController.state.button.l1;
    buttonsR.r1 = driveController.state.button.r1;
    buttonsR.l2 = constrain(map(driveController.state.analog.button.l2, 0, 255, 0, 100), 0, 100);
    buttonsR.l3 = driveController.state.button.l3;
    buttonsR.cross = driveController.state.button.cross;
    buttonsR.circle = driveController.state.button.circle;
    buttonsR.up = driveController.state.button.up;
    buttonsR.down = driveController.state.button.down;
    buttonsR.left = driveController.state.button.left;
    buttonsR.right = driveController.state.button.right;
    buttonsR.ps = driveController.state.button.ps;
    buttonsR.rightStickX = driveController.state.analog.stick.lx;
    buttonsR.rightStickY = driveController.state.analog.stick.ly;
    applyDeadzone(buttonsR.rightStickX, buttonsR.rightStickY);

    buttonsL.l1 = domeController.state.button.l1;
    buttonsL.l2 = constrain(map(domeController.state.analog.button.l2, 0, 255, 0, 100), 0, 100);
    buttonsL.l3 = domeController.state.button.l3;
    buttonsL.cross = domeController.state.button.cross;
    buttonsL.circle = domeController.state.button.circle;
    buttonsL.up = domeController.state.button.up;
    buttonsL.down = domeController.state.button.down;
    buttonsL.left = domeController.state.button.left;
    buttonsL.right = domeController.state.button.right;
    buttonsL.ps = domeController.state.button.ps;
    buttonsL.leftStickX = domeController.state.analog.stick.lx;
    buttonsL.leftStickY = domeController.state.analog.stick.ly;
    applyDeadzone(buttonsL.leftStickX, buttonsL.leftStickY);
  }
}

void applyDeadzone(int8_t& x, int8_t& y) {
  x = (abs(x) > JOYSTICK_DEADZONE) ? x : 0;
  y = (abs(y) > JOYSTICK_DEADZONE) ? y : 0;
}

int filterPotValue(int raw) {
  potValues[potValueIndex] = raw;
  potValueIndex = (potValueIndex + 1) % POT_FILTER_SIZE;
  long sum = 0;
  for (int i = 0; i < POT_FILTER_SIZE; i++) sum += potValues[i];
  return sum / POT_FILTER_SIZE;
}

/* ------------------- BUTTONS ------------------- */
void handleButtonActions() {
  if (!controllerConnected) return;
  static ControllerButtons prevR, prevL;
#define BTN_PRESSED(b,s) (prev##s.b != buttons##s.b && buttons##s.b)

  if (buttonsR.l1) {
    flywheel = 0;
    sendTo32u4Data.domeSpin = buttonsR.rightStickX;
    buttonsR.rightStickY = 0;
  } else if (buttonsL.l1) {
    sendTo32u4Data.domeSpin = 0;
    EnableFlywheel = true;
  } else {
    sendTo32u4Data.domeSpin = 0;
    EnableFlywheel = false;
  }

  // Sound commands (exact v8.7)
  if (buttonsR.up && buttonsL.up) sendTo32u4Data.soundcmd = 6;
  else if (buttonsR.up) sendTo32u4Data.soundcmd = 1;
  else if (buttonsR.right) sendTo32u4Data.soundcmd = 2;
  else if (buttonsR.down) sendTo32u4Data.soundcmd = 3;
  else if (buttonsR.left) sendTo32u4Data.soundcmd = 4;
  else if (buttonsL.up) sendTo32u4Data.soundcmd = 5;
  else if (buttonsL.right) sendTo32u4Data.soundcmd = 6;
  else if (buttonsL.down) sendTo32u4Data.soundcmd = 7;
  else if (buttonsL.left) sendTo32u4Data.soundcmd = 8;
  else if (buttonsR.circle) sendTo32u4Data.soundcmd = 9;
  else sendTo32u4Data.soundcmd = 0;

  if (BTN_PRESSED(ps,R)) {
    enableDrive = !enableDrive;
    sendTo32u4Data.driveEnabled = enableDrive;
  }
  if (BTN_PRESSED(l3,L)) {
    DomeServoMode = !DomeServoMode;
    sendTo32u4Data.moveR3 = DomeServoMode;
  }
  if (BTN_PRESSED(l3,R)) {
    reverseDrive = !reverseDrive;
    sendTo32u4Data.moveL3 = reverseDrive;
  }
  if (BTN_PRESSED(cross,R)) {
    autoBalance = !autoBalance;
    Serial.printf("T: %lu Auto Balance %s\n", millis(), autoBalance?"ON":"OFF");
  }

  sendTo32u4Data.leftStickX = buttonsL.leftStickX;
  sendTo32u4Data.leftStickY = buttonsL.leftStickY;
  sendTo32u4Data.psiFlash = false;  // Set as needed

  prevR = buttonsR; prevL = buttonsL;
}

/* ------------------- S2S MOVEMENT ------------------- */
void S2S_Movement() {
  unsigned long now = millis();
  float dt = (lastS2STime > 0) ? (now - lastS2STime) / 1000.0f : 0.01f;
  lastS2STime = now;

  int rawPot   = analogRead(S2S_POT_PIN);
  int potValue = filterPotValue(rawPot);
  potValue     = constrain(potValue, S2S_POT_MIN, S2S_POT_MAX);

  int joyX = buttonsR.rightStickX;

  int pwm = 0;
  bool dir1 = false, dir2 = false;
  s2sMode = S2SMode::STOP;

  if (abs(joyX) > JOYSTICK_DEADZONE) {
    // JOYSTICK CONTROL
    pwm = map(abs(joyX), 0, 127, 0, 255);
    dir1 = (joyX < 0) ? REVERSE_S2S : !REVERSE_S2S;
    dir2 = (joyX < 0) ? !REVERSE_S2S : REVERSE_S2S;
    s2sMode = S2SMode::JOY;
    PID2_S2S.SetMode(MANUAL);
    Output2 = 0;
  } else {
    if (autoBalance && IMUconnected) {
      // PID BALANCE MODE - USE ROLL ANGLE
      Input2 = receiveIMUData.roll - rollOffset;
      if (abs(Input2) > IMU_DEADZONE) {
        PID2_S2S.SetMode(AUTOMATIC);
        PID2_S2S.Compute();

        Output2_S2S_pwm = constrain(abs(Output2), MIN_RETURN_PWM, MAX_RETURN_PWM);
        pwm = (int)Output2_S2S_pwm;

        dir1 = (Output2 > 0) ? REVERSE_S2S : !REVERSE_S2S;
        dir2 = (Output2 > 0) ? !REVERSE_S2S : REVERSE_S2S;
        s2sMode = S2SMode::RTN;
      } else {
        PID2_S2S.SetMode(MANUAL);
        Output2 = 0;
      }
    } else {
      // STANDARD RETURN-TO-CENTER (pot-based)
      int target = potOffset;
      if (ENABLE_ROLL_BIAS) target += (int)(sendTo32u4Data.roll * ROLL_BIAS_GAIN);
      int error = potValue - target;
      int candidate = constrain(abs(error) * 2, 0, MAX_RETURN_PWM);
      if (candidate >= MIN_RETURN_PWM && abs(error) > RETURN_DEADBAND) {
        pwm = candidate;
        dir1 = (error < 0) ? REVERSE_S2S : !REVERSE_S2S;
        dir2 = (error < 0) ? !REVERSE_S2S : REVERSE_S2S;
        s2sMode = S2SMode::RTN;
      }
    }
  }

  if (controllerConnected && enableDrive && !buttonsL.l1 && !buttonsR.l1) {
    digitalWrite(S2S_PIN_1, dir1 ? HIGH : LOW);
    digitalWrite(S2S_PIN_2, dir2 ? HIGH : LOW);
    analogWrite(S2S_PWM, pwm);
    lastPwmS2S = pwm;
  } else {
    digitalWrite(S2S_PIN_1, LOW);
    digitalWrite(S2S_PIN_2, LOW);
    analogWrite(S2S_PWM, 0);
    lastPwmS2S = 0;
  }
}

/* ------------------- DRIVE + PID ------------------- */
void drive_Movement() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  int joyY = reverseDrive ? -buttonsR.rightStickY : buttonsR.rightStickY;
  int baseSpeed = map(joyY, -127, 127, -255, 255);
  int finalSpeed = baseSpeed;

  if (autoBalance && IMUconnected && abs(buttonsR.rightStickY) <= JOYSTICK_DEADZONE) {
    float pitchError = receiveIMUData.pitch - pitchOffset;
    if (abs(pitchError) > IMU_DEADZONE) {
      float P = KP_PITCH * pitchError;
      pitchIntegral += pitchError * dt;
      pitchIntegral = constrain(pitchIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
      float I = KI_PITCH * pitchIntegral;
      float D = KD_PITCH * (pitchError - prevPitchError) / dt;
      prevPitchError = pitchError;
      finalSpeed += (int)(P + I + D);
    } else {
      pitchIntegral *= 0.95;
    }
  } else {
    pitchIntegral = 0.0;
    prevPitchError = 0.0;
  }

  finalSpeed = constrain(finalSpeed, -255, 255);

  if (controllerConnected && enableDrive && abs(finalSpeed) > 5) {
    digitalWrite(DRIVE_PIN_1, finalSpeed < 0 ? HIGH : LOW);
    digitalWrite(DRIVE_PIN_2, finalSpeed < 0 ? LOW : HIGH);
    analogWrite(DRIVE_PWM, abs(finalSpeed));
  } else {
    digitalWrite(DRIVE_PIN_1, LOW);
    digitalWrite(DRIVE_PIN_2, LOW);
    analogWrite(DRIVE_PWM, 0);
  }
}

/* ------------------- FLYWHEEL ------------------- */
void spinFlywheel() {
  if (EnableFlywheel && enableDrive && buttonsL.l1) {
    flywheel = constrain(map(buttonsR.rightStickX, -128, 128,
                reverseDrive ? 255 : -255,
                reverseDrive ? -255 : 255), -255, 255);
    if (abs(flywheel) > 10) {
      digitalWrite(FLYWHEEL_PIN_A, flywheel > 0 ? HIGH : LOW);
      digitalWrite(FLYWHEEL_PIN_B, flywheel > 0 ? LOW : HIGH);
      analogWrite(FLYWHEEL_PWM, abs(flywheel));
    } else {
      digitalWrite(FLYWHEEL_PIN_A, LOW);
      digitalWrite(FLYWHEEL_PIN_B, LOW);
      analogWrite(FLYWHEEL_PWM, 0);
    }
  } else {
    digitalWrite(FLYWHEEL_PIN_A, LOW);
    digitalWrite(FLYWHEEL_PIN_B, LOW);
    analogWrite(FLYWHEEL_PWM, 0);
  }
}

/* ------------------- TUNING MODE ------------------- */
void handleTuningModeEntry() {
  static bool wasPSHeld = false;

  if (buttonsR.ps) {
    if (!wasPSHeld) psHoldStart = millis();
    wasPSHeld = true;

    if (buttonsR.up && (millis() - psHoldStart >= PS_HOLD_TIME) && !tuningMode) {
      startTuningMode(true);
      return;
    }
    if (buttonsR.right && (millis() - psHoldStart >= PS_HOLD_TIME) && !tuningMode) {
      startTuningMode(false);
      return;
    }
  } else {
    wasPSHeld = false;
    psHoldStart = 0;
  }
}

void startTuningMode(bool pitchMode) {
  tuningMode = true;
  tuneState = pitchMode ? TuneState::PITCH_KP : TuneState::ROLL_KP;
  autoBalance = true;
  enableDrive = true;

  if (pitchMode) {
    tempKP_PITCH = 45.0f; tempKD_PITCH = 0.0f; tempKI_PITCH = 0.0f;
    KP_PITCH = tempKP_PITCH; KI_PITCH = 0.0f; KD_PITCH = 0.0f;
    Serial.println(F("\n=== PITCH PID TUNING MODE ==="));
  } else {
    tempPk2 = 45.0; tempDk2 = 0.0; tempIk2 = 0.0;
    Pk2 = tempPk2; Ik2 = 0.0; Dk2 = 0.0;
    Serial.println(F("\n=== ROLL PID TUNING MODE ==="));
  }
}

void handleTuningMode() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();

  bool up = buttonsR.up, down = buttonsR.down;
  bool x = buttonsR.cross && !prevR.cross;

  if (up || down) {
    float step = (tuneState == TuneState::PITCH_KP || tuneState == TuneState::ROLL_KP) ? 5.0 :
                 (tuneState == TuneState::PITCH_KD || tuneState == TuneState::ROLL_KD) ? 1.0 : 0.1;

    if (tuneState == TuneState::PITCH_KP) { tempKP_PITCH += up ? step : -step; KP_PITCH = tempKP_PITCH; }
    else if (tuneState == TuneState::PITCH_KD) { tempKD_PITCH += up ? step : -step; KD_PITCH = tempKD_PITCH; }
    else if (tuneState == TuneState::PITCH_KI) { tempKI_PITCH += up ? step : -step; KI_PITCH = tempKI_PITCH; }
    else if (tuneState == TuneState::ROLL_KP) { tempPk2 += up ? step : -step; Pk2 = tempPk2; }
    else if (tuneState == TuneState::ROLL_KD) { tempDk2 += up ? step : -step; Dk2 = tempDk2; }
    else if (tuneState == TuneState::ROLL_IK) { tempIk2 += up ? step : -step; Ik2 = tempIk2; }
  }

  if (x) {
    if (tuneState == TuneState::PITCH_KP || tuneState == TuneState::ROLL_KP) {
      if (tuneState == TuneState::PITCH_KP) {
        KP_PITCH = tempKP_PITCH * 0.75f;
        Serial.printf("KP_PITCH FINAL: %.2f\n", KP_PITCH);
      } else {
        Pk2 = tempPk2 * 0.75;
        Serial.printf("Pk2 FINAL: %.2f\n", Pk2);
      }
      tuneState = tuneState == TuneState::PITCH_KP ? TuneState::PITCH_KD : TuneState::ROLL_KD;
    }
    else if (tuneState == TuneState::PITCH_KD || tuneState == TuneState::ROLL_KD) {
      tuneState = tuneState == TuneState::PITCH_KD ? TuneState::PITCH_KI : TuneState::ROLL_IK;
    }
    else {
      tuningMode = false;
      tuneState = TuneState::IDLE;
      Serial.println("=== PID TUNING COMPLETE ===");
      Serial.printf("PITCH: KP=%.2f KI=%.2f KD=%.2f\n", KP_PITCH, KI_PITCH, KD_PITCH);
      Serial.printf("ROLL:  Pk2=%.2f Ik2=%.2f Dk2=%.2f\n", Pk2, Ik2, Dk2);
    }
  }
  prevR = buttonsR;
}

void printTuningDebug() {
  static unsigned long last = 0;
  if (millis() - last < 250) return;  // 4 Hz update — smooth & readable
  last = millis();

  const char* mode = (tuneState <= TuneState::PITCH_KI) ? "PITCH" : "ROLL";

  // Current values (live during tuning)
  float curP = (tuneState <= TuneState::PITCH_KI) ? KP_PITCH : Pk2;
  float curI = (tuneState <= TuneState::PITCH_KI) ? KI_PITCH : Ik2;
  float curD = (tuneState <= TuneState::PITCH_KI) ? KD_PITCH : Dk2;

  // Final stored values (after X press)
  float finalP = (tuneState <= TuneState::PITCH_KI) ? 
                 (tuneState > TuneState::PITCH_KP ? KP_PITCH : tempKP_PITCH * 0.75f) :
                 (tuneState > TuneState::ROLL_KP ? Pk2 : tempPk2 * 0.75);

  // Live error & PID output
  float pitchError = receiveIMUData.pitch - pitchOffset;
  float rollError  = receiveIMUData.roll  - rollOffset;
  float driveOut = 0, s2sOut = 0;
  if (autoBalance && IMUconnected) {
    // Approximate current PID output (good enough for tuning)
    driveOut = KP_PITCH * pitchError + KD_PITCH * (pitchError - prevPitchError);
    s2sOut   = Output2;  // from roll PID
  }

  Serial.printf(
    "\r%s TUNING | P:%.2f→%.2f  I:%.2f  D:%.2f  |  ERR P:%.1f R:%.1f  |  OUT D:%+4.0f S:%+4.0f  |  X=Next ",
    mode,
    curP, finalP,
    curI, curD,
    pitchError, rollError,
    driveOut, s2sOut
  );

  // Highlight current tuning parameter
  if (tuneState == TuneState::PITCH_KP || tuneState == TuneState::ROLL_KP)
    Serial.print("← P ");
  else if (tuneState == TuneState::PITCH_KD || tuneState == TuneState::ROLL_KD)
    Serial.print("← D ");
  else if (tuneState == TuneState::PITCH_KI || tuneState == TuneState::ROLL_IK)
    Serial.print("← I ");
}

/* ------------------- DEBUG OUTPUT ------------------- */
void printDebugInfo() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug < 1000) return;
  lastDebug = millis();

#ifdef DEBUG_ALL
  #define DEBUG_CALIBRATION
  #define DEBUG_JOYSTICK
  #define DEBUG_S2S_MODE
  #define DEBUG_MOTOR_OUTPUT
  #define DEBUG_IMU_RAW
#endif

#if defined(DEBUG_CALIBRATION) || defined(DEBUG_JOYSTICK) || defined(DEBUG_S2S_MODE) || defined(DEBUG_MOTOR_OUTPUT) || defined(DEBUG_IMU_RAW)
  Serial.printf("\n=== DEBUG T:%lu ===\n", millis());
#endif

#ifdef DEBUG_CALIBRATION
  int currentPot = filterPotValue(analogRead(S2S_POT_PIN));
  currentPot = constrain(currentPot, S2S_POT_MIN, S2S_POT_MAX);
  float adjPitch = receiveIMUData.pitch - pitchOffset;
  float adjRoll  = receiveIMUData.roll  - rollOffset;

  Serial.printf("POT: Curr=%4d  SavedCenter=%4d  Err=%+4d\n", 
                currentPot, potOffset, currentPot - potOffset);
  Serial.printf("IMU: Pitch=%+6.2f (offset=%+6.2f)  Roll=%+6.2f (offset=%+6.2f)  Connect=%s\n",
                adjPitch, pitchOffset, adjRoll, rollOffset, IMUconnected?"YES":"NO");
#endif

#ifdef DEBUG_JOYSTICK
  Serial.printf("DRIVEJOY: X=%+4d  Y=%+4d  (Deadzone=%d)\n",
                buttonsR.rightStickX, buttonsR.rightStickY, JOYSTICK_DEADZONE);
  Serial.printf("DOMEJOY: X=%+4d  Y=%+4d  (Deadzone=%d)\n",
                buttonsL.leftStickX, buttonsL.leftStickY, JOYSTICK_DEADZONE);
#endif

#ifdef DEBUG_S2S_MODE
  const char* modeStr = 
    (s2sMode == S2SMode::JOY) ? "JOY MANUAL" :
    (autoBalance && IMUconnected && abs(receiveIMUData.roll - rollOffset) > IMU_DEADZONE) ? "PID BALANCE" :
    "POT RETURN";
  Serial.printf("S2S MODE: %s  |  AutoBal=%s  DriveEn=%s\n",
                modeStr, autoBalance?"ON":"OFF", enableDrive?"ON":"OFF");
#endif

#ifdef DEBUG_MOTOR_OUTPUT
  Serial.printf("MOTORS → S2S: PWM=%3d  DIR=%d%d  |  DRIVE: PWM=%3d\n",
                lastPwmS2S,
                digitalRead(S2S_PIN_1), digitalRead(S2S_PIN_2),
                0);  // Add drive PWM tracking if desired
#endif

#ifdef DEBUG_IMU_RAW
  Serial.printf("IMU RAW: Pitch=%+6.2f  Roll=%+6.2f\n",
                receiveIMUData.pitch, receiveIMUData.roll);
#endif

#if defined(DEBUG_CALIBRATION) || defined(IMUDEBUG) || defined(DEBUG_JOYSTICK) || defined(DEBUG_S2S_MODE) || defined(DEBUG_MOTOR_OUTPUT) || defined(DEBUG_IMU_RAW)
  Serial.println("=====================================");
#endif

#ifdef IMUDEBUG
  static unsigned long last = 0;
  if (millis() - last < 500) return;  // 2 Hz update
  last = millis();

  float rawPitch = receiveIMUData.pitch;
  float rawRoll  = receiveIMUData.roll;
  float adjPitch = rawPitch - pitchOffset;
  float adjRoll  = rawRoll  - rollOffset;

  Serial.printf(
    "\n=== IMU DEBUG ===\n"
    "RAW:     Pitch=%+8.3f°   Roll=%+8.3f°\n"
    "OFFSET:  Pitch=%+8.3f°   Roll=%+8.3f°\n"
    "ADJUSTED:Pitch=%+8.3f°   Roll=%+8.3f°\n"
    "STATUS:  %s  |  AutoBal=%s\n"
    "=================\n",
    rawPitch, rawRoll,
    pitchOffset, rollOffset,
    adjPitch, adjRoll,
    IMUconnected ? "CONNECTED" : "LOST    ",
    autoBalance ? "ON " : "OFF"
  );
#endif

}

/* ------------------- LOOP ------------------- */
void loop() {}