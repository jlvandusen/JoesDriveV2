/*
  JOE'S DRIVE V2 | Z-Drive v8.9 - PURE JOYSTICK TEST (DFRobot 12A) - FINAL v8.9b
  Right Stick X → S2S motor (exactly like v8.7)
  Right Stick Y → Drive motor (exactly like v8.7)
  Signed speed mapping -127..+127 → -255..+255
*/

#include <Arduino.h>

#include <PSController.h>
#include <analogWrite.h>

// ------------------- YOUR EXACT PINS FROM v8.7 -------------------
const uint8_t S2S_PWM      = 33;
const uint8_t S2S_PIN_1    = 26;
const uint8_t S2S_PIN_2    = 25;

const uint8_t DRIVE_PWM    = 21;
const uint8_t DRIVE_PIN_1  = 4;
const uint8_t DRIVE_PIN_2  = 27;

const char* MASTER_NAV = "7c:9e:bd:d7:63:c6";   // Your working MAC

const int DEADZONE = 20;

// PSController instances exactly like v8.7
PSController driveController(nullptr);
PSController domeController(nullptr);

int16_t joyX = 0;   // -127 to +127
int16_t joyY = 0;   // -127 to +127
bool controllerConnected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Motor pins
  pinMode(S2S_PWM,     OUTPUT);
  pinMode(S2S_PIN_1,   OUTPUT);
  pinMode(S2S_PIN_2,   OUTPUT);
  pinMode(DRIVE_PWM,   OUTPUT);
  pinMode(DRIVE_PIN_1, OUTPUT);
  pinMode(DRIVE_PIN_2, OUTPUT);

  // Start BT exactly like v8.7
  if (!PSController::startListening(MASTER_NAV)) {
    Serial.println("FATAL: BT failed!");
    while (true) delay(100);
  }

  Serial.println("\nJOYSTICK-ONLY TEST - v8.7 STYLE");
  Serial.println("Right Stick X = S2S | Right Stick Y = Drive");
  Serial.println("Signed mapping: -127..+127 → -255..+255");
  Serial.println("Waiting for controller...\n");
}

void loop() {
  readController();
  driveMotors();
  printDebug();
  delay(20);
}

void readController() {
  bool driveConn = driveController.isConnected();
  bool domeConn  = domeController.isConnected();
  controllerConnected = driveConn || domeConn;

  if (controllerConnected) {
    joyX = driveController.state.analog.stick.lx;   // -127 to +127
    joyY = driveController.state.analog.stick.ly;

    if (abs(joyX) < DEADZONE) joyX = 0;
    if (abs(joyY) < DEADZONE) joyY = 0;
  } else {
    joyX = joyY = 0;
  }
}

void driveMotors() {
  // =================== S2S MOTOR ===================
  int s2sSpeed = map(joyX, -127, 127, -255, 255);   // Signed speed

  if (controllerConnected && abs(s2sSpeed) > 5) {
    bool dir1 = (s2sSpeed >= 0);   // Positive = one direction
    bool dir2 = (s2sSpeed <  0);   // Opposite direction
    digitalWrite(S2S_PIN_1, dir1 ? HIGH : LOW);
    digitalWrite(S2S_PIN_2, dir2 ? HIGH : LOW);
    analogWrite(S2S_PWM, abs(s2sSpeed));
  } else {
    digitalWrite(S2S_PIN_1, LOW);
    digitalWrite(S2S_PIN_2, LOW);
    analogWrite(S2S_PWM, 0);
  }

  // =================== DRIVE MOTOR ===================
  int driveSpeed = map(joyY, -127, 127, -255, 255);  // Signed speed

  if (controllerConnected && abs(driveSpeed) > 5) {
    digitalWrite(DRIVE_PIN_1, driveSpeed < 0 ? HIGH : LOW);
    digitalWrite(DRIVE_PIN_2, driveSpeed < 0 ? LOW  : HIGH);
    analogWrite(DRIVE_PWM, abs(driveSpeed));
  } else {
    digitalWrite(DRIVE_PIN_1, LOW);
    digitalWrite(DRIVE_PIN_2, LOW);
    analogWrite(DRIVE_PWM, 0);
  }
}

void printDebug() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  last = millis();

  int s2sOut = map(joyX, -127, 127, -255, 255);
  int drvOut = map(joyY, -127, 127, -255, 255);

  Serial.printf(
    "T:%4lu | JOY X:%+4d → S2S %+4d (PWM %3d) | JOY Y:%+4d → DRV %+4d (PWM %3d) | PS:%s\r\n",
    millis(),
    joyX, s2sOut, abs(s2sOut),
    joyY, drvOut, abs(drvOut),
    controllerConnected ? "OK" : "NO"
  );
}