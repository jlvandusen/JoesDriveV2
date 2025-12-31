
/*
 * Joe's Drive – V9.1 (DFPlayer + NeoPixel + pin updates + audio abstraction)
 * Secondary 32u4 – Dome Movement & PSI Lights
 * Secondary Feather M0 Basic (SAMD21) Optional
 * - 32u4 Feather <--> ESP32 Huzzah on Serial1 (EasyTransfer)
 * - DFPlayer Mini via SoftwareSerial on A4(22)/D9 + BUSY on D10
 * - NeoPixel DIN on D13
 * - Dome Motor: PWM D3 (Timer3), DirA D5, DirB D6
 * - Servos: D11 (right), D12 (left)
 * - Hall: A2 (20)
 * - Encoder: choose INTERRUPT (D2/D4) or POLLING (A0/A1)
 *
 * Audio: Select one (#define AUDIO_SPARKFUN or #define AUDIO_DFPLAYER).
 * If neither is defined, AUDIO_NONE stubs compile & run silently.
 *
 * BOARDS: Install the Adafruit SAMD Library following this URL: 
 * https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup
 * Add the Adafruit SleepyDog Library for Watchdog replacement
 *
 */

 /*
  Feather 32u4 Pin Assignments
  Serial Communication
  Pin 0 (RX1) → ESP32 TX (EasyTransfer)
  Pin 1 (TX1) → ESP32 RX (EasyTransfer)

  DFPlayer Mini
  Pin D9 → DFPlayer TX (via voltage divider: 20 kΩ / 10 kΩ)
  Pin A4 → DFPlayer RX (via 1 kΩ series resistor)
  Pin D10 → DFPlayer BUSY (active LOW; INPUT_PULLUP or divider if BUSY = 5 V idle)

  NeoPixel
  Pin D13 → NeoPixel DIN (through SparkFun BOB‑12009 level shifter LV1 → HV1 → 330 Ω series resistor)
  BOB‑12009 LV → Feather 3.3 V
  BOB‑12009 HV → External 5 V rail
  GND → Common ground
  Power: External 5 V with 1000 µF capacitor across 5 V/GND near strip

  Encoder (NeveRest Classic 60)
  Phase A → D2 (interrupt source)
  Phase B → A0 (read inside ISR)
  Pull-ups: 4.7–10 kΩ to 3.3 V on A and B if encoder outputs are open-collector

  Hall Sensor
  Pin A2 → Hall sensor input (use INPUT_PULLUP)

  Dome Motor
  Pin D3 → PWM (Timer3)
  Pin D5 → Direction A
  Pin D6 → Direction B

  Servos
  Pin D11 → Right servo signal
  Pin D12 → Left servo signal

  Board Type Use
  You can comment out MODERNBOARD to utilize Legacy v8.2 board with i2c to SPARKFUN MP3 trigger
  Or v9.1b with major pin changes as mentioned above

  NOTE: Primary sketch can still run v9.1 always as it supports both boards 8.2 and 9.1

*/

// debugging the watch dog service...
// #define DEBUG_WATCHDOG

// ------------------- FEATURE SWITCHES -------------------
#define MOVECONTROLLER
#define UseHallMonitor
#define MODERNBOARD // Comment this line for ALTERNATE compatibility - note must require Sparkfun MP3 trigger (i2c)

// Enable NeoPixel strip support:
#define USE_NEOPIXEL

// ---- AUDIO DEVICE SELECTION ----
// Uncomment ONE if you want sound:
// #define AUDIO_SPARKFUN        // SparkFun Qwiic MP3 Trigger (I2C) from 32u4
#define AUDIO_DFPLAYER          // DFPlayer Mini on SoftwareSerial pins 22 and 9 to be used with v8.9 Motherboard only



// Checks and balances for various definitions
#if defined(AUDIO_SPARKFUN) && defined(AUDIO_DFPLAYER)
  #error "Enable only one audio device: AUDIO_SPARKFUN or AUDIO_DFPLAYER"
#endif

// Require AUDIO_SPARKFUN
#if defined(AUDIO_DFPLAYER) && !defined(MODERNBOARD)
  #error "Enable only AUDIO_SPARKFUN AND do not use AUDIO_DFPLAYER"
#endif

// If neither device is selected, compile with no audio
#if !defined(AUDIO_SPARKFUN) && !defined(AUDIO_DFPLAYER)
  #define AUDIO_NONE
#endif

#if defined(MODERNBOARD) // Select one encoder method:
  #define ENCODER_METHOD_INTERRUPT // Use of pins D2 (INT1) + D4
#else
  #define ENCODER_METHOD_POLLING // use for v8.2 backwards compatibility on pins A0(18) + A1(19) for encoder
#endif



// ------------------- PIN DEFINITIONS (per your map) -------------------
/* DFPlayer Mini */
#if defined(AUDIO_DFPLAYER)  && defined(MODERNBOARD)
  #define DFPLAYER_TX_PIN   22    // Feather A4 (TX from Feather -> DFPlayer RX) through ~1kΩ
  #define DFPLAYER_RX_PIN   9     // Feather D9 (RX to Feather <- DFPlayer TX)
  #define DFPLAYER_BUSY_PIN 10    // Active LOW while playing (consider 10k pull-up to 5V)
  #define DFPLAYER_BAUD     9600
  #define AUDIO_DEFAULT_VOLUME 25
#endif

/* NeoPixel Strip */
#if defined(USE_NEOPIXEL)
  #define NEO_PIN           13    // DIN
  #define NEO_NUM_PIXELS    16    // <-- set to your actual count
#endif


/* Dome Motor (DFRobot Controller) */
#ifndef MODERNBOARD
#define domeMotor_pwm        10
#define domeMotor_pin_A      9
#define domeMotor_pin_B      6
#else
#define domeMotor_pwm        3    // PWM (Timer3)
#define domeMotor_pin_A      5    // Direction A
#define domeMotor_pin_B      6    // Direction B
#endif

/* Servos (HS-805BB x2) */
#define rightServo_pin       11   // Servo 1 signal
#define leftServo_pin        12   // Servo 2 signal

/* Hall Effect Sensor */
#define hallEffectSensor_Pin 20   // A2 (INPUT_PULLUP)

/* Encoder (NeveRest Classic 60) */
#if defined(ENCODER_METHOD_INTERRUPT)
  #define motorEncoder_pin_A 2    // Phase A -> D2 (INT1)
  #define motorEncoder_pin_B A0   // Phase B -> A0 (read inside ISR)
#elif defined(ENCODER_METHOD_POLLING)
  #define motorEncoder_pin_A 18   // A0 (PCINT7)
  #define motorEncoder_pin_B 19   // A1 (PCINT6)
#endif

// ------------------- CONSTANTS -------------------
#define servoSpeed                255
#define servoEase                 10
#define domeTiltYAxis_MaxAngle    12
#define domeTiltXAxis_MaxAngle    12
#define printMillis               100
#define leftServoOffset           -7
#define rightServoOffset           0
#define TICKS_PER_REV             1680
#define CENTER_TICK               878        // YOUR ACTUAL CENTER
#define MAX_ANGLE_TICKS           140
#define MP3_INIT_TIMEOUT          5000
#define SERVO_UPDATE_INTERVAL     5
#define DOME_CENTER_TIMEOUT       10000
#define HALL_DEBOUNCE             50
#define CENTER_TOLERANCE          15
#define MOTOR_SPEED               100
#define FIND_CENTER_SPEED         50
#define ENC_MIN_INTERVAL          500      // µs debounce
#define DRIFT_CHECK_INTERVAL      5000     // ms

// ---- AUDIO ABSTRACTION PROTOTYPES ----
bool audioInit(unsigned long timeoutMs = 5000);
void audioStop();
void audioPlay(uint16_t track);
bool audioIsPlaying();

// ------------------- LIBRARIES -------------------

#if defined(__AVR__)
  #include <VarSpeedServo.h>
  // VarSpeedServo myservo1, myservo2;
#elif defined(ARDUINO_ARCH_SAMD)
  #include <Servo.h>
  // Servo myservo1, myservo2;
#endif

#include <EasyTransfer.h>
#include <PID_v1.h>

#if defined(__AVR__)
// AVR-specific includes and code
#include <avr/wdt.h>
#define USE_AVR_WATCHDOG
#endif

#if defined(ARDUINO_ARCH_SAMD)
// SAMD-specific includes and code
#include <Adafruit_SleepyDog.h>
#define USE_SAMD_WATCHDOG
#endif


#if defined(AUDIO_SPARKFUN)
  #include <Wire.h>
  #include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h"
  MP3TRIGGER mp3;                              // SparkFun object
  const uint8_t SPARKFUN_MP3_I2C_ADDR = MP3TRIGGER_ADDRESS; // default 0x37
#endif

#if defined(AUDIO_DFPLAYER)
  #include "DFRobotDFPlayerMini.h"
  #if defined(__AVR__)
  // AVR boards (32u4): Use SoftwareSerial on pins 9 (RX) and 22 (TX)
  #include <SoftwareSerial.h>
  SoftwareSerial mp3Serial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN); // RX, TX

  #elif defined(ARDUINO_ARCH_SAMD)
  // SAMD boards (Feather M0): Use hardware Serial1
  // Serial1 is mapped to pins 0 (RX) and 1 (TX) by default on Feather M0
  // But we need pins 9 and 22, so we use SERCOM remapping
  #include "wiring_private.h" // For pinPeripheral()

  #define DFPLAYER_RX_PIN 9   // RX from DFPlayer TX
  #define DFPLAYER_TX_PIN 22  // TX to DFPlayer RX

  Uart mp3Serial(&sercom3, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);

  #endif

  DFRobotDFPlayerMini mp3;                   // DFPlayer object

#endif

#if defined(USE_NEOPIXEL)
  #include <Adafruit_NeoPixel.h>
#endif

// ------------------- GLOBALS -------------------
volatile long encPos = CENTER_TICK;
volatile unsigned long lastEncInterrupt = 0;

double Input_domeSpinServoPid, Output_domeSpinServoPid, Setpoint_domeSpinServoPid = 0;
int domeServoPWM;

#if defined(__AVR__)
  VarSpeedServo myservo1, myservo2;
#elif defined(ARDUINO_ARCH_SAMD)
  Servo myservo1, myservo2;
#endif

EasyTransfer recESP32, sendESP32;
PID myPID_domeSpinServoPid(&Input_domeSpinServoPid, &Output_domeSpinServoPid,
                           &Setpoint_domeSpinServoPid, 3.5, 0.8, 0.05, DIRECT);

int16_t leftServo_0_Position  = 70 + leftServoOffset;
int16_t rightServo_0_Position = 110 + rightServoOffset;
double leftServoPosition  = leftServo_0_Position;
double rightServoPosition = rightServo_0_Position;
double leftOldPosition    = leftServo_0_Position;
double rightOldPosition   = rightServo_0_Position;
double domeTiltAngle_X_Axis, domeTiltAngle_Y_Axis, leftStickY, leftStickX;

bool domeCenterSet = false, domeServoMode = false, enableDrive = false, reverseDrive = false;

// Shared audio state (visible to the whole sketch)
bool     sndplaying   = false;
int8_t   soundcmd     = 0;
uint16_t randomsound  = 0;  // will be set in audioInit()

unsigned long currentMillis, receiveMillis, lastPrintMillis, lastServoUpdateMillis;
unsigned long domeCenterStartMillis, lastHallLowMillis;
unsigned long lastDriftCheck = 0;
long lastStableEnc = CENTER_TICK;

#if defined(USE_NEOPIXEL)
  Adafruit_NeoPixel strip(NEO_NUM_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
#endif

// ------------------- DATA STRUCTURES -------------------
struct RECEIVE_DATA_STRUCTURE {
  bool   driveEnabled;
  int8_t domeSpin;
  bool   moveL3;
  bool   moveR3;
  int8_t leftStickX;
  int8_t leftStickY;
  int8_t soundcmd;
  int8_t psiFlash;   // reuse for simple NeoPixel effect trigger
  float  pitch;
  float  roll;
} receiveFromESP32Data;

struct SEND_DATA_STRUCTURE {
  int16_t tiltAngle;
  bool    sndplaying;
} sendToESP32Data;

// ------------------- ENCODER ISR/Handlers -------------------
#if defined(ENCODER_METHOD_INTERRUPT)
// External interrupt on D2 (INT1), read Phase B on D4
void encoderISR() {
  unsigned long now = micros();
  if (now - lastEncInterrupt < ENC_MIN_INTERVAL) return;
  lastEncInterrupt = now;

  uint8_t a = digitalRead(motorEncoder_pin_A);
  uint8_t b = digitalRead(motorEncoder_pin_B);
  // Basic quadrature: if A changed, compare B for direction
  if (a == b) encPos++; else encPos--;
  encPos = (encPos % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV;
}
#else
// Original PCINT on A0/A1 (polling inside ISR)
ISR(PCINT0_vect) {
  unsigned long now = micros();
  if (now - lastEncInterrupt < ENC_MIN_INTERVAL) return;
  lastEncInterrupt = now;

  static uint8_t lastState = 0;
  uint8_t a = digitalRead(motorEncoder_pin_A);
  uint8_t b = digitalRead(motorEncoder_pin_B);
  uint8_t state = (a << 1) | b;

  static const int8_t transitionTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  int8_t delta = transitionTable[(lastState << 2) | state];
  if (delta != 0) {
    encPos += delta;
    encPos = (encPos % TICKS_PER_REV + TICKS_PER_REV) % TICKS_PER_REV;
  }
  lastState = state;
}
#endif

// ------------------- SETUP -------------------
void setup() {
 // Enable watchdog with ~4s timeout 
  #if defined(USE_AVR_WATCHDOG)
    wdt_enable(WDTO_4S);
  #elif defined(USE_SAMD_WATCHDOG)
    Watchdog.enable(4500);
  #endif

  pinMode(LED_BUILTIN, OUTPUT); // Optional heartbeat LED
  Serial.begin(115200);
  Serial1.begin(74880);           // ESP32 EasyTransfer

#if defined(USE_NEOPIXEL)
  strip.begin();
  strip.show(); // All off
#endif

  // Servos
  myservo2.attach(leftServo_pin);
  myservo1.attach(rightServo_pin);
  myservo2.write(leftServo_0_Position);
  myservo1.write(rightServo_0_Position);

  // EasyTransfer
  recESP32.begin(details(receiveFromESP32Data), &Serial1);
  sendESP32.begin(details(sendToESP32Data), &Serial1);

  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);

  myPID_domeSpinServoPid.SetMode(AUTOMATIC);
  myPID_domeSpinServoPid.SetOutputLimits(-255, 255);

  pinMode(domeMotor_pwm,   OUTPUT);
  pinMode(domeMotor_pin_A, OUTPUT);
  pinMode(domeMotor_pin_B, OUTPUT);

#if defined(ENCODER_METHOD_INTERRUPT)
  pinMode(motorEncoder_pin_A, INPUT_PULLUP);
  pinMode(motorEncoder_pin_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motorEncoder_pin_A), encoderISR, CHANGE);
#else
  pinMode(motorEncoder_pin_A, INPUT_PULLUP);
  pinMode(motorEncoder_pin_B, INPUT_PULLUP);
  // Enable PCINT on A0 (PCINT7) and A1 (PCINT6)
  PCMSK0 |= (1 << PCINT7) | (1 << PCINT6);
  PCICR  |= (1 << PCIE0);
#endif

#ifdef UseHallMonitor
  domeCenterStartMillis = millis();
  // while (!domeCenterSet && millis() - domeCenterStartMillis < DOME_CENTER_TIMEOUT) {
  //   setDomeCenter();
  // }
  if (!domeCenterSet) Serial.println(F("Centering timeout"));
#else
  domeCenterSet = true;
  encPos = CENTER_TICK;
#endif

  Setpoint_domeSpinServoPid = 0;
  lastDriftCheck = millis();
  lastStableEnc = encPos;

  // ---- AUDIO INIT (unified) ----
  (void)audioInit(MP3_INIT_TIMEOUT);

  Serial.println(F("JOE'S DRIVE V9.15 — 32u4 SLAVE — SER — FULLY READY"));
}

// ------------------- MAIN LOOP -------------------
void loop() {
  // Reset watchdog every loop
  #if defined(USE_AVR_WATCHDOG)
    wdt_reset();
  #elif defined(USE_SAMD_WATCHDOG)
    Watchdog.reset();
  #endif

  Timechecks();
  SendRecieveData();
  DriftMonitor();  // Auto-snap to center

  if (enableDrive) {
    attemptDomeCentering();
    Servos();
    spinStuff();
    if (!domeCenterSet && domeServoMode && domeServoPWM == 0) setDomeCenter();
  }
  mp3play();

#if defined(USE_NEOPIXEL)
  neoUpdate();
#endif
}


void attemptDomeCentering() {
  if (!domeCenterSet && millis() - domeCenterStartMillis < DOME_CENTER_TIMEOUT) {
    setDomeCenter();
  } else if (!domeCenterSet) {
    Serial.println(F("Centering timeout"));
    domeCenterSet = true; // Fail-safe
  }
}

// ------------------- DRIFT MONITOR (AUTO-CORRECT) -------------------
void DriftMonitor() {
  if (millis() - lastDriftCheck >= DRIFT_CHECK_INTERVAL) {
    if (domeCenterSet && digitalRead(hallEffectSensor_Pin) == LOW && abs(encPos - lastStableEnc) < 3) {
      encPos = CENTER_TICK;  // Snap to center
      Serial.println(F("Drift corrected: snapped to 878"));
    }
    lastStableEnc = encPos;
    lastDriftCheck = millis();
  }
}

// ------------------- HALL CENTERING -------------------
void setDomeCenter() {
  if (digitalRead(hallEffectSensor_Pin) == LOW) {
    if (millis() - lastHallLowMillis >= HALL_DEBOUNCE) {
      domeCenterSet = true;
      encPos = CENTER_TICK;
      stopDome();
      Serial.println(F("Centered at 878"));
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

// ------------------- DOME SPIN CONTROL -------------------
void spinStuff() {
  static bool lastMoveR3 = false;
  bool curR3 = receiveFromESP32Data.moveR3;
  if (curR3 != lastMoveR3) {
    domeServoMode = curR3;
    lastMoveR3 = curR3;
    if (domeServoMode) domeCenterSet = false;
  }

  if (domeServoMode) domeServoMovement();
  else               spinDome();
}

void spinDome() {
  int speed = abs(domeServoPWM) > 3 ? constrain(map(abs(domeServoPWM), 3, 127, 50, 255), 0, 255) : 0;
  if (domeServoPWM > 3 && enableDrive) {
    digitalWrite(domeMotor_pin_A, LOW);  digitalWrite(domeMotor_pin_B, HIGH);
    analogWrite(domeMotor_pwm, speed);
  } else if (domeServoPWM < -3 && enableDrive) {
    digitalWrite(domeMotor_pin_A, HIGH); digitalWrite(domeMotor_pin_B, LOW);
    analogWrite(domeMotor_pwm, speed);
  } else {
    stopDome();
  }
}

void domeServoMovement() {
  if (!domeCenterSet) { setDomeCenter(); return; }

  Setpoint_domeSpinServoPid = enableDrive
      ? constrain(map(domeServoPWM, -180, 180, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS),
                  -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS)
      : 0;

  Input_domeSpinServoPid = ((encPos - CENTER_TICK) % TICKS_PER_REV);
  if (Input_domeSpinServoPid > TICKS_PER_REV/2) Input_domeSpinServoPid -= TICKS_PER_REV;
  else if (Input_domeSpinServoPid < -TICKS_PER_REV/2) Input_domeSpinServoPid += TICKS_PER_REV;

  myPID_domeSpinServoPid.Compute();

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

// ------------------- TIME / COMMS -------------------
void Timechecks() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    #ifdef DEBUG_WATCHDOG
    Serial.println(F("[DEBUG] Loop alive")); // Heartbeat
    #endif
  }
  currentMillis = millis();
  receiveMillis = currentMillis;
  if (currentMillis - lastPrintMillis >= printMillis) {
    lastPrintMillis = currentMillis;
    debugRoutines();
  }
  // if (currentMillis - receiveMillis >= 1000 && enableDrive) enableDrive = false;
  
// Comm watchdog: disable drive if no data for >2s
  if (millis() - receiveMillis > 2000) {
    enableDrive = false;
    domeServoPWM = 0;
    soundcmd = 0;
  }

}

void SendRecieveData() {
  if (recESP32.receiveData()) {
    receiveMillis = currentMillis;
    enableDrive   = receiveFromESP32Data.driveEnabled;
    reverseDrive  = receiveFromESP32Data.moveL3;
    domeServoPWM  = map(receiveFromESP32Data.domeSpin, -127, 127, -180, 180);
    soundcmd      = receiveFromESP32Data.soundcmd;


    // Check for reset trigger
    if (soundcmd == -99) {
      Serial.println(F("[RESET] Command received from ESP32. Rebooting..."));
      #if defined(USE_AVR_WATCHDOG)
        wdt_enable(WDTO_15MS); // Trigger watchdog reset in 15ms
        while (1) {}
      #elif defined(USE_SAMD_WATCHDOG)
        NVIC_SystemReset(); // Native SAMD reset
      #endif
    }

    if (!enableDrive || !domeServoMode) {
      Setpoint_domeSpinServoPid = 0;
    }

    sendToESP32Data.sndplaying = sndplaying;
    sendESP32.sendData();

  }
}

// ------------------- AUDIO (unified) -------------------
void mp3play() {
  sndplaying = audioIsPlaying();

  // Only play if DFPlayer is idle
  if (soundcmd && !sndplaying) {
    if (soundcmd == 9) soundcmd = randomsound; // Random track
    audioPlay(soundcmd);
    soundcmd = 0;
  }
}
// void mp3play() {
//   sndplaying = audioIsPlaying();

//   if (soundcmd) {
//     if (soundcmd == 9) soundcmd = randomsound;  // keep your random behavior
//     if (sndplaying) audioStop();
//     audioPlay(soundcmd);
//     soundcmd = 0;
//   }
// }

// ------------------- NEOPIXEL (simple effect) -------------------
#if defined(USE_NEOPIXEL)
void neoUpdate() {
  // Basic activity: blink PSI based on enableDrive and domeServoPWM magnitude
  static uint32_t lastNeo = 0;
  if (millis() - lastNeo < 33) return;  // ~30 FPS max
  lastNeo = millis();

  uint8_t level = constrain(abs(domeServoPWM), 0, 127);
  uint32_t color = strip.Color(level, 0, 127 - level); // simple red/blue mix
  bool flash = (receiveFromESP32Data.psiFlash != 0);

  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (flash && (i % 2 == ((millis()/200) % 2))) {
      strip.setPixelColor(i, strip.Color(255, 255, 255)); // flash white
    } else {
      strip.setPixelColor(i, color);
    }
  }
  strip.show();
}
#endif

// ------------------- SERVO TILT (SAFE SCALING) -------------------

void Servos() {
  int y_Axis = receiveFromESP32Data.leftStickY;
  int x_Axis = receiveFromESP32Data.leftStickX;

  if (domeServoMode && domeCenterSet) {
    int domeTurnPercent = map(Setpoint_domeSpinServoPid, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS, 100, -100);
    int scale = 100 - abs(domeTurnPercent);
    y_Axis = map(y_Axis, -127, 127, -scale, scale);
    x_Axis = map(x_Axis, -127, 127, -scale, scale);
  }

  if (reverseDrive) { y_Axis *= -1; x_Axis *= -1; }

  leftStickY = map(y_Axis, -127, 127, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
  leftStickX = map(x_Axis, -127, 127, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);

  if (currentMillis - lastServoUpdateMillis >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdateMillis = currentMillis;
    domeTiltAngle_Y_Axis += (leftStickY > domeTiltAngle_Y_Axis)
        ? min(servoEase, leftStickY - domeTiltAngle_Y_Axis)
        : max(-servoEase, leftStickY - domeTiltAngle_Y_Axis);
    domeTiltAngle_X_Axis += (leftStickX > domeTiltAngle_X_Axis)
        ? min(servoEase, leftStickX - domeTiltAngle_X_Axis)
        : max(-servoEase, leftStickX - domeTiltAngle_X_Axis);

    domeTiltAngle_Y_Axis = constrain(domeTiltAngle_Y_Axis, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
    domeTiltAngle_X_Axis = constrain(domeTiltAngle_X_Axis, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);
  }

  float pitch = receiveFromESP32Data.pitch;
  float roll  = receiveFromESP32Data.roll;

  double leftTarget  = leftServo_0_Position  + map(domeTiltAngle_Y_Axis - pitch, -24, 24, 36, -30)
                                        + map(domeTiltAngle_X_Axis + roll, -24, 24, 30, -30);
  double rightTarget = rightServo_0_Position + map(domeTiltAngle_Y_Axis - pitch, -24, 24, -36, 30)
                                        + map(domeTiltAngle_X_Axis + roll, -24, 24, 36, -36);

  double lDiff = abs(leftOldPosition  - leftTarget);
  double rDiff = abs(rightOldPosition - rightTarget);

  if (lDiff > rDiff && lDiff > 0.1) {
    leftOldPosition  += (leftOldPosition < leftTarget) ? min(1.0, lDiff) : max(-1.0, -lDiff);
    rightOldPosition += (rightOldPosition < rightTarget) ? rDiff/lDiff : -rDiff/lDiff;
  } else if (rDiff >= lDiff && rDiff > 0.1) {
    rightOldPosition += (rightOldPosition < rightTarget) ? min(1.0, rDiff) : max(-1.0, -rDiff);
    leftOldPosition  += (leftOldPosition < leftTarget) ? lDiff/rDiff : -lDiff/rDiff;
  }

#if defined(__AVR__)
  // VarSpeedServo supports speed parameter
  myservo2.write(constrain(leftOldPosition,  leftServo_0_Position-36,  leftServo_0_Position+36), servoSpeed);
  myservo1.write(constrain(rightOldPosition, rightServo_0_Position-36, rightServo_0_Position+36), servoSpeed);
#elif defined(ARDUINO_ARCH_SAMD)
  // Servo.h does not support speed, so easing is handled above
  myservo2.write(constrain(leftOldPosition,  leftServo_0_Position-36,  leftServo_0_Position+36));
  myservo1.write(constrain(rightOldPosition, rightServo_0_Position-36, rightServo_0_Position+36));
#endif
}

// void Servos() {
//   int y_Axis = receiveFromESP32Data.leftStickY;
//   int x_Axis = receiveFromESP32Data.leftStickX;

//   if (domeServoMode && domeCenterSet) {
//     int domeTurnPercent = map(Setpoint_domeSpinServoPid, -MAX_ANGLE_TICKS, MAX_ANGLE_TICKS, 100, -100);
//     int scale = 100 - abs(domeTurnPercent);
//     y_Axis = map(y_Axis, -127, 127, -scale, scale);
//     x_Axis = map(x_Axis, -127, 127, -scale, scale);
//   }

//   if (reverseDrive) { y_Axis *= -1; x_Axis *= -1; }

//   leftStickY = map(y_Axis, -127, 127, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
//   leftStickX = map(x_Axis, -127, 127, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);

//   if (currentMillis - lastServoUpdateMillis >= SERVO_UPDATE_INTERVAL) {
//     lastServoUpdateMillis = currentMillis;
//     domeTiltAngle_Y_Axis += (leftStickY > domeTiltAngle_Y_Axis)
//         ? min(servoEase, leftStickY - domeTiltAngle_Y_Axis)
//         : max(-servoEase, leftStickY - domeTiltAngle_Y_Axis);
//     domeTiltAngle_X_Axis += (leftStickX > domeTiltAngle_X_Axis)
//         ? min(servoEase, leftStickX - domeTiltAngle_X_Axis)
//         : max(-servoEase, leftStickX - domeTiltAngle_X_Axis);

//     domeTiltAngle_Y_Axis = constrain(domeTiltAngle_Y_Axis, -domeTiltYAxis_MaxAngle, domeTiltYAxis_MaxAngle);
//     domeTiltAngle_X_Axis = constrain(domeTiltAngle_X_Axis, -domeTiltXAxis_MaxAngle, domeTiltXAxis_MaxAngle);
//   }

//   float pitch = receiveFromESP32Data.pitch;
//   float roll  = receiveFromESP32Data.roll;

//   double leftTarget  = leftServo_0_Position  + map(domeTiltAngle_Y_Axis - pitch, -24, 24, 36, -30)
//                                         + map(domeTiltAngle_X_Axis + roll, -24, 24, 30, -30);
//   double rightTarget = rightServo_0_Position + map(domeTiltAngle_Y_Axis - pitch, -24, 24, -36, 30)
//                                         + map(domeTiltAngle_X_Axis + roll, -24, 24, 36, -36);

//   double lDiff = abs(leftOldPosition  - leftTarget);
//   double rDiff = abs(rightOldPosition - rightTarget);

//   if (lDiff > rDiff && lDiff > 0.1) {
//     leftOldPosition  += (leftOldPosition < leftTarget) ? min(1.0, lDiff) : max(-1.0, -lDiff);
//     rightOldPosition += (rightOldPosition < rightTarget) ? rDiff/lDiff : -rDiff/lDiff;
//   } else if (rDiff >= lDiff && rDiff > 0.1) {
//     rightOldPosition += (rightOldPosition < rightTarget) ? min(1.0, rDiff) : max(-1.0, -rDiff);
//     leftOldPosition  += (leftOldPosition < leftTarget) ? lDiff/rDiff : -lDiff/rDiff;
//   }

//   myservo2.write(constrain(leftOldPosition,  leftServo_0_Position-36,  leftServo_0_Position+36),  servoSpeed);
//   myservo1.write(constrain(rightOldPosition, rightServo_0_Position-36, rightServo_0_Position+36), servoSpeed);
// }

// ================= AUDIO IMPLEMENTATION =================
#if defined(AUDIO_SPARKFUN)
// ---- SparkFun Qwiic MP3 Trigger (I2C) ----
bool audioInit(unsigned long timeoutMs) {
  randomsound = random(1, 55);
  Wire.begin();
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    wdt_reset(); // Prevent watchdog reset during DFPlayer init
    if (mp3.begin(SPARKFUN_MP3_I2C_ADDR)) {
      mp3.setVolume(AUDIO_DEFAULT_VOLUME);     // 0..31
      Serial.print(F("SparkFun MP3 ready. Songs: "));
      Serial.println(mp3.getSongCount());
      return true;
    }
    delay(100);
  }
  Serial.println(F("SparkFun MP3 init failed"));
  return false;
}
void audioStop() { if (mp3.isConnected() && mp3.isPlaying()) mp3.stop(); }
void audioPlay(uint16_t track) { if (mp3.isConnected()) mp3.playFile(track); }
bool audioIsPlaying() { return mp3.isConnected() ? mp3.isPlaying() : false; }

#elif defined(AUDIO_DFPLAYER)
// ---- DFPlayer Mini (SoftwareSerial) ----

bool audioInit(unsigned long timeoutMs) {
  randomsound = random(1, 55);
  #if defined(__AVR__)
  pinMode(DFPLAYER_BUSY_PIN, INPUT_PULLUP);
  #elif defined(ARDUINO_ARCH_SAMD)
  pinPeripheral(DFPLAYER_RX_PIN, PIO_SERCOM_ALT);
  pinPeripheral(DFPLAYER_TX_PIN, PIO_SERCOM_ALT);
  #endif

  mp3Serial.begin(DFPLAYER_BAUD); // mp3Serial.begin(9600);

  unsigned long startTime = millis();
  while (millis() - startTime < timeoutMs) {
    // Prevent watchdog reset    
    #if defined(USE_AVR_WATCHDOG)
      wdt_reset();
    #elif defined(USE_SAMD_WATCHDOG)
      Watchdog.reset();
    #endif

    if (mp3.begin(mp3Serial)) {
      mp3.volume(AUDIO_DEFAULT_VOLUME);
      Serial.println(F("DFPlayer ready"));
      return true;
    }
    delay(10); // Short delay instead of 100ms
  }
  Serial.println(F("DFPlayer init failed"));
  return false;
}

// bool audioInit(unsigned long timeoutMs) {
//   randomsound = random(1, 55);
//   pinMode(DFPLAYER_BUSY_PIN, INPUT_PULLUP); // BUSY active LOW
//   mp3Serial.begin(DFPLAYER_BAUD);
//   unsigned long t0 = millis();
//   while (millis() - t0 < timeoutMs) {
//     if (mp3.begin(mp3Serial)) {
//       mp3.volume(AUDIO_DEFAULT_VOLUME);        // 0..30
//       Serial.println(F("DFPlayer ready"));
//       return true;
//     }
//     delay(100);
//   }
//   Serial.println(F("DFPlayer init failed"));
//   return false;
// }

void audioStop() { mp3.stop(); }
void audioPlay(uint16_t track) { mp3.play(track); }
bool audioIsPlaying() { return (digitalRead(DFPLAYER_BUSY_PIN) == LOW); }

#elif defined(AUDIO_NONE)
// ---- No audio device: provide empty stubs ----
bool audioInit(unsigned long timeoutMs) { return true; } // Pretend success
void audioStop() {}
void audioPlay(uint16_t track) {}
bool audioIsPlaying() { return false; }

#endif // audio implementation

// ------------------- DEBUG -------------------
void debugRoutines() {
#ifdef debugDOME
  Serial.print(F("enc:")); Serial.print(encPos);
  Serial.print(F(" c:")); Serial.print(domeCenterSet);
  Serial.print(F(" m:")); Serial.print(domeServoMode);
  Serial.print(F(" pwm:")); Serial.println(domeServoPWM);
#endif

#ifdef debugHALL
  Serial.print(F("Hall:")); Serial.print(digitalRead(hallEffectSensor_Pin));
  Serial.print(F(" enc:")); Serial.print(encPos);
  Serial.print(F(" diff:")); Serial.println(((encPos - CENTER_TICK) % TICKS_PER_REV) * 360.0 / TICKS_PER_REV);
#endif

#ifdef debugENC
  Serial.print(F("encPos:")); Serial.println(encPos);
#endif
}
