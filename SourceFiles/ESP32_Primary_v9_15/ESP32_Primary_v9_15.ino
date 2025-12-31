
/*
 * JOE'S DRIVE - FINAL: S2S + PID BALANCE + 3-SEC CALIBRATION
 * ESP32 Primary v9.15 — enhanced ESPNOW mirroring with reliable send-queue
 * (Primary device)
 */

#include <Arduino.h>
#include <EasyTransfer.h>
#include <analogWrite.h>
#include <Preferences.h>
#include <PSController.h> // uses legacy 1.67 library of https://github.com/reeltwo/PSController
#include <Ticker.h>
#include <PID_v1.h>
#include <Wire.h>
#include <stdarg.h>   // va_list, va_start, va_end
#include <deque>      // ESPNOW send queue
#include "esp_task_wdt.h"  // enable watchdog service
#include "SoundMappings.h" // support sound mappings based on button presses etc
#include "ControllerDefs.h" // support for controllerdefs

/* --------------------ESPNOW Needed Libraries-----------*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h> // Add OTA updates for firmware

/* ------------------- CONFIG ------------------- */
const int  MAX_S2S_TILT       = 300;
const int  JOYSTICK_DEADZONE  = 25;
const int  S2S_POT_MIN        = 1300;
const int  S2S_POT_MAX        = 2150;
const int  POT_FILTER_SIZE    = 10;

bool REVERSE_S2S = false;    // Default
bool REVERSE_DRIVE = false;    // Default
bool USE_ESPNOW   = false;  // Default

bool DEBUG_CALIBRATION  = false;
bool DEBUG_JOYSTICK     = false;
bool DEBUG_S2S_MODE     = false;
bool DEBUG_MOTOR_OUTPUT = false;
bool DEBUG_IMU_RAW      = false;
bool DEBUG_ALL          = false;
bool DEBUG_COMPACT      = false;

// ----- Feature toggles -----
bool ENABLE_DIAGNOSTIC_TELEMETRY = false;  // default OFF; toggle via "SET ENABLE_DIAGNOSTIC_TELEMETRY ON|OFF"
bool ENABLE_SMOOTH_BALANCE = false; // Default OFF

// ESPNOW remote MAC address
// Remote ESP32 MAC for ESPNOW (receiver): C4:5B:BE:90:6A:68
uint8_t remoteMAC[] = {0xC4, 0x5B, 0xBE, 0x90, 0x6A, 0x68};
const char* MASTER_NAV   = "7c:9e:bd:d7:63:c6";

/*------------------- OTA ESP32 Updates for firmware over Wifi--------------- */
String wifiSSID;
String wifiPassword;
bool OTA_ENABLED = false; // Dynamic OTA toggle

/* ------------------ RETURN-TO-CENTER TUNING --------------------------- */
const int  RETURN_DEADBAND   = 25;
const int  MIN_RETURN_PWM    = 30;
const int  MAX_RETURN_PWM    = 180;
bool ENABLE_ROLL_BIAS  = false;
const float ROLL_BIAS_GAIN   = 10.0;
const int defaultPOT         = 1708;


/* ----- PID GAINS (MUTABLE for tuning; persisted in Preferences) ----- */
float KP_PITCH = 45.0f;
float KI_PITCH = 0.8f;
float KD_PITCH = 12.0f;

/*
Pk2 = 45.0f;   // Lower P gain for gentle corrections
Dk2 = 1.0f;   // Moderate D gain for damping oscillations
Ik2 = 15.0f;    // Small I gain to avoid overshoot
PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
*/

float Pk2 = 25.0f;
float Ik2 = 12.0f;
float Dk2 = 0.5f;

/* ----- IMU / Timing ----- */
const float IMU_DEADZONE   = 1.45;
const unsigned long IMU_TIMEOUT = 500;

/* ----- CALIBRATION HOLD ----- */
const unsigned long CALIB_HOLD_TIME = 3000;  // 3s
unsigned long calibHoldStart = 0;
bool calibSaveActive = false;
bool calibResetActive = false;

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
struct IMUData { float pitch, roll; };

struct Send32u4Data {
  bool   driveEnabled;
  int8_t domeSpin;
  bool   moveL3, moveR3;
  int8_t leftStickX, leftStickY;
  int8_t soundcmd, psiFlash;
  float  pitch, roll;
};

// struct ControllerButtons {
//   bool cross, circle, up, down, left, right, ps, l1, l3, r1;
//   int8_t leftStickX, leftStickY, rightStickX, rightStickY;
//   int8_t l2;
// };

/* ------------------- SOUND SYSTEM ------------------- */
// Moved to seperate support file SoundMappings.h

uint16_t pendingSound = SOUND_NONE;
static inline void emitSound(uint16_t track) { if (track != SOUND_NONE) pendingSound = track; }

#define EDGE_PRESSED(cur, prev) ((prev) == false && (cur) == true)

/* ------------------- LIVE TUNING MODE ------------------- */
enum class TuningSession { NONE, PITCH_KP, PITCH_KD, PITCH_KI, ROLL_PK, ROLL_DK, ROLL_IK };
TuningSession tuning = TuningSession::NONE;

unsigned long tuningHoldStart = 0;
bool tuningHoldActive = false;

float  kpPitchWork = 45.0f, kdPitchWork = 0.0f, kiPitchWork = 0.0f;
float  pk2Work     = 45.0f, dk2Work     = 0.0f, ik2Work     = 0.0f;

const float KP_STEP = 1.0f, KD_STEP = 1.0f, KI_STEP = 0.1f;
const float PK2_STEP = 1.0f, DK2_STEP = 1.0f, IK2_STEP = 0.1f;
const float REDUCE_FACTOR = 0.25f;

/* ------------------- GLOBALS ------------------- */
EasyTransfer recIMU, send32u4;
Preferences preferences;
PSController driveController(nullptr), domeController(nullptr);
Ticker mainLoopTicker;
Ticker telemetryTicker;


IMUData receiveIMUData;
Send32u4Data sendTo32u4Data;
ControllerButtons buttonsL, buttonsR;
bool IMUconnected = false, controllerConnected = false;
bool drivecontrollerConnected = false, domecontrollerConnected = false;
bool DomeServoMode = false, enableDrive = false, reverseDrive = false, EnableFlywheel = false;
bool autoBalance = false;
float flywheel = 0;
unsigned long lastIMUMillis = 0;
int potOffset = 1708;
float pitchOffset = 0.0, rollOffset = 0.0;
int potValues[POT_FILTER_SIZE];
int potValueIndex = 0;

unsigned long lastS2STime = 0;
unsigned long lastCompactDebug = 0;
const unsigned long COMPACT_DEBUG_INTERVAL = 100;

/* ------------------- AUTO-CENTER (BOOT) ------------------- */
bool autoCenterPending = true;
bool autoCenterDone     = false;
const float AUTO_CENTER_TOL_DEG      = 0.50f;
const unsigned long AUTO_CENTER_STABLE_MS = 1500;
unsigned long autoCenterStart = 0;

const int AUTO_POT_SAMPLES = 20;
int autoPotBuf[AUTO_POT_SAMPLES];
int autoPotIdx = 0;
bool autoPotFilled = false;
const float AUTO_POT_STD_MAX = 5.0f;

/* S2S Debug */
int lastPwmS2S = 0;
enum class S2SMode { JOY, RTN, STOP };
S2SMode s2sMode = S2SMode::STOP;

/* PID internals */
float pitchIntegral = 0.0;
float prevPitchError = 0.0;
unsigned long lastPIDTime = 0;
const float INTEGRAL_LIMIT = 100.0;

double Setpoint2 = 0.0;
double Input2 = 0.0;
double Output2 = 0.0;
double Output2_S2S_pwm = 0;

PID PID2_S2S(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);

/* =========================================================
   ============== ESPNOW + Serial MIRROR HELPERS ===========
   ========================================================= */

// ESPNOW payloads should be < ~250 bytes. Keep lines compact.
#ifndef STATUS_MAX_LEN
#define STATUS_MAX_LEN 220
#endif

// ---------- ESPNOW Send Queue with in-flight tracking ----------
struct Msg {
  size_t len;
  char   buf[STATUS_MAX_LEN];
  int    retries;     // retry counter for this message
};

std::deque<Msg> txQ;

// Only one ESP-NOW send at a time
static bool inFlight = false;
static Msg  currentMsg{};
static unsigned long lastSendMs = 0;

const unsigned SEND_INTERVAL_MS = 5;  // pace: max ~200 msgs/sec
const int      MAX_RETRIES      = 3;  // retry failed sends a few times

// Enqueue a line for ESPNOW (mirroring uses this)
inline void enqueueLineForEspNow(const char* s, size_t len) {
  if (!s || len == 0) return;
  Msg m{};
  m.len = (len < STATUS_MAX_LEN ? len : STATUS_MAX_LEN - 1);
  memcpy(m.buf, s, m.len);
  m.buf[m.len] = '\0';
  m.retries = 0;
  if (txQ.size() < 256) { txQ.push_back(m); }
}

void sendStatusESPNOW(const char* message) {
    if (!USE_ESPNOW) return;
    if (!message || strlen(message) == 0) return;

    // Enqueue message for ESPNOW send queue
    enqueueLineForEspNow(message, strlen(message));
}

// Pump the queue from task context; do NOT pop until onDataSent() returns success
inline void pumpEspNowSends() {
  unsigned long now = millis();

  // If a send is in-flight, wait for the callback
  if (inFlight) return;

  // Ready to send next?
  if (!txQ.empty() && (now - lastSendMs) >= SEND_INTERVAL_MS) {
    currentMsg = txQ.front();    // copy but DO NOT pop yet
    inFlight = true;

    // Include trailing NUL so remote can treat as C-string
    esp_now_send(remoteMAC, (const uint8_t*)currentMsg.buf, currentMsg.len + 1);
    // We will pop (or retry) in onDataSent()
  }
}

// Printf-style: mirrors to Serial and ENQUEUE for ESPNOW
inline void dbgPrintf(const char* fmt, ...) {
  char line[STATUS_MAX_LEN];
  va_list args; va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  Serial.print(line);
  enqueueLineForEspNow(line, strlen(line));
}

// println-style: mirrors to Serial and ENQUEUE (adds newline)
inline void dbgPrintln(const char* s) {
  Serial.println(s);
  char line[STATUS_MAX_LEN];
  size_t n = strlcpy(line, s, sizeof(line));
  if (n + 1 < sizeof(line)) { line[n++] = '\n'; line[n] = '\0'; }
  enqueueLineForEspNow(line, n);
}

// print-style: mirrors to Serial and ENQUEUE (no newline)
inline void dbgPrint(const char* s) {
  Serial.print(s);
  enqueueLineForEspNow(s, strlen(s));
}

// Macros for easier replacement
#define LOGF(...)  dbgPrintf(__VA_ARGS__)
#define LOGLN(S)   dbgPrintln(S)
#define LOG(S)     dbgPrint(S)

/* ------------------- TELEMETRY (flag + builder; ALWAYS VISIBLE) ------------------- */
volatile bool telemetryDue = false;  // set by ticker, consumed in loop()

void sendTelemetryNow() {
  char line[STATUS_MAX_LEN];
  snprintf(line, sizeof(line),
    "T=%lu,ROLL=%.2f,PITCH=%.2f,PWM_S2S=%d,DriveEn=%d,AutoBal=%d,KP=%.1f,KD=%.1f,KI=%.1f,Pk2=%.1f,Dk2=%.1f,Ik2=%.1f",
    millis(),
    receiveIMUData.roll, receiveIMUData.pitch,
    lastPwmS2S,
    enableDrive ? 1 : 0,
    autoBalance ? 1 : 0,
    KP_PITCH, KD_PITCH, KI_PITCH, Pk2, Dk2, Ik2
  );
  enqueueLineForEspNow(line, strlen(line));   // queued & paced in loop
}

// ---- Telemetry flag setter (called by Ticker) ----
void telemetryTick() {
  telemetryDue = true;
}

void applyTelemetrySetting() {
  telemetryTicker.detach();
  telemetryDue = false;
  if (ENABLE_DIAGNOSTIC_TELEMETRY) telemetryTicker.attach_ms(500, telemetryTick);
}

/* ------------------- FORWARD DECLS ------------------- */
void mainLoop();
void handleButtonActions();
void receiveIMU();
void sendDataTo32u4();
void S2S_Movement();
void drive_Movement();
void spinFlywheel();
void printDebugInfo();
void handleCalibrationHold();
void saveCalibration();
void resetCalibration();
void updateControllerStates();
void applyDeadzone(int8_t& x, int8_t& y);
void handleSerialCommands(String cmd = "");
void printTuningStatus();
int  filterPotValue(int raw);
void handleTuningHold();
void handleLiveTuning();
void printTuningBanner();
void cancelLiveTuning(bool announce = true);
void printActiveDebugFlagsOnce();

// Forward declarations for ESPNOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

void initEspNow();

/* ------------------- SETUP ------------------- */

void setup() {
    // -------------------- 1. Boot Diagnostics --------------------
    Serial.begin(115200);
    Serial.printf("\n[BOOT] Version: JOE'S DRIVE v9.1 | Build: %s %s\n", __DATE__, __TIME__);
    Serial.printf("[BOOT] Reset Reason: %d\n", esp_reset_reason());
  
    preferences.begin("JoeDriveV2", false);
    OTA_ENABLED = preferences.getBool("otaEnabled", OTA_ENABLED);

    // OTA Configuration pull from preferences if set for Wifi
    wifiSSID = preferences.getString("wifiSSID", "");
    wifiPassword = preferences.getString("wifiPassword", "");

    if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        Serial.printf("[WiFi] Connecting to %s...\n", wifiSSID.c_str());

        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
            ArduinoOTA.begin();
            Serial.printf("[OTA] Ready for updates");
        } else {
            Serial.printf("\n[WiFi] Failed to connect. OTA disabled.");
        }
    } else {
        Serial.printf("[WiFi] No credentials set. OTA disabled.");
    }

    // Store version and build info in preferences
    preferences.putString("version", "JOE'S DRIVE v9.1");
    preferences.putString("build", String(__DATE__) + " " + String(__TIME__));

    // Watchdog reset tracking
    int wdResetCount = preferences.getInt("wdResetCount", 0);
    esp_reset_reason_t reason = esp_reset_reason();

    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
        wdResetCount++;
        preferences.putInt("wdResetCount", wdResetCount);
        Serial.printf("[BOOT] Watchdog reset detected! Count=%d\n", wdResetCount);
    } else {
        wdResetCount = 0; // Normal boot clears counter
        preferences.putInt("wdResetCount", wdResetCount);
    }

    // -------------------- 2. Safe Recovery --------------------
    if (wdResetCount >= 3) {
        Serial.println("[SAFE RECOVERY] Applying failsafe settings...");
        USE_ESPNOW = false;
        ENABLE_DIAGNOSTIC_TELEMETRY = false;
        DEBUG_ALL = DEBUG_COMPACT = DEBUG_CALIBRATION = DEBUG_JOYSTICK =
        DEBUG_S2S_MODE = DEBUG_MOTOR_OUTPUT = DEBUG_IMU_RAW = false;

        preferences.putBool("useEspNow", false);
        preferences.putBool("diagTelemetry", false);
        preferences.putBool("debugAll", false);
        preferences.putBool("debugCompact", false);
        // ... repeat for other debug flags
    }

    // -------------------- 3. Enable Watchdog --------------------
    esp_task_wdt_init(5, true); // 5-second timeout
    esp_task_wdt_add(NULL);     // Add main loop task

    // -------------------- 4. Load Preferences --------------------
    REVERSE_S2S = preferences.getBool("reverseS2S", REVERSE_S2S);
    reverseDrive = preferences.getBool("reverseDrive", reverseDrive);
    USE_ESPNOW = preferences.getBool("useEspNow", USE_ESPNOW);
    ENABLE_DIAGNOSTIC_TELEMETRY = preferences.getBool("diagTelemetry", ENABLE_DIAGNOSTIC_TELEMETRY);
    ENABLE_SMOOTH_BALANCE = preferences.getBool("smoothBalance", ENABLE_SMOOTH_BALANCE);

    potOffset = preferences.getInt("potOffset", defaultPOT);
    pitchOffset = preferences.getFloat("pitchOffset", 0.0f);
    rollOffset = preferences.getFloat("rollOffset", 0.0f);

    KP_PITCH = preferences.getFloat("kp_pitch", KP_PITCH);
    KI_PITCH = preferences.getFloat("ki_pitch", KI_PITCH);
    KD_PITCH = preferences.getFloat("kd_pitch", KD_PITCH);
    Pk2 = preferences.getFloat("pk2", Pk2);
    Ik2 = preferences.getFloat("ik2", Ik2);
    Dk2 = preferences.getFloat("dk2", Dk2);
    PID2_S2S.SetTunings(Pk2, Ik2, Dk2);

    // -------------------- 5. Initialize ESPNOW if enabled --------------------
    if (USE_ESPNOW) {
        initEspNow();
    }

    // -------------------- 6. Hardware Setup --------------------
    Serial1.begin(115200);                    // IMU serial
    Serial2.begin(74880, SERIAL_8N1, 13, 12); // 32u4 serial
    randomSeed((uint32_t)micros());

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

    // -------------------- 7. Controller & Communication --------------------
    if (!PSController::startListening(MASTER_NAV))
        LOGF("T: %lu BT failed\n", millis());
    else
        LOGF("T: %lu BT MAC: %s\n", millis(), PSController::getDeviceAddress().c_str());

    recIMU.begin(details(receiveIMUData), &Serial1);
    send32u4.begin(details(sendTo32u4Data), &Serial2);

    // -------------------- 8. IMU Quick Probe --------------------
    for (int i = 0; i < 10000; i++) {
        if (recIMU.receiveData()) { IMUconnected = true; lastIMUMillis = millis(); break; }
        delay(1);
    }

    // -------------------- 9. Auto-Center on First Boot --------------------
    if (!preferences.isKey("potOffset")) {
        performAutoCenter(); // Helper function for clarity
    }

    // -------------------- 10. PID Setup --------------------
    PID2_S2S.SetMode(AUTOMATIC);
    PID2_S2S.SetOutputLimits(-255, 255);
    PID2_S2S.SetSampleTime(10);
    lastPIDTime = millis();

    // -------------------- 11. Start Main Loop --------------------
    mainLoopTicker.attach_ms(10, mainLoop);


    emitSound(SOUND_STARTUP);
    sendDataTo32u4();

    LOGLN("JOE'S DRIVE v9.1 — ESP32 MASTER — READY");
    printActiveDebugFlagsOnce();
}

void initOTA() {
    ArduinoOTA.setHostname("JoeDriveESP32");
    ArduinoOTA.setPassword("securepassword"); // Optional

    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] Update starting...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("[OTA] Update finished!");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] Error[%u]\n", error);
    });

    ArduinoOTA.begin();
    Serial.println("[OTA] Ready for updates");
}


/*--------------- Helpers-----------------------------------------------*/

void performAutoCenter() {
    Serial.println("[AUTO-CENTER] First boot detected. Waiting for stability...");

    const unsigned long STABILITY_TIME = 5000;      // 5 seconds
    const float STABILITY_THRESHOLD = 1.0f;         // Max allowed roll flex
    const float LEVEL_THRESHOLD = 0.50f;            // Must be near level
    unsigned long start = millis();

    float pitchSum = 0, rollSum = 0;
    int potSum = 0, samples = 0;
    float rollMin = 9999, rollMax = -9999;

    while (millis() - start < STABILITY_TIME) {
        if (recIMU.receiveData()) {
            float pitch = receiveIMUData.pitch;
            float roll = receiveIMUData.roll;

            pitchSum += pitch;
            rollSum += roll;
            potSum += analogRead(S2S_POT_PIN);
            samples++;

            // Track roll range for stability
            rollMin = min(rollMin, roll);
            rollMax = max(rollMax, roll);
        }
        delay(10);
    }

    if (samples > 0) {
        float avgPitch = pitchSum / samples;
        float avgRoll = rollSum / samples;
        int avgPot = potSum / samples;
        float rollRange = rollMax - rollMin;

        Serial.printf("[AUTO-CENTER] Samples=%d AvgRoll=%.2f RollRange=%.2f\n", samples, avgRoll, rollRange);

        // Check stability AND level
        if (rollRange <= STABILITY_THRESHOLD && fabs(avgRoll) <= LEVEL_THRESHOLD) {
            potOffset = avgPot;
            pitchOffset = avgPitch;
            rollOffset = avgRoll;

            preferences.putInt("potOffset", potOffset);
            preferences.putFloat("pitchOffset", pitchOffset);
            preferences.putFloat("rollOffset", rollOffset);

            Serial.printf("[AUTO-CENTER] Saved: Pot=%d Pitch=%.2f Roll=%.2f\n", potOffset, pitchOffset, rollOffset);
            if (USE_ESPNOW) sendStatusESPNOW("[AUTO-CENTER COMPLETE]");
        } else {
            Serial.printf("[AUTO-CENTER] Skipped: Roll unstable or not level (Range=%.2f°, AvgRoll=%.2f°)\n", rollRange, avgRoll);
        }
    } else {
        Serial.println("[AUTO-CENTER] Failed: No IMU data. Using defaults.");
    }
}

/* ------------------- ESPNOW CALLBACKS ------------------- */
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (USE_ESPNOW) {
    unsigned long now = millis();

    if (status == ESP_NOW_SEND_SUCCESS) {
      // Success: pop the message and clear in-flight
      if (!txQ.empty()) txQ.pop_front();
      inFlight = false;
      lastSendMs = now;
      // Optional: Serial.printf("[ESPNOW] Sent OK (%u bytes)\n", (unsigned)currentMsg.len);
    } else {
      // Failure: retry a limited number of times
      currentMsg.retries++;
      // IMPORTANT: log to Serial ONLY; do not mirror via ESPNOW to avoid feedback loop
      Serial.printf("[ESPNOW] Send FAIL (%d), retry %d/%d\n",
                    (int)status, currentMsg.retries, MAX_RETRIES);

      inFlight = false;     // allow pump to issue a retry
      lastSendMs = now;

      if (currentMsg.retries < MAX_RETRIES) {
        // Leave the message in the front of queue; pumpEspNowSends() will retry
        // after SEND_INTERVAL_MS
      } else {
        // Drop after too many retries to prevent blocking subsequent messages
        if (!txQ.empty()) txQ.pop_front();
        // Optional: Serial.println("[ESPNOW] Dropping message after max retries.");
      }
    }
  }
}
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Interpret incoming payload as command line (from remote)
  if (USE_ESPNOW) {
    String cmd = String((char*)incomingData);
    cmd.trim();
    if (cmd.length() > 0) {
      LOGF("[REMOTE CMD] %s\n", cmd.c_str());
      handleSerialCommands(cmd);
    }
  }
}


void initEspNow() {
    WiFi.mode(WIFI_STA);
    Serial.printf("[MAC] This ESP32 STA: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init Failed!");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, remoteMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESPNOW] Failed to add peer");
    }

    applyTelemetrySetting(); // respects ENABLE_DIAGNOSTIC_TELEMETRY
    Serial.println("[ESPNOW] Initialized successfully");
}


/* ------------------- MAIN LOOP (Ticker @10ms) ------------------- */
void mainLoop() {
  // NOTE: DO NOT read Serial in this timer context.
  receiveIMU();
  updateControllerStates();
  handleTuningHold();
  
// Always check for Serial commands
  handleSerialCommands();

  
  if (tuning != TuningSession::NONE) {
    handleLiveTuning();
    printTuningStatus();

    // Neutralize outputs while tuning
    enableDrive = false;
    sendTo32u4Data.driveEnabled = false;
    sendTo32u4Data.domeSpin = 0;
    sendTo32u4Data.moveL3 = false;
    sendTo32u4Data.moveR3 = false;
    pendingSound = SOUND_NONE;
    sendTo32u4Data.soundcmd = 0;

    bool allowS2SForRollTuning = (tuning == TuningSession::ROLL_DK || tuning == TuningSession::ROLL_IK);
    if (allowS2SForRollTuning) {
      autoBalance = true;
      S2S_Movement();
    } else {
      digitalWrite(S2S_PIN_1, LOW);
      digitalWrite(S2S_PIN_2, LOW);
      analogWrite(S2S_PWM, 0);
      lastPwmS2S = 0;
    }

    digitalWrite(DRIVE_PIN_1, LOW);
    digitalWrite(DRIVE_PIN_2, LOW);
    analogWrite(DRIVE_PWM, 0);

    sendDataTo32u4();

  } else {
    // Normal operation
    handleCalibrationHold();
    handleButtonActions();
    S2S_Movement();
    drive_Movement();
    spinFlywheel();
    sendDataTo32u4();
    printDebugInfo();

    if (DEBUG_COMPACT || DEBUG_ALL) {
      if (millis() - lastCompactDebug >= COMPACT_DEBUG_INTERVAL) {
        lastCompactDebug = millis();
        int rawPot      = analogRead(S2S_POT_PIN);
        int filteredPot = filterPotValue(rawPot);
        filteredPot     = constrain(filteredPot, S2S_POT_MIN, S2S_POT_MAX);
        const char* modeStr = (s2sMode == S2SMode::JOY) ? "JOY" :
                              (s2sMode == S2SMode::RTN) ? "RTN" : "STOP";
        LOGF(
          "T:%lu | JOY_X:%4d | POT_FILT:%4d | ROLL:%6.2f | PID_OUT:%6.1f | S2S:%3d | DIR:%d%d | MODE:%s%s%s%s\r\n",
          millis(),
          (int)buttonsR.rightStickX,
          filteredPot,
          receiveIMUData.roll,
          Output2,
          lastPwmS2S,
          digitalRead(S2S_PIN_1), digitalRead(S2S_PIN_2),
          modeStr,
          autoBalance ? " [BAL]" : "",
          calibSaveActive ? " [SAVE?]" : "",
          calibResetActive ? " [RESET?]" : ""
        );
      }
    }
  }
}


/* ------------------- IMU ------------------- */
void receiveIMU() {
  unsigned long now = millis();
  if (recIMU.receiveData()) {
    float rawPitch = receiveIMUData.pitch - pitchOffset;
    float rawRoll  = receiveIMUData.roll  - rollOffset;

    // Deadzone gate
    sendTo32u4Data.pitch = (abs(rawPitch) > IMU_DEADZONE) ? rawPitch : 0;
    sendTo32u4Data.roll  = (abs(rawRoll)  > IMU_DEADZONE) ? rawRoll  : 0;

    IMUconnected = true;
    lastIMUMillis = now;
  } else if (now - lastIMUMillis > IMU_TIMEOUT) {
    if (IMUconnected && autoBalance) {
      autoBalance = false;
      LOGF("T: %lu IMU Lost → Balance OFF\n", now);
    }
    IMUconnected = false;
  }
}

/* ------------------- SEND TO 32u4 (one-shot sound) ------------------- */
void sendDataTo32u4() {
  // Sound is one-shot per frame
  sendTo32u4Data.soundcmd = (int8_t)pendingSound;
  send32u4.sendData();
  pendingSound = SOUND_NONE;
}

/* ------------------- BUTTONS + SOUNDS (NORMAL MODE) ------------------- */
void handleButtonActions() {
    if (!controllerConnected) return;
    if (tuning != TuningSession::NONE) return;

    static ControllerButtons prevR{}, prevL{};

    // Dome spin vs. Flywheel L1 gating
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

    // Individual controller sounds
    uint16_t soundR = resolveDriveControllerSound(buttonsR, prevR);
    uint16_t soundL = resolveDomeControllerSound(buttonsL, prevL);

    // Combo sounds (both controllers)
    uint16_t comboSound = resolveComboSound(buttonsL, buttonsR, prevL, prevR);

    // Priority: combo > individual
    if (comboSound != SOUND_NONE) {
        emitSound(comboSound);
    } else {
        uint16_t chosen = (soundR != SOUND_NONE) ? soundR : soundL;
        if (chosen != SOUND_NONE) emitSound(chosen);
    }

    // Toggles with unique sounds
    if (EDGE_PRESSED(buttonsR.ps, prevR.ps)) {
        enableDrive = !enableDrive;
        sendTo32u4Data.driveEnabled = enableDrive;
        LOGF("T: %lu enableDrive %s\n", millis(), enableDrive ? "ON" : "OFF");
        emitSound(SOUND_TOGGLE_DRIVE);
    }

    if (EDGE_PRESSED(buttonsL.l3, prevL.l3)) {
        DomeServoMode = !DomeServoMode;
        sendTo32u4Data.moveR3 = DomeServoMode;
        LOGF("T: %lu Dome Servo Mode %s\n", millis(), DomeServoMode ? "ON" : "OFF");
        emitSound(SOUND_TOGGLE_DOME_SERVO);
    }

    if (EDGE_PRESSED(buttonsR.l3, prevR.l3)) {
        reverseDrive = !reverseDrive;
        sendTo32u4Data.moveL3 = reverseDrive;
        LOGF("T: %lu Reverse Drive %s\n", millis(), reverseDrive ? "ON" : "OFF");
        emitSound(SOUND_TOGGLE_REVERSE);
    }

    if (EDGE_PRESSED(buttonsR.cross, prevR.cross)) {
        autoBalance = !autoBalance;
        LOGF("T: %lu Auto Balance %s\n", millis(), autoBalance ? "ON" : "OFF");
        emitSound(SOUND_TOGGLE_BALANCE);
    }

    // Dome tilt stick to 32u4
    sendTo32u4Data.leftStickX = buttonsL.leftStickX;
    sendTo32u4Data.leftStickY = buttonsL.leftStickY;

    prevR = buttonsR;
    prevL = buttonsL;
}



/* ------------------- POT FILTER ------------------- */
int filterPotValue(int raw) {
  potValues[potValueIndex] = raw;
  potValueIndex = (potValueIndex + 1) % POT_FILTER_SIZE;
  long sum = 0;
  for (int i = 0; i < POT_FILTER_SIZE; i++) sum += potValues[i];
  return (int)(sum / POT_FILTER_SIZE);
}

// int filterPotValue(int raw) {
//   potValues[potValueIndex] = raw;
//   potValueIndex = (potValueIndex + 1) % POT_FILTER_SIZE;
//   long sum = 0;
//   for (int i = 0; i < POT_FILTER_SIZE; i++) sum += potValues[i];
//   return sum / POT_FILTER_SIZE;
// }


/* ------------------- S2S MOVEMENT (roll PID or return-to-center) ------------------- */


void S2S_Movement() {
  unsigned long now = millis();
  lastS2STime = now;

  int rawPot   = analogRead(S2S_POT_PIN);
  int potValue = filterPotValue(rawPot);
  potValue     = constrain(potValue, S2S_POT_MIN, S2S_POT_MAX);

  int joyX = buttonsR.rightStickX;

  int pwm = 0;
  bool dir1 = false, dir2 = false;
  s2sMode = S2SMode::STOP;

  // Manual joystick S2S
  if (abs(joyX) > JOYSTICK_DEADZONE && tuning == TuningSession::NONE) {
    pwm = map(abs(joyX), 0, 127, 0, 255);
    dir1 = (joyX < 0) ? REVERSE_S2S : !REVERSE_S2S;
    dir2 = (joyX < 0) ? !REVERSE_S2S : REVERSE_S2S;
    s2sMode = S2SMode::JOY;
    PID2_S2S.SetMode(MANUAL);
    Output2 = 0;
  } else {
    // Auto modes
    if (autoBalance && IMUconnected) {
      // Smooth IMU filtering
      static float filteredRoll = 0.0f;
      filteredRoll = 0.9f * filteredRoll + 0.1f * (receiveIMUData.roll - rollOffset);
      Input2 = filteredRoll;

      if (abs(Input2) > IMU_DEADZONE) {
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
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
      // Return to center by pot
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

  // Safety / gating
  bool allowNormal = controllerConnected && (tuning == TuningSession::NONE) && enableDrive && !buttonsL.l1 && !buttonsR.l1;
  bool allowDuringRollTuning = (tuning == TuningSession::ROLL_DK || tuning == TuningSession::ROLL_IK);

  if (allowNormal || allowDuringRollTuning) {
    digitalWrite(S2S_PIN_1, dir1 ? HIGH : LOW);
    digitalWrite(S2S_PIN_2, dir2 ? HIGH : LOW);

    // Smooth Balance Toggle
    if (ENABLE_SMOOTH_BALANCE && autoBalance) {
      const int RAMP_STEP = 5; // max PWM change per loop
      if (pwm > lastPwmS2S) {
        lastPwmS2S = min(lastPwmS2S + RAMP_STEP, pwm);
      } else {
        lastPwmS2S = max(lastPwmS2S - RAMP_STEP, pwm);
      }
    } else {
      lastPwmS2S = pwm;
    }

    analogWrite(S2S_PWM, lastPwmS2S);
  } else {
    digitalWrite(S2S_PIN_1, LOW);
    digitalWrite(S2S_PIN_2, LOW);
    analogWrite(S2S_PWM, 0);
    lastPwmS2S = 0;
  }
}

// void S2S_Movement() {
//   unsigned long now = millis();
//   lastS2STime = now;

//   int rawPot   = analogRead(S2S_POT_PIN);
//   int potValue = filterPotValue(rawPot);
//   potValue     = constrain(potValue, S2S_POT_MIN, S2S_POT_MAX);

//   int joyX = buttonsR.rightStickX;

//   int pwm = 0;
//   bool dir1 = false, dir2 = false;
//   s2sMode = S2SMode::STOP;
//   int targetPWM = pwm;

//   // Manual joystick S2S
//   if (abs(joyX) > JOYSTICK_DEADZONE && tuning == TuningSession::NONE) {
//     pwm = map(abs(joyX), 0, 127, 0, 255);
//     dir1 = (joyX < 0) ? REVERSE_S2S : !REVERSE_S2S;
//     dir2 = (joyX < 0) ? !REVERSE_S2S : REVERSE_S2S;
//     s2sMode = S2SMode::JOY;
//     PID2_S2S.SetMode(MANUAL);
//     Output2 = 0;
//   } else {
//     // Auto modes
//     if (autoBalance && IMUconnected) {
//       // Roll PID balance around 0°
//       Input2 = receiveIMUData.roll - rollOffset;
//       if (abs(Input2) > IMU_DEADZONE) {
//         PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
//         PID2_S2S.SetMode(AUTOMATIC);
//         PID2_S2S.Compute();

//         Output2_S2S_pwm = constrain(abs(Output2), MIN_RETURN_PWM, MAX_RETURN_PWM);
//         pwm = (int)Output2_S2S_pwm;

//         dir1 = (Output2 > 0) ? REVERSE_S2S : !REVERSE_S2S;
//         dir2 = (Output2 > 0) ? !REVERSE_S2S : REVERSE_S2S;
//         s2sMode = S2SMode::RTN;
//       } else {
//         PID2_S2S.SetMode(MANUAL);
//         Output2 = 0;
//       }
//     } else {
//       // Return to center by pot
//       int target = potOffset;
//       if (ENABLE_ROLL_BIAS) target += (int)(sendTo32u4Data.roll * ROLL_BIAS_GAIN);
//       int error = potValue - target;
//       int candidate = constrain(abs(error) * 2, 0, MAX_RETURN_PWM);
//       if (candidate >= MIN_RETURN_PWM && abs(error) > RETURN_DEADBAND) {
//         pwm = candidate;
//         dir1 = (error < 0) ? REVERSE_S2S : !REVERSE_S2S;
//         dir2 = (error < 0) ? !REVERSE_S2S : REVERSE_S2S;
//         s2sMode = S2SMode::RTN;
//       }
//     }
//   }

//   // Safety / gating
//   bool allowNormal = controllerConnected && (tuning == TuningSession::NONE) && enableDrive && !buttonsL.l1 && !buttonsR.l1;
//   bool allowDuringRollTuning = (tuning == TuningSession::ROLL_DK || tuning == TuningSession::ROLL_IK);

//   if (allowNormal || allowDuringRollTuning) {
//     digitalWrite(S2S_PIN_1, dir1 ? HIGH : LOW);
//     digitalWrite(S2S_PIN_2, dir2 ? HIGH : LOW);
//     analogWrite(S2S_PWM, pwm);
//     lastPwmS2S = pwm;
//   } else {
//     digitalWrite(S2S_PIN_1, LOW);
//     digitalWrite(S2S_PIN_2, LOW);
//     analogWrite(S2S_PWM, 0);
//     lastPwmS2S = 0;
//   }
// }

/* ------------------- DRIVE + PITCH PID ------------------- */
void drive_Movement() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
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
      float D = KD_PITCH * (pitchError - prevPitchError) / max(dt, 0.001f);
      prevPitchError = pitchError;
      finalSpeed += (int)(P + I + D);
    } else {
      pitchIntegral *= 0.95f;
    }
  } else {
    pitchIntegral = 0.0f;
    prevPitchError = 0.0f;
  }

  finalSpeed = constrain(finalSpeed, -255, 255);

  if (controllerConnected && (tuning == TuningSession::NONE) && enableDrive && abs(finalSpeed) > 5) {
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
  if (EnableFlywheel && (tuning == TuningSession::NONE) && enableDrive && buttonsL.l1) {
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
/* ------------------- SERIAL + ESPNOW COMMAND HANDLER ------------------- */
void handleSerialCommands(String cmd) {
  // If no explicit command, read from primary Serial (USB) non-blocking
  if (cmd == "" && Serial.available()) {
    cmd = Serial.readStringUntil('\n');
  }
  if (cmd == "") return;

  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("SET ")) {
    int spaceIndex = cmd.indexOf(' ', 4);
    String param = cmd.substring(4, spaceIndex);
    String valueStr = cmd.substring(spaceIndex + 1);
    valueStr.toUpperCase();

    bool boolVal = (valueStr == "ON" || valueStr == "TRUE" || valueStr == "1");
    float numVal = valueStr.toFloat();

    // PID tuning
    if (param == "KP") { KP_PITCH = numVal; preferences.putFloat("kp_pitch", KP_PITCH); }
    else if (param == "KD") { KD_PITCH = numVal; preferences.putFloat("kd_pitch", KD_PITCH); }
    else if (param == "KI") { KI_PITCH = numVal; preferences.putFloat("ki_pitch", KI_PITCH); }
    else if (param == "PK2") { Pk2 = numVal; preferences.putFloat("pk2", Pk2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); }
    else if (param == "DK2") { Dk2 = numVal; preferences.putFloat("dk2", Dk2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); }
    else if (param == "IK2") { Ik2 = numVal; preferences.putFloat("ik2", Ik2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); }

    // Debug toggles
    else if (param == "DEBUG_ALL") DEBUG_ALL = boolVal;
    else if (param == "DEBUG_COMPACT") DEBUG_COMPACT = boolVal;
    else if (param == "DEBUG_CALIBRATION") DEBUG_CALIBRATION = boolVal;
    else if (param == "DEBUG_JOYSTICK") DEBUG_JOYSTICK = boolVal;
    else if (param == "DEBUG_S2S_MODE") DEBUG_S2S_MODE = boolVal;
    else if (param == "DEBUG_MOTOR_OUTPUT") DEBUG_MOTOR_OUTPUT = boolVal;
    else if (param == "DEBUG_IMU_RAW") DEBUG_IMU_RAW = boolVal;

    // Feature toggles and commands

    else if (param == "WIFI_SSID") {
        wifiSSID = valueStr;
        preferences.putString("wifiSSID", wifiSSID);
        Serial.printf("[WiFi] SSID set to: %s\n", wifiSSID.c_str());
    }
    else if (param == "WIFI_PASS") {
        wifiPassword = valueStr;
        preferences.putString("wifiPassword", wifiPassword);
        Serial.println("[WiFi] Password updated.");
    }
    else if (param == "USE_ESPNOW") {
        USE_ESPNOW = boolVal;
        preferences.putBool("useEspNow", USE_ESPNOW);
        emitSound(SOUND_TOGGLE_ESPNOW);
    }

    else if (param == "OTA") {
        OTA_ENABLED = boolVal;
        preferences.putBool("otaEnabled", OTA_ENABLED);
        emitSound(SOUND_TOGGLE_OTA);
    }

    else if (param == "ENABLE_ROLL_BIAS") {
        ENABLE_ROLL_BIAS = boolVal;
        preferences.putBool("enableRollBias", ENABLE_ROLL_BIAS);
        emitSound(SOUND_TOGGLE_ROLL_BIAS);
    }

    else if (param == "SMOOTH_BALANCE") {
        ENABLE_SMOOTH_BALANCE = boolVal;
        preferences.putBool("smoothBalance", ENABLE_SMOOTH_BALANCE);
        Serial.printf("[SMOOTH BALANCE] %s\n", ENABLE_SMOOTH_BALANCE ? "Enabled" : "Disabled");
    }

    else if (param == "FACTORY_RESET") {
        preferences.clear();
        Serial.println("[CMD] Factory reset complete. Rebooting...");
        delay(1000);
        ESP.restart();
    }

    else if (param == "REVERSE_S2S") {
        REVERSE_S2S = boolVal;
        preferences.putBool("reverseS2S", REVERSE_S2S);
    }
    
    else if (param == "REVERSE_DRIVE") {
        reverseDrive = boolVal;
        preferences.putBool("reverseDrive", reverseDrive);
    }

    else if (param == "USE_ESPNOW") {
      USE_ESPNOW = boolVal;
      preferences.putBool("useEspNow", USE_ESPNOW);
      if (USE_ESPNOW) {
          initEspNow(); // Activate immediately
      } else {
          esp_now_deinit();
      }
    }

    else if (param == "TELEMETRY") {
        ENABLE_DIAGNOSTIC_TELEMETRY = boolVal;
        applyTelemetrySetting();
        emitSound(SOUND_TOGGLE_TELEMETRY);
    }
    

    LOGF("[CMD] %s set to %s\n", param.c_str(), boolVal ? "ON" : String(numVal, 2).c_str());
  }
  else if (cmd == "SHOW PID") {
    String status =
      String("[PID] Pitch: KP=") + String(KP_PITCH, 2) +
      " KD=" + String(KD_PITCH, 2) +
      " KI=" + String(KI_PITCH, 2) +
      " | Roll: Pk2=" + String(Pk2, 2) +
      " Dk2=" + String(Dk2, 2) +
      " Ik2=" + String(Ik2, 2);
    LOGF("%s\n", status.c_str());
  }
  else if (cmd == "SHOW CONFIG") {
    String status = String("[CONFIG] REVERSE_S2S=") + (REVERSE_S2S ? "ON" : "OFF") +
                    " REVERSE_DRIVE=" + (reverseDrive ? "ON" : "OFF") +
                    " USE_ESPNOW=" + (USE_ESPNOW ? "ON" : "OFF") +
                    " DEBUG_ALL=" + String(DEBUG_ALL ? "ON" : "OFF") +
                    " DEBUG_COMPACT=" + String(DEBUG_COMPACT ? "ON" : "OFF") +
                    " DEBUG_CALIBRATION=" + String(DEBUG_CALIBRATION ? "ON" : "OFF") +
                    " DEBUG_JOYSTICK=" + String(DEBUG_JOYSTICK ? "ON" : "OFF") +
                    " DEBUG_S2S_MODE=" + String(DEBUG_S2S_MODE ? "ON" : "OFF") +
                    " DEBUG_MOTOR_OUTPUT=" + String(DEBUG_MOTOR_OUTPUT ? "ON" : "OFF") +
                    " DEBUG_IMU_RAW=" + String(DEBUG_IMU_RAW ? "ON" : "OFF") +
                    " TELEMETRY=" + String(ENABLE_DIAGNOSTIC_TELEMETRY ? "ON" : "OFF");
    LOGF("%s\n", status.c_str());
  }
  else if (cmd == "SHOW WIFI") {
      Serial.printf("[WiFi Config] SSID: %s | Password: %s\n",
                    wifiSSID.c_str(),
                    wifiPassword.length() > 0 ? "********" : "(none)");
  }
  else if (cmd == "SAVE CALIBRATION") {
    saveCalibration();
    LOGLN("[CALIBRATION SAVED]");
  }
  else if (cmd == "RESET CALIBRATION") {
    resetCalibration();
    LOGLN("[CALIBRATION RESET]");
  }
  else if (cmd == "RESET_32U4") {
    if (USE_ESPNOW) sendStatusESPNOW("[RESET COMMAND SENT]");

    // Send the reset command to 32u4 via EasyTransfer
    sendTo32u4Data.soundcmd = -99; // Use a special value as reset trigger
    send32u4.sendData();
    Serial.println("[CMD] Reset command sent to 32u4");
  }
  else {
    LOGLN("[CMD] Unknown command");
  }
}
// #endif

/* ------------------- LIVE TUNING: HOLD DETECTION ------------------- */
void handleTuningHold() {
  unsigned long now = millis();

  // CANCEL current tuning: PS + DOWN (drive) held for 3s
  if (tuning != TuningSession::NONE && buttonsR.ps && buttonsR.down) {
    if (!tuningHoldActive) { tuningHoldActive = true; tuningHoldStart = now; }
    if (tuningHoldActive && now - tuningHoldStart >= CALIB_HOLD_TIME) {
      cancelLiveTuning(true);
      return;
    }
  } else if (tuning != TuningSession::NONE) {
    tuningHoldActive = false;
  }

  // Start new tuning sessions (only when NOT already tuning)
  if (tuning == TuningSession::NONE) {
    // Pitch session: PS+UP held 3s
    if (buttonsR.ps && buttonsR.up) {
      if (!tuningHoldActive) { tuningHoldActive = true; tuningHoldStart = now; }
      if (tuningHoldActive && now - tuningHoldStart >= CALIB_HOLD_TIME) {
        kpPitchWork = 45.0f; kdPitchWork = 0.0f; kiPitchWork = 0.0f;
        KP_PITCH = kpPitchWork; KD_PITCH = kdPitchWork; KI_PITCH = kiPitchWork;
        autoBalance = false;
        tuning = TuningSession::PITCH_KP;
        printTuningBanner();
        LOGLN("[TUNE] Pitch: Stage 1 (KP) — UP/DOWN adjust, X to store (25% reduction)");
        tuningHoldActive = false;
      }
    }
    // Roll session: PS+RIGHT held 3s
    else if (buttonsR.ps && buttonsR.right) {
      if (!tuningHoldActive) { tuningHoldActive = true; tuningHoldStart = now; }
      if (tuningHoldActive && now - tuningHoldStart >= CALIB_HOLD_TIME) {
        pk2Work = 45.0f; dk2Work = 0.0f; ik2Work = 0.0f;
        Pk2 = pk2Work; Dk2 = dk2Work; Ik2 = ik2Work;
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        tuning = TuningSession::ROLL_PK;
        printTuningBanner();
        LOGLN("[TUNE] Roll: Stage 1 (Pk2) — UP/DOWN adjust, X to store (25% reduction)");
        tuningHoldActive = false;
      }
    } else {
      tuningHoldActive = false;
    }
  }
}

/* ------------------- LIVE TUNING: PROCESS BUTTONS ------------------- */
void handleLiveTuning() {
  static ControllerButtons prevTuneR{};
  buttonsR.rightStickX = 0;
  buttonsR.rightStickY = 0;

  bool upPressed   = EDGE_PRESSED(buttonsR.up,    prevTuneR.up);
  bool downPressed = EDGE_PRESSED(buttonsR.down,  prevTuneR.down);
  bool xPressed    = EDGE_PRESSED(buttonsR.cross, prevTuneR.cross);

  switch (tuning) {
    case TuningSession::PITCH_KP:
      if (upPressed)   { kpPitchWork += KP_STEP; KP_PITCH = kpPitchWork; LOGF("[TUNE] KP %.2f\n", KP_PITCH); }
      if (downPressed) { kpPitchWork = max(0.0f, kpPitchWork - KP_STEP); KP_PITCH = kpPitchWork; LOGF("[TUNE] KP %.2f\n", KP_PITCH); }
      if (xPressed) {
        KP_PITCH = kpPitchWork * (1.0f - REDUCE_FACTOR);
        preferences.putFloat("kp_pitch", KP_PITCH);
        LOGF("[TUNE] KP STORED = %.2f (reduced 25%% from %.2f)\n", KP_PITCH, kpPitchWork);
        kdPitchWork = 0.0f; KD_PITCH = kdPitchWork; preferences.putFloat("kd_pitch", KD_PITCH);
        autoBalance = true;
        LOGLN("[TUNE] Pitch: Stage 2 (KD) — UP/DOWN to adjust, X to store");
        tuning = TuningSession::PITCH_KD;
      }
      break;

    case TuningSession::PITCH_KD:
      if (upPressed)   { kdPitchWork += KD_STEP; KD_PITCH = kdPitchWork; LOGF("[TUNE] KD %.2f\n", KD_PITCH); }
      if (downPressed) { kdPitchWork = max(0.0f, kdPitchWork - KD_STEP); KD_PITCH = kdPitchWork; LOGF("[TUNE] KD %.2f\n", KD_PITCH); }
      if (xPressed) {
        preferences.putFloat("kd_pitch", KD_PITCH);
        LOGF("[TUNE] KD STORED = %.2f\n", KD_PITCH);
        kiPitchWork = 0.0f; KI_PITCH = kiPitchWork; preferences.putFloat("ki_pitch", KI_PITCH);
        LOGLN("[TUNE] Pitch: Stage 3 (KI) — UP/DOWN to adjust (0.1), X to store & exit");
        tuning = TuningSession::PITCH_KI;
      }
      break;

    case TuningSession::PITCH_KI:
      if (upPressed)   { kiPitchWork += KI_STEP; KI_PITCH = kiPitchWork; LOGF("[TUNE] KI %.2f\n", KI_PITCH); }
      if (downPressed) { kiPitchWork = max(0.0f, kiPitchWork - KI_STEP); KI_PITCH = kiPitchWork; LOGF("[TUNE] KI %.2f\n", KI_PITCH); }
      if (xPressed) {
        preferences.putFloat("ki_pitch", KI_PITCH);
        LOGF("[TUNE] KI STORED = %.2f\n", KI_PITCH);
        LOGLN("[TUNE] Pitch tuning complete — exiting live tuning.");
        tuning = TuningSession::NONE;
      }
      break;

    case TuningSession::ROLL_PK:
      if (upPressed)   { pk2Work += PK2_STEP; Pk2 = pk2Work; PID2_S2S.SetTunings(Pk2, Ik2, Dk2); LOGF("[TUNE] Pk2 %.2f\n", Pk2); }
      if (downPressed) { pk2Work = max(0.0f, pk2Work - PK2_STEP); Pk2 = pk2Work; PID2_S2S.SetTunings(Pk2, Ik2, Dk2); LOGF("[TUNE] Pk2 %.2f\n", Pk2); }
      if (xPressed) {
        Pk2 = pk2Work * (1.0f - REDUCE_FACTOR);
        preferences.putFloat("pk2", Pk2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        LOGF("[TUNE] Pk2 STORED = %.2f (reduced 25%% from %.2f)\n", Pk2, pk2Work);
        dk2Work = 0.0f; Dk2 = dk2Work; preferences.putFloat("dk2", Dk2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        LOGLN("[TUNE] Roll: Stage 2 (Dk2) — UP/DOWN to adjust, X to store");
        tuning = TuningSession::ROLL_DK;
      }
      break;

    case TuningSession::ROLL_DK:
      if (upPressed)   { dk2Work += DK2_STEP; Dk2 = dk2Work; preferences.putFloat("dk2", Dk2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); LOGF("[TUNE] Dk2 %.2f\n", Dk2); }
      if (downPressed) { dk2Work = max(0.0f, dk2Work - DK2_STEP); Dk2 = dk2Work; preferences.putFloat("dk2", Dk2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); LOGF("[TUNE] Dk2 %.2f\n", Dk2); }
      if (xPressed) {
        preferences.putFloat("dk2", Dk2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        LOGF("[TUNE] Dk2 STORED = %.2f\n", Dk2);
        ik2Work = 0.0f; Ik2 = ik2Work; preferences.putFloat("ik2", Ik2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        LOGLN("[TUNE] Roll: Stage 3 (Ik2) — UP/DOWN to adjust (0.1), X to store & exit");
        tuning = TuningSession::ROLL_IK;
      }
      break;

    case TuningSession::ROLL_IK:
      if (upPressed)   { ik2Work += IK2_STEP; Ik2 = ik2Work; preferences.putFloat("ik2", Ik2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); LOGF("[TUNE] Ik2 %.2f\n", Ik2); }
      if (downPressed) { ik2Work = max(0.0f, ik2Work - IK2_STEP); Ik2 = ik2Work; preferences.putFloat("ik2", Ik2); PID2_S2S.SetTunings(Pk2, Ik2, Dk2); LOGF("[TUNE] Ik2 %.2f\n", Ik2); }
      if (xPressed) {
        preferences.putFloat("ik2", Ik2);
        PID2_S2S.SetTunings(Pk2, Ik2, Dk2);
        LOGF("[TUNE] Ik2 STORED = %.2f\n", Ik2);
        LOGLN("[TUNE] Roll tuning complete — exiting live tuning.");
        tuning = TuningSession::NONE;
      }
      break;

    default: break;
  }

  prevTuneR = buttonsR;
}

void printTuningBanner() {
  LOGLN("");
  LOGLN("================ LIVE TUNING MODE ================");
  LOGLN("Controls (Drive controller):");
  LOGLN("  • Hold PS+UP    3s → Pitch PID (KP → KD → KI)");
  LOGLN("  • Hold PS+RIGHT 3s → Roll PID  (Pk2 → Dk2 → Ik2)");
  LOGLN("  • UP/DOWN           → adjust current gain");
  LOGLN("  • X                 → store & advance to next stage");
  LOGLN("  • Hold PS+DOWN  3s  → CANCEL tuning (restore persisted gains)");
  LOGLN("  • During tuning: joystick, sounds, dome, flywheel disabled");
  LOGLN("=================================================");
  LOGLN("");
}

void cancelLiveTuning(bool announce) {
  KP_PITCH = preferences.getFloat("kp_pitch", KP_PITCH);
  KD_PITCH = preferences.getFloat("kd_pitch", KD_PITCH);
  KI_PITCH = preferences.getFloat("ki_pitch", KI_PITCH);

  Pk2      = preferences.getFloat("pk2", Pk2);
  Dk2      = preferences.getFloat("dk2", Dk2);
  Ik2      = preferences.getFloat("ik2", Ik2);
  PID2_S2S.SetTunings(Pk2, Ik2, Dk2);

  tuning = TuningSession::NONE;
  tuningHoldActive = false;
  autoBalance = false;

  digitalWrite(S2S_PIN_1, LOW);
  digitalWrite(S2S_PIN_2, LOW);
  analogWrite(S2S_PWM, 0);
  digitalWrite(DRIVE_PIN_1, LOW);
  digitalWrite(DRIVE_PIN_2, LOW);
  analogWrite(DRIVE_PWM, 0);

  if (announce) {
    LOGLN("[TUNE] CANCELED — restored persisted PID gains and exited live tuning.");
    LOGF("[TUNE] Restored Pitch PID: KP=%.2f KD=%.2f KI=%.2f | Roll PID: Pk2=%.2f Dk2=%.2f Ik2=%.2f\n",
         KP_PITCH, KD_PITCH, KI_PITCH, Pk2, Dk2, Ik2);
  }
}

void printTuningStatus() {
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus < 1000) return;
  lastStatus = millis();

  const char* stage =
    (tuning == TuningSession::PITCH_KP) ? "Pitch KP" :
    (tuning == TuningSession::PITCH_KD) ? "Pitch KD" :
    (tuning == TuningSession::PITCH_KI) ? "Pitch KI" :
    (tuning == TuningSession::ROLL_PK)  ? "Roll Pk2" :
    (tuning == TuningSession::ROLL_DK)  ? "Roll Dk2" :
    (tuning == TuningSession::ROLL_IK)  ? "Roll Ik2" : "NONE";

  String status = String("[TUNE STATUS] Stage: ") + stage +
                  " | KP="  + String(KP_PITCH, 2) +
                  " KD="    + String(KD_PITCH, 2) +
                  " KI="    + String(KI_PITCH, 2) +
                  " | Pk2=" + String(Pk2, 2) +
                  " Dk2="   + String(Dk2, 2) +
                  " Ik2="   + String(Ik2, 2);

  LOGF("%s\n", status.c_str());
}

/* ------------------- 3-SEC CALIBRATION HOLD ------------------- */
void handleCalibrationHold() {
  bool bothUp   = buttonsL.up && buttonsR.up;
  bool bothDown = buttonsL.down && buttonsR.down;

  unsigned long now = millis();

  // SAVE (UP+UP)
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

  // RESET (DOWN+DOWN)
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

  LOGF("T: %lu INFO: CAL SAVED → Pot:%d Pitch:%.2f Roll:%.2f\n",
       millis(), potOffset, pitchOffset, rollOffset);
}

void resetCalibration() {
  potOffset = 0;
  pitchOffset = 0.0f;
  rollOffset = 0.0f;

  preferences.putInt("potOffset", 0);
  preferences.putFloat("pitchOffset", 0.0f);
  preferences.putFloat("rollOffset", 0.0f);

  LOGF("T: %lu INFO: CAL RESET → All offsets set to 0\n", millis());
}

/* ------------------- CONTROLLER ------------------- */
void updateControllerStates() {
  drivecontrollerConnected = driveController.isConnected();
  domecontrollerConnected  = domeController.isConnected();
  controllerConnected      = drivecontrollerConnected || domecontrollerConnected;

  if (controllerConnected) {
    // Drive controller (right)
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

    // Dome controller (left)
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


/* ------------------- SOUND MAPPING HELPERS ------------------- */
// static uint16_t resolveDriveControllerSound(const ControllerButtons& cur, const ControllerButtons& prev) {
//   DPad d; d.up = cur.up; d.right = cur.right; d.down = cur.down; d.left = cur.left;

//   // L1 + D-pad combos (21..24)
//   if (cur.l1) {
//     if (EDGE_PRESSED(d.up,    prev.up))    return 21;
//     if (EDGE_PRESSED(d.right, prev.right)) return 22;
//     if (EDGE_PRESSED(d.down,  prev.down))  return 23;
//     if (EDGE_PRESSED(d.left,  prev.left))  return 24;
//   }
//   // R1 + D-pad combos (25..28)
//   if (cur.r1) {
//     if (EDGE_PRESSED(d.up,    prev.up))    return 25;
//     if (EDGE_PRESSED(d.right, prev.right)) return 26;
//     if (EDGE_PRESSED(d.down,  prev.down))  return 27;
//     if (EDGE_PRESSED(d.left,  prev.left))  return 28;
//   }
//   // Single D-pad (1..4)
//   if (EDGE_PRESSED(cur.up,    prev.up))    return 1;
//   if (EDGE_PRESSED(cur.right, prev.right)) return 2;
//   if (EDGE_PRESSED(cur.down,  prev.down))  return 3;
//   if (EDGE_PRESSED(cur.left,  prev.left))  return 4;

//   // L2 analog threshold
//   if (cur.l2 > 60 && prev.l2 <= 60)        return 50;

//   // Circle → random 1..30
//   if (EDGE_PRESSED(cur.circle, prev.circle)) return pickRandom1to30();

//   return SOUND_NONE;
// }

// static uint16_t resolveDomeControllerSound(const ControllerButtons& cur, const ControllerButtons& prev) {
//   DPad d; d.up = cur.up; d.right = cur.right; d.down = cur.down; d.left = cur.left;

//   // L1 + D-pad combos (11..14)
//   if (cur.l1) {
//     if (EDGE_PRESSED(d.up,    prev.up))    return 11;
//     if (EDGE_PRESSED(d.right, prev.right)) return 12;
//     if (EDGE_PRESSED(d.down,  prev.down))  return 13;
//     if (EDGE_PRESSED(d.left,  prev.left))  return 14;
//   }
//   // Single D-pad (5..8)
//   if (EDGE_PRESSED(cur.up,    prev.up))    return 5;
//   if (EDGE_PRESSED(cur.right, prev.right)) return 6;
//   if (EDGE_PRESSED(cur.down,  prev.down))  return 7;
//   if (EDGE_PRESSED(cur.left,  prev.left))  return 8;

//   // L2 analog threshold
//   if (cur.l2 > 60 && prev.l2 <= 60)        return 99;

//   // Circle → random 1..30
//   if (EDGE_PRESSED(cur.circle, prev.circle)) return pickRandom1to30();

//   return SOUND_NONE;
// }

/* ------------------- DEBUG ------------------- */
void printDebugInfo() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug < 1000) return;  // 1 Hz
  lastDebug = millis();

  if (DEBUG_ALL || DEBUG_CALIBRATION || DEBUG_JOYSTICK || DEBUG_S2S_MODE || DEBUG_MOTOR_OUTPUT || DEBUG_IMU_RAW) {
    LOGF("\n=== DEBUG T:%lu ===\n", millis());
  }

  if (DEBUG_ALL || DEBUG_CALIBRATION) {
    int currentPot = filterPotValue(analogRead(S2S_POT_PIN));
    currentPot = constrain(currentPot, S2S_POT_MIN, S2S_POT_MAX);
    float adjPitch = receiveIMUData.pitch - pitchOffset;
    float adjRoll  = receiveIMUData.roll  - rollOffset;

    LOGF("POT: Curr=%4d  SavedCenter=%4d  Err=%+4d\n", currentPot, potOffset, currentPot - potOffset);
    LOGF("IMU: Pitch=%+6.2f (offset=%+6.2f)  Roll=%+6.2f (offset=%+6.2f)  Conn=%s\n",
         adjPitch, pitchOffset, adjRoll, rollOffset, IMUconnected ? "YES" : "NO");
  }

  if (DEBUG_ALL || DEBUG_JOYSTICK) {
    LOGF("DRIVE_JOY: X=%+4d  Y=%+4d  (Deadzone=%d)\n", buttonsR.rightStickX, buttonsR.rightStickY, JOYSTICK_DEADZONE);
    LOGF("DOME_JOY: X=%+4d  Y=%+4d  (Deadzone=%d)\n", buttonsL.leftStickX, buttonsL.leftStickY, JOYSTICK_DEADZONE);
  }

  if (DEBUG_ALL || DEBUG_S2S_MODE) {
    const char* modeStr =
      (s2sMode == S2SMode::JOY) ? "JOY MANUAL" :
      (autoBalance && IMUconnected && fabs(receiveIMUData.roll - rollOffset) > IMU_DEADZONE) ? "PID BALANCE" :
      "POT RETURN";
    LOGF("S2S MODE: %s  |  AutoBal=%s  DriveEn=%s\n", modeStr, autoBalance ? "ON" : "OFF", enableDrive ? "ON" : "OFF");
  }

  if (DEBUG_ALL || DEBUG_MOTOR_OUTPUT) {
    LOGF("MOTORS → S2S: PWM=%3d  DIR=%d%d  |  DRIVE: PWM=%3d\n",
         lastPwmS2S, digitalRead(S2S_PIN_1), digitalRead(S2S_PIN_2), /* Add drive PWM if tracked */ 0);
  }

  if (DEBUG_ALL || DEBUG_IMU_RAW) {
    LOGF("IMU RAW: Pitch=%+6.2f  Roll=%+6.2f\n", receiveIMUData.pitch, receiveIMUData.roll);
  }

  if (DEBUG_ALL || DEBUG_CALIBRATION || DEBUG_JOYSTICK || DEBUG_S2S_MODE || DEBUG_MOTOR_OUTPUT || DEBUG_IMU_RAW) {
    LOGLN("=====================================");
  }
}

/* ------------------- DEBUG FLAGS BANNER ------------------- */
void printActiveDebugFlagsOnce() {
  LOG("DEBUG FLAGS: ");
  if (DEBUG_ALL) LOG("ALL ");
  if (DEBUG_COMPACT) LOG("COMPACT ");
  if (DEBUG_CALIBRATION) LOG("CALIBRATION ");
  if (DEBUG_JOYSTICK) LOG("JOYSTICK ");
  if (DEBUG_S2S_MODE) LOG("S2S_MODE ");
  if (DEBUG_MOTOR_OUTPUT) LOG("MOTOR_OUTPUT ");
  if (DEBUG_IMU_RAW) LOG("IMU_RAW ");
  if (ENABLE_DIAGNOSTIC_TELEMETRY) LOG("TELEMETRY");
  if (!DEBUG_ALL && !DEBUG_COMPACT && !DEBUG_CALIBRATION && !DEBUG_JOYSTICK &&
      !DEBUG_S2S_MODE && !DEBUG_MOTOR_OUTPUT && !DEBUG_IMU_RAW && !ENABLE_DIAGNOSTIC_TELEMETRY) {
    LOG("(none)");
  }
  LOGLN("");
}

/* ------------------- LOOP ------------------- */
void loop() {
  
  if (WiFi.status() == WL_CONNECTED && OTA_ENABLED) {
      ArduinoOTA.handle(); // Non-blocking
  }

  if (USE_ESPNOW) {
    pumpEspNowSends(); // Pump ESPNOW send queue in task context
  }

  esp_task_wdt_reset(); // Feed the watchdog
  
  if (ENABLE_DIAGNOSTIC_TELEMETRY && telemetryDue) {
    telemetryDue = false; // Telemetry (if enabled) — enqueue from task context
    sendTelemetryNow();
  }

  // Process commands typed into PRIMARY Serial Monitor (USB) or ESPNOW
  handleSerialCommands();   // non-blocking unless a line is available

  delay(1);
}
