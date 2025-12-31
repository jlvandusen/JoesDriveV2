
/*
 * Dome ESP32 HUZZAH32 Controller
 * ESPNOW Communication + LED Animations + Battery Reporting
 * Written by James VanDusen (Extended)
 */

#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>

// -------------------- CONFIG --------------------
#define psiPIN        25
#define sLogicPIN     27
#define lLogicPIN     33
#define hpPIN         15
#define eyePIN        32
#define battPin       A13

#define psiPixels     1
#define sLogicPixels  1
#define lLogicPixels  1
#define hpPixels      1
#define eyePixels     1

#define BRIGHTNESS    64

// ESPNOW MAC of Body ESP32
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4};

// -------------------- STRUCT --------------------
typedef struct struct_message {
  int psi;       // PSI trigger
  byte btn;      // Button state
  float bat;     // Battery voltage
  int anim;      // Animation code (1–99)
} struct_message;

struct_message incomingReadings; // From Body
struct_message outgoingData;     // To Body

// -------------------- LED OBJECTS --------------------
Adafruit_NeoPixel PSI(psiPixels, psiPIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel sLOGIC(sLogicPixels, sLogicPIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lLOGIC(lLogicPixels, lLogicPIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel HP(hpPixels, hpPIN, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel EYE(eyePixels, eyePIN, NEO_GRB + NEO_KHZ800);

// -------------------- TIMERS --------------------
unsigned long sLogicLastChange = 0;
unsigned long lLogicLastChange = 0;
unsigned long EYELastChange = 0;
int sLogicChangeInterval = random(3000, 10000);
int lLogicChangeInterval = random(3000, 10000);
int EYEChangeInterval = random(3000, 10000);

bool shouldFlicker = false;
unsigned long flickerEndTime = 0;

unsigned long lastSendTime = 0;
const int sendInterval = 1000; // Send battery every second

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Init LEDs
  PSI.begin(); PSI.setBrightness(BRIGHTNESS);
  sLOGIC.begin(); sLOGIC.setBrightness(BRIGHTNESS);
  lLOGIC.begin(); lLOGIC.setBrightness(BRIGHTNESS);
  HP.begin(); HP.setBrightness(BRIGHTNESS);
  EYE.begin(); EYE.setBrightness(BRIGHTNESS);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Dome Controller Ready!");
}

// -------------------- LOOP --------------------
void loop() {
  // Idle animations
  displayPSI();
  displayEYE();
  displayHP();
  displaysLOGIC();
  displaylLOGIC();

  // Run animation if animCode > 0
  if (incomingReadings.anim > 0) {
    handleAnimation(incomingReadings.anim);
  }

  // Send battery status back every second
  if (millis() - lastSendTime > sendInterval) {
    outgoingData.psi = incomingReadings.psi;
    outgoingData.btn = incomingReadings.btn;
    outgoingData.bat = readBatteryVoltage();
    outgoingData.anim = 0; // Optional: send current anim if needed

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
    if (result == ESP_OK) {
      Serial.print("Battery sent: ");
      Serial.println(outgoingData.bat);
    } else {
      Serial.println("Send failed");
    }
    lastSendTime = millis();
  }
}

// -------------------- ESPNOW CALLBACKS --------------------
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Received PSI: "); Serial.print(incomingReadings.psi);
  Serial.print(" | Anim: "); Serial.println(incomingReadings.anim);

  if (incomingReadings.psi != 0 && !shouldFlicker) {
    shouldFlicker = true;
    flickerEndTime = millis() + random(1000, 3000);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// -------------------- BATTERY --------------------
float readBatteryVoltage() {
  int raw = analogRead(battPin);
  float voltage = (raw / 4095.0) * 3.3; // ADC to volts
  voltage *= 2; // Adjust if using a divider
  return voltage;
}

// -------------------- LED FUNCTIONS --------------------
void displayPSI() {
  if (shouldFlicker) {
    if (millis() <= flickerEndTime) {
      flickerDisplay();
    } else {
      shouldFlicker = false;
      PSI.clear();
      PSI.show();
    }
  }
}

void flickerDisplay() {
  int blueLevel = random(100, 255);
  PSI.setPixelColor(0, PSI.Color(0, 0, blueLevel));
  PSI.show();
}

void displayEYE() {
  if (millis() - EYELastChange > EYEChangeInterval) {
    int eyeBrightness = random(50, 255);
    EYE.setPixelColor(0, EYE.Color(eyeBrightness, 0, 0)); // Red flicker
    EYE.show();
    EYELastChange = millis();
    EYEChangeInterval = random(3000, 10000);
  }
}

void displayHP() {
  HP.setPixelColor(0, HP.Color(128, 128, 128)); // White at 50%
  HP.show();
}

void displaysLOGIC() {
  if (millis() - sLogicLastChange > sLogicChangeInterval) {
    uint32_t colors[] = {sLOGIC.Color(255, 255, 0), sLOGIC.Color(255, 0, 0), sLOGIC.Color(255, 255, 255)};
    sLOGIC.setPixelColor(0, colors[random(0, 3)]);
    sLOGIC.show();
    sLogicLastChange = millis();
    sLogicChangeInterval = random(3000, 10000);
  }
}

void displaylLOGIC() {
  if (millis() - lLogicLastChange > lLogicChangeInterval) {
    uint32_t colors[] = {lLOGIC.Color(255, 255, 0), lLOGIC.Color(255, 0, 0), lLOGIC.Color(255, 255, 255)};
    lLOGIC.setPixelColor(0, colors[random(0, 3)]);
    lLOGIC.show();
    lLogicLastChange = millis();
    lLogicChangeInterval = random(3000, 10000);
  }
}

// -------------------- ANIMATION HANDLER --------------------
void handleAnimation(int animCode) {
  switch(animCode) {
    case 1: setAllLights(255, 255, 255); break; // All white
    case 2: radarEyePulse(); break;             // Eye pulse red
    case 3: psiBlueFlicker(); logicRainbow(); break; // PSI flicker + rainbow logic
    case 4: setAllLights(255, 0, 0); break;     // All red
    case 5: setAllLights(255, 255, 0); break;   // All yellow
    case 6: alternatingLogic(); break;          // Logic lights alternate
    case 7: rainbowCycle(); break;              // Full rainbow cycle
    case 8: specialSoundEffect(); break;        // NEW: Special sound animation
    case 9: psiPulseBlue(); break;              // PSI pulse blue
    case 10: tiltEffect(); break;               // NEW: Tilt animation
    default: clearAllLights(); break;
  }
}


// -------------------- ANIMATION FUNCTIONS --------------------
void setAllLights(int r, int g, int b) {
  PSI.setPixelColor(0, PSI.Color(r,g,b));
  sLOGIC.setPixelColor(0, sLOGIC.Color(r,g,b));
  lLOGIC.setPixelColor(0, lLOGIC.Color(r,g,b));
  HP.setPixelColor(0, HP.Color(r,g,b));
  EYE.setPixelColor(0, EYE.Color(r,g,b));
  PSI.show(); sLOGIC.show(); lLOGIC.show(); HP.show(); EYE.show();
}

void clearAllLights() {
  PSI.clear(); sLOGIC.clear(); lLOGIC.clear(); HP.clear(); EYE.clear();
  PSI.show(); sLOGIC.show(); lLOGIC.show(); HP.show(); EYE.show();
}

void radarEyePulse() {
  int brightness = abs(sin(millis() / 200.0) * 255);
  EYE.setPixelColor(0, EYE.Color(brightness, 0, 0));
  EYE.show();
}

void psiBlueFlicker() {
  int blueLevel = random(100, 255);
  PSI.setPixelColor(0, PSI.Color(0, 0, blueLevel));
  PSI.show();
}

void logicRainbow() {
  static uint16_t hue = 0;
  sLOGIC.setPixelColor(0, sLOGIC.gamma32(sLOGIC.ColorHSV(hue)));
  lLOGIC.setPixelColor(0, lLOGIC.gamma32(lLOGIC.ColorHSV(hue + 10000)));
  sLOGIC.show(); lLOGIC.show();
  hue += 256;
}

void alternatingLogic() {
  static bool toggle = false;
  if (toggle) {
    sLOGIC.setPixelColor(0, sLOGIC.Color(255, 0, 0));
    lLOGIC.setPixelColor(0, lLOGIC.Color(255, 255, 0));
  } else {
    sLOGIC.setPixelColor(0, sLOGIC.Color(255, 255, 0));
    lLOGIC.setPixelColor(0, lLOGIC.Color(255, 0, 0));
  }
  sLOGIC.show(); lLOGIC.show();
  toggle = !toggle;
}

void rainbowCycle() {
  static uint16_t hue = 0;
  PSI.setPixelColor(0, PSI.gamma32(PSI.ColorHSV(hue)));
  sLOGIC.setPixelColor(0, sLOGIC.gamma32(sLOGIC.ColorHSV(hue + 5000)));
  lLOGIC.setPixelColor(0, lLOGIC.gamma32(lLOGIC.ColorHSV(hue + 10000)));
  HP.setPixelColor(0, HP.gamma32(HP.ColorHSV(hue + 15000)));
  EYE.setPixelColor(0, EYE.gamma32(EYE.ColorHSV(hue + 20000)));
  PSI.show(); sLOGIC.show(); lLOGIC.show(); HP.show(); EYE.show();
  hue += 256;
}

void eyeBlink() {
  static bool on = false;
  if (on) {
    EYE.setPixelColor(0, EYE.Color(255, 0, 0));
  } else {
    EYE.setPixelColor(0, EYE.Color(0, 0, 0));
  }
  EYE.show();
  on = !on;
}

void psiPulseBlue() {
  int brightness = abs(sin(millis() / 200.0) * 255);
  PSI.setPixelColor(0, PSI.Color(0, 0, brightness));
  PSI.show();
}

void hpFlash() {
  static bool on = false;
  if (on) {
    HP.setPixelColor(0, HP.Color(255, 255, 255));
  } else {
    HP.setPixelColor(0, HP.Color(0, 0, 0));
  }
  HP.show();
  on = !on;
}


void specialSoundEffect() {
  // Flash all lights white rapidly for special sound
  static bool toggle = false;
  if (toggle) {
    setAllLights(255, 255, 255);
  } else {
    setAllLights(0, 0, 0);
  }
  toggle = !toggle;
}


void tiltEffect() {
  // Simulate tilt by alternating logic lights and eye brightness
  static bool toggle = false;
  if (toggle) {
    sLOGIC.setPixelColor(0, sLOGIC.Color(255, 255, 0)); // Yellow
    lLOGIC.setPixelColor(0, lLOGIC.Color(255, 0, 0));   // Red
    EYE.setPixelColor(0, EYE.Color(255, 0, 0));         // Bright red
  } else {
    sLOGIC.setPixelColor(0, sLOGIC.Color(0, 0, 255));   // Blue
    lLOGIC.setPixelColor(0, lLOGIC.Color(255, 255, 255)); // White
    EYE.setPixelColor(0, EYE.Color(128, 0, 0));         // Dim red
  }
  sLOGIC.show();
  lLOGIC.show();
  EYE.show();
  toggle = !toggle;
}

