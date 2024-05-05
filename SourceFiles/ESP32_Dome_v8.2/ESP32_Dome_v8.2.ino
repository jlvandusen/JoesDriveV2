/*
 * Joe's Drive  - V2 5/1/2024
 * Dome ESP32 HUZZAH32
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need libraries: 
 * AdafruitNeopixel: https://github.com/adafruit/Adafruit_NeoPixel
 * ESp32 Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * replace this with the mac of body uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
*/
#define SerialDebug Serial  // redirect debug to alternate Serial ports
//#define debugDome
#define debugBody // Validate and feedback what is being received from Body (PSI, BTN and Bat)
//#define debugRecieveESPNOW // Raw values over the ESPNOW channel
//#define debugSendESPNOW
//#define ESPNOWCONFIG  // Configure the system to display its wifi mac and bluetooth addresses for insert into Body code.
//#define DebugESPNOW
/*
 * PIN DEFINITIONS
*/

#define battPin     A13 //A0    
#define psiPIN      25 //A1 // Use the appropriate GPIO pin (e.g., GPIO16)
#define psiPixels    1 // Number of LEDs in your NeoPixel strip
#define sLogicPIN   27
#define lLogicPIN   33 
#define hpPIN       15
#define eyePIN      32

#define dataDelay   0
#define recDelay    10
#define interval    40
#define interval2   2 
#define sendDelay   50 

#define LED_PIN     15
#define NUM_LEDS    1
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

#include <Adafruit_NeoPixel.h>
#include <analogWrite.h>  // https://www.arduinolibraries.info/libraries/esp32-analog-write
#include <esp_now.h>
#include <WiFi.h>


/* 
 *  Define variables to store incoming readings
*/
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4}; // Body ESP32
int incomingPSI = 0;
byte incomingBTN = 0;
float incomingBAT = 0;      

unsigned long randomMillis, previousMillis, previousMillis2, lastSendRecMillis;
unsigned long lastHPCycleMillis, randomMillisSingle, BTNstateMillis, lastBattUpdate;
unsigned long lastBodyReceive, lastFlash;
int psiVal;
unsigned long startTime = millis();
unsigned long duration = 3000; // 3 seconds

// Structure to send data
// Must match the receiver structure
typedef struct struct_message {
    int psi;
    byte btn;
    float bat;
    int dis;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

#define PIN_NEO_PIXEL 25  // Use the appropriate GPIO pin (e.g., GPIO16)
#define NUM_PIXELS 1     // Number of LEDs in your NeoPixel strip

// Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel sLOGIC = Adafruit_NeoPixel(1, sLogicPIN, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel lLOGIC = Adafruit_NeoPixel(1, lLogicPIN, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel HP = Adafruit_NeoPixel(1, hpPIN, NEO_RGB + NEO_KHZ800); 
Adafruit_NeoPixel EYE = Adafruit_NeoPixel(1, eyePIN, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel PSI = Adafruit_NeoPixel(psiPixels, psiPIN, NEO_GRB + NEO_KHZ800);

void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(5000);  
  PSI.begin();  // Initialize NeoPixel strip
  PSI.setBrightness(50);  // Set overall brightness (0-255)

  #ifdef ESPNOWCONFIG
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("Please write down the following WIFI MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("WIFI ESPNOW Ready.");
  #endif

  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) { // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.print("My MAC Address is: "); Serial.println(WiFi.macAddress());
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;   // Register peer ESPNOW chip
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;


  if (esp_now_add_peer(&peerInfo) != ESP_OK) {   // Add peer        
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // check_SND_Timing();
  displayPSI();
  debugRoutines();

}

void displayPSI () {
 startTime = millis();
      for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
          // Generate random white level (adjust as needed)
          int whiteLevel = 255;
          PSI.setPixelColor(pixel, PSI.Color(whiteLevel, whiteLevel, whiteLevel));
      }
      PSI.show(); // Apply the changes
      // delay(50); // Flicker rate (adjust as needed)


  // if (incomingPSI != 0 || psiVal == 1){
  //   psiVal = 1;
  //   // while (random(0,3000)) {
  //     // Flicker effect: Randomly adjust brightness for each pixel
  //   for (int i = 0; i < NUM_PIXELS; i++) {
  //       int brightness = random(0, 256);  // Random brightness value
  //       PSI.setPixelColor(i, PSI.Color(brightness, brightness, brightness));
  //   }
  //   PSI.show();  // Update NeoPixel colors
  //   delay(random(0, 256));    // Adjust delay for flicker speed
  //   // }
    
  // } else {
  //   psiVal = 0;
  //   PSI.setBrightness(0); // Set brightness to 0 (fully off)
  //   // PSI.setPixelColor(NUM_PIXELS, PSI.Color(brightness, brightness, brightness));
  //   PSI.show();  // Update NeoPixel colors
  //   // delay(100);    // Adjust delay for flicker speed
  // }
}

/*  
 *  ESPNOW Callback when data is received
 *  See walkthrough: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
*/
 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  #ifdef debugRecieveESPNOW
  Serial.print("Bytes received: ");
  Serial.println(len);
  #endif
  incomingPSI = incomingReadings.psi;
  incomingBTN = incomingReadings.btn;
  incomingBAT = incomingReadings.bat;
}

/* 
 *  ESPNOW Callback when data is sent
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef debugSendESPNOW
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  #endif 
}

void check_SND_Timing(){
//   if (incomingPSI != 0) {
//     flickerPSI(); // Continue flickering for 3 seconds
//   } else {
//     while (millis() - startTime < 3000) {
//             flickerPSI();
//     }
//         // Turn off NeoPixel
//     PSI.setBrightness(0); // Set brightness to 0 (fully off)
//     // PSI.setPixelColor(NUM_PIXELS, PSI.Color(brightness, brightness, brightness));
//     PSI.show();  // Update NeoPixel colors
//   }
}

void sendAndReceive(){
  if(millis() - lastSendRecMillis >= recDelay){
    // PSILED(); 
    lastBodyReceive = millis();
  }
  lastSendRecMillis = millis(); 
  // battLevel();
}

void debugRoutines() {
#ifdef debugBody
// if (incomingPSI != 0 || incomingBTN != 0 || incomingBAT != 0.00) {
  Serial.print(F("incomingPSI: ")); Serial.print(incomingPSI); Serial.print('\t'); 
  Serial.print(F("incomingBTN: ")); Serial.print(incomingBTN); Serial.print('\t');
  // Serial.print(F("BTNstate: ")); Serial.print(BTNstate); Serial.print('\t');
  Serial.print(F("incomingBAT: ")); Serial.print(incomingBAT); Serial.println('\t'); 
  // Serial.print(F("Dome/BAT: ")); Serial.print(sendBAT); Serial.print('\t'); 
  // Serial.print(F("incomingPSI: ")); Serial.print(incomingPSI); Serial.println('\t');  
// }
#endif
}