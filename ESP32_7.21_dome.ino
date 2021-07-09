/*
 * Joe's Drive  - V2 7/7/2021
 * Dome ESP32 HUZZAH32
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need libraries: 
 * AdafruitNeopixel: https://github.com/adafruit/Adafruit_NeoPixel
 * ESp32 Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * replace this with the mac of body uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
*/
#define debugBody // Validate and feedback what is being received from Body (PSI, BTN and Bat)
//#define ESPNOWCONFIG  // Configure the system to display its wifi mac and bluetooth addresses for insert into Body code.

/*
 * PIN DEFINITIONS
*/

#define battPin     26 //A0    
#define psiPIN      25 //A1
#define sLogicPIN   12
#define lLogicPIN   27 
#define hpPIN       33
#define eyePIN      15

#define dataDelay   0
#define recDelay    10
#define interval    40
#define interval2   2 
#define sendDelay   50 
  
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include "WiFi.h"

/*
 * *********** IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE MAC ADDRESS ************
 * Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
 * As per the following walkthrough https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 * broadcastAddress REPLACE WITH THE MAC Address of your receiver - the other ESP32 in the body of BB8 7C:9E:BD:D7:63:C4
*/
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4}; // Body ESP32

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define analogWrite ledcWrite  // Allows ESP32 to function using its own ledcwrite function when dealing with analog pins

/* 
 *  Define variables to store incoming readings
*/
int incomingPSI = 0;
byte incomingBTN = 0;
float incomingBAT = 0;      

/*
 * Define variables to store readings to be sent
*/
int sendPSI = 0;
byte sendHP = 0;
float sendBAT = 0;    

/*
 * Variable to store if sending data was successful
*/

String success;

// Structure to send data
// Must match the receiver structure
typedef struct struct_message {
    int psi;
    byte btn;
    float bat;
} struct_message;

// Create a struct_message called outgoingReadings to hold sensor readings
struct_message outgoingReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

byte i, a, b, Set = 1, s;
bool blinkState = false;
float domeBatt1; 

Adafruit_NeoPixel sLOGIC = Adafruit_NeoPixel(3, sLogicPIN, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel lLOGIC = Adafruit_NeoPixel(6, lLogicPIN, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel HP = Adafruit_NeoPixel(1, hpPIN, NEO_RGB + NEO_KHZ800); 
Adafruit_NeoPixel EYE = Adafruit_NeoPixel(1, eyePIN, NEO_GRB + NEO_KHZ800); 

unsigned long randomMillis, previousMillis, previousMillis2, lastSendRecMillis;
unsigned long lastHPCycleMillis, randomMillisSingle, but4StateMillis, lastBattUpdate;
unsigned long lastBodyReceive, lastFlash;
int psiVal;

const byte numChars = 32;
int LEDState = 1;
int but4State;
int flashtime;
int holoPulseState = 2;
int bpulse = 80;

int hpCycleState, hpRed, hpGreen, hpBlue;

int rearFadeState, rearFadeRed, rearFadeBlue, rearFadeGreen;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  sLOGIC.begin();
  lLOGIC.begin();
  HP.begin();
  EYE.begin();

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
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  eyeLED();
 
}

void loop () {
  debugreceivebody();
  sendReadings();
  ledControls();
}
