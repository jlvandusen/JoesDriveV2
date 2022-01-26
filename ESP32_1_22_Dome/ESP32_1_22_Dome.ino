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
#define psiPIN      25 //A1
#define sLogicPIN   27
#define lLogicPIN   33 
#define hpPIN       15
#define eyePIN      32

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
//#define analogWrite ledcWrite  // Allows ESP32 to function using its own ledcwrite function when dealing with analog pins
#include <analogWrite.h>  // https://www.arduinolibraries.info/libraries/esp32-analog-write

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
int sendDIS = 0;

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
    int dis;
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
Adafruit_NeoPixel PSI = Adafruit_NeoPixel(1, psiPIN, NEO_GRB + NEO_KHZ800);

unsigned long randomMillis, previousMillis, previousMillis2, lastSendRecMillis;
unsigned long lastHPCycleMillis, randomMillisSingle, BTNstateMillis, lastBattUpdate;
unsigned long lastBodyReceive, lastFlash;
int psiVal;

const byte numChars = 32;
int LEDState = 0;
int BTNstate;
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
  eyeLED();
 
}

void loop () {
  debugRoutines();
  sendReadings();
  ledControls();
}

/*  
 *  ESPNOW Send readings back to the Body ESP32
 *  See walkthrough: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 */
 
void sendReadings() {
  outgoingReadings.psi = sendPSI;
  outgoingReadings.btn = sendHP;
  outgoingReadings.bat = sendBAT;
  outgoingReadings.dis = sendDIS;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
  #ifdef DebugESPNOW
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  #endif
}

/*  
 *  Led Decision making based on times and state of buttons / psi
*/

void ledControls() {
  if(millis() - previousMillis > interval) {
    previousMillis = millis();
    if (incomingBTN == 1 && BTNstate < 2) {  // check button presses being passed from Body
      LED_State();
    } else if (incomingBTN == 0 && BTNstate != 0) {
      BTNstate = 0;
    }
    
    if (LEDState == 1) {
      doubleLogic();
      Holo();
      rearLogic();
    } else if (LEDState == 2) {
      doubleLogicFade();
      holoPulse();
      rearLogicFade();
      if(hpCycleState != 0) {
        hpCycleState = 0;
      }
    } else if (LEDState == 3) {
      doubleLogicRandom();
      hpCycle();
      rearLogicRandom();
    }
  

  }
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


void rearLogic() {
  for(int i = 0; i < 3; i++) {
    sLOGIC.setPixelColor(i, sLOGIC.Color(23, 67, 96)); //small logic is light blue
    sLOGIC.show();
  }
}
    
void doubleLogic() {
  lLOGIC.setPixelColor(0, lLOGIC.Color(47, 49, 50)); //large logic is almost white
  lLOGIC.setPixelColor(1, lLOGIC.Color(47, 49, 50)); //large logic is almost white
  lLOGIC.setPixelColor(2, lLOGIC.Color(47, 49, 50)); //large logic is almost white
  lLOGIC.setPixelColor(3, lLOGIC.Color(0, 0, 0)); 
  lLOGIC.setPixelColor(4, lLOGIC.Color(0, 0, 0)); 
  lLOGIC.setPixelColor(5, lLOGIC.Color(0, 0, 0)); 
  lLOGIC.show();
}        
  
void eyeLED() { 
  EYE.setPixelColor(0, EYE.Color(0, 200, 0)); // Eye is red
  EYE.show();
}     
  
void Holo() {
  HP.setPixelColor(0, HP.Color(0, 0, 0));
  HP.show();
} 
  
void PSILED(){
  if(incomingPSI != 0){
    if(millis() - lastFlash > flashtime){
      lastFlash = millis();
      flashtime = random(20,70);
      if(psiVal == 255){
        psiVal = 60;
      } else {
        psiVal = 255; 
      }
    }     
  } else {
      psiVal = 0;
    }
  analogWrite(psiPIN, psiVal);  
}

void doubleLogicRandom() {
  if(millis() - randomMillis > 300) {
    int random_i = random(0,6);
    lLOGIC.setPixelColor(random_i, lLOGIC.Color(random(0,255),random(0,255),random(0,255))); 
    lLOGIC.show();
    randomMillis = millis();
  }
}

void doubleLogicFade() {
  if(Set == 1){
    a++;
    if (b > 0){
      b--;
    }
  } else if (Set == 2) {
    a--;
    b++;
  }
  constrain(a, 0, 50);
  constrain(b, 0, 50);
  
  if(a== 50) {
    Set=2;
  } else if (a == 0){
    Set=1;
  }
  lLOGIC.setPixelColor(0, lLOGIC.Color(a, a, a)); 
  lLOGIC.setPixelColor(1, lLOGIC.Color(a, a, a)); 
  lLOGIC.setPixelColor(2, lLOGIC.Color(a, a, a)); 
  lLOGIC.setPixelColor(3, lLOGIC.Color(b, b, b));
  lLOGIC.setPixelColor(4, lLOGIC.Color(b, b, b)); 
  lLOGIC.setPixelColor(5, lLOGIC.Color(b, b, b));  
  lLOGIC.show();
    
}

void LED_State() {
  if(BTNstate == 0){
    BTNstateMillis = millis();
    BTNstate = 1;
  }
  if(BTNstate == 1 && (millis() - BTNstateMillis > 400)){
    if(LEDState == 3){
      LEDState = 1;
    }else{
      LEDState++;
    }
  BTNstate = 2;
  }
}



void holoPulse() {
  if(holoPulseState == 1 && bpulse <= 155){
    bpulse++;
  }else if(bpulse >= 155){
    holoPulseState = 2;
  }

  if(holoPulseState == 2 && bpulse >= 80){
    bpulse--;
  }else if(bpulse <= 80){
    holoPulseState = 1;
  }
  
  HP.setPixelColor(0, HP.Color(0, 0, bpulse));
  HP.show();
}
  
  
void hpCycle() {
  if(hpCycleState == 0){
    hpRed = 0;
    hpGreen = 0;
    hpBlue = 0;
    hpCycleState = 1;
  }else if(hpCycleState == 1){
    if( hpRed <= 250){
      hpRed += 3;
    }else{
      hpCycleState = 2;
    }
  }else if(hpCycleState == 2){
    if(hpRed >= 10){
      hpRed -= 3;
    }else {
      hpCycleState = 3;
    }
  } else if(hpCycleState == 3){
    if(hpGreen <= 250){
      hpGreen += 3;
    }else {
      hpCycleState = 4;
    }
  }else if(hpCycleState == 4){
    if(hpGreen >=10){
      hpGreen -= 3;
    }else {
      hpCycleState = 5;
    }
  }else if(hpCycleState == 5){
    if(hpBlue <= 250){
      hpBlue += 3;
    }else {
      hpCycleState = 6; 
    }
  }else if(hpCycleState == 6){
    if(hpBlue >=10){
      hpBlue -= 3;
    }else {
      hpCycleState = 7;
    }
  }else if(hpCycleState == 7 ){
    if(hpRed <= 250){
      hpRed+= 3;
    }else if(hpGreen <= 250){
      hpGreen+= 3;
    }else if(hpBlue <= 250){
      hpBlue+= 3;
    }else{
      hpCycleState=8;
    }
  }else if(hpCycleState == 8 ){
    if(hpRed >= 10){
      hpRed-= 3;
    }else if(hpGreen >= 10){
      hpGreen-= 3;
    }else if(hpBlue >= 10){
        hpBlue-= 3;
      }else{
        hpCycleState = 1;
      }
    }
    HP.setPixelColor(0, HP.Color(hpRed, hpGreen, hpBlue));
    HP.show();  
}

void rearLogicRandom() {
  if(millis() - randomMillisSingle > 300) {
    int random_i = random(0,3);
    sLOGIC.setPixelColor(random_i, sLOGIC.Color(random(0,255),random(0,255),random(0,255))); 
    sLOGIC.show();
    randomMillisSingle = millis();
  }
}


void rearLogicFade() {
  if(rearFadeState == 0){
    if(rearFadeRed < 24){
      rearFadeRed++;
      rearFadeGreen = map(rearFadeRed, 0, 23, 0, 67);
      rearFadeBlue = map(rearFadeRed, 0, 23, 0, 96);
    }else{
      rearFadeState = 1;
    }
  }else if(rearFadeState == 1){
    if(rearFadeRed > 0){
      rearFadeRed--;
      rearFadeGreen = map(rearFadeRed, 0, 23, 0, 67);
      rearFadeBlue = map(rearFadeRed, 0, 23, 0, 96);
    }else{
      rearFadeState = 0;
    }
  }
  sLOGIC.setPixelColor(0, sLOGIC.Color(rearFadeRed, rearFadeGreen, rearFadeBlue));
  sLOGIC.setPixelColor(1, sLOGIC.Color(rearFadeRed, rearFadeGreen, rearFadeBlue));
  sLOGIC.setPixelColor(2, sLOGIC.Color(rearFadeRed, rearFadeGreen, rearFadeBlue));
  sLOGIC.show();
 
}



void sendAndReceive(){
  if(millis() - lastSendRecMillis >= recDelay){
    PSILED(); 
    lastBodyReceive = millis();
  }
  lastSendRecMillis = millis(); 
  battLevel();
}
         
  
void battLevel(){
  if(millis() - lastBattUpdate >= sendDelay) {
    if(millis() - lastBodyReceive >= 3000) {
      sendBAT = 99.99;
    } else {
      sendBAT = sendBAT;
    }
    domeBatt1 = analogRead(battPin);
    domeBatt1 *= 2;    // we divided by 2, so multiply back
    domeBatt1 *= 3.3;  // Multiply by 3.3V, our reference voltage
    domeBatt1  /= 1024; // convert to voltage
    sendBAT = domeBatt1;
    lastBattUpdate = millis();     
  }
}

void debugRoutines() {
#ifdef debugBody
  SerialDebug.print(F("Body/PSI: ")); SerialDebug.print(incomingPSI); SerialDebug.print('\t'); 
  SerialDebug.print(F("Body/BTN: ")); SerialDebug.print(incomingBTN); SerialDebug.print('\t');
  SerialDebug.print(F("BTNstate: ")); SerialDebug.print(BTNstate); SerialDebug.print('\t');
  SerialDebug.print(F("Body/BAT: ")); SerialDebug.print(incomingBAT); SerialDebug.print('\t'); 
  SerialDebug.print(F("Dome/BAT: ")); SerialDebug.print(sendBAT); SerialDebug.print('\t'); 
  SerialDebug.print(F("incomingPSI: ")); SerialDebug.print(incomingPSI); SerialDebug.println('\t');  
#endif

#ifdef debugDome
  SerialDebug.print(F("Dome/BAT: ")); SerialDebug.print(sendBAT); SerialDebug.print('\t'); 
  SerialDebug.print(F("BTNstate: ")); SerialDebug.print(BTNstate); SerialDebug.print('\t');
  SerialDebug.print(F("Body/BAT: ")); SerialDebug.print(incomingBAT); SerialDebug.print('\t');
  SerialDebug.print(F("PSI: ")); SerialDebug.print(incomingPSI); SerialDebug.print('\t');
  SerialDebug.print(F("ESPNOW: ")); SerialDebug.print(success); SerialDebug.println('\t');  
#endif

#ifdef debugRecieveESPNOW
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("PSI: ");
  Serial.print(incomingReadings.psi);
  Serial.print(" ButtonStatus: ");
  Serial.print(incomingReadings.btn);
  Serial.print(" Body Battery Level: ");
  Serial.print(incomingReadings.bat);
  Serial.println();
#endif
}
