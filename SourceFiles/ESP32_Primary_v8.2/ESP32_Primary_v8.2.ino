/*
 * Joe's Drive  - V8.2 5/2024
 * Primary CPU uses ESP32 HUZZAH32
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need ESP32 Hardware: 
 * ESP32 HUZZAH From Adafruit: https://www.adafruit.com/product/3619 used for Dome and Primary Feather
 * ESp32 Board Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
 * 
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * On Line 148 - replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 * Modified ESP32 BT Host Library https://github.com/reeltwo/PSController
 * 
 * Libraries Required
 * https://github.com/netlabtoolkit/VarSpeedServo - Servo Controls for Feather (Non ESP32)
 * https://github.com/PaulStoffregen/Encoder - Encoder for the Dome Spin motor
 * https://github.com/reeltwo/PSController - Modified USBhost Controller for PS3 Move Nav/Xbox Controllers
 * https://github.com/ERROPiX/ESP32_AnalogWrite - Used for Proper PWM with ESP32 devices
 * https://github.com/br3ttb/Arduino-PID-Library/ - PID Library Official
 * http://www.billporter.info/easytransfer-arduino-library/ - Easy Transfer
 * https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences - Equivilent to the EEPROM Controls
 * https://www.andymark.com/products/neverest-classic-60-gearmotor?Power%20Connector=Anderson%20Powerpole%2015A%20(am-3103)&quantity=1> - Dome Rotation with Encoder
 *
*/



#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)
#define SerialDebug Serial  // Where you wish to send the output for debug (Define which SerialX)

/*
 * Configuration Parameters
 * Comment and uncomment which function needs to be configured
*/

// #define IMU_Bypass              //used for disabling and not using the IMU - good for testing just S2S without IMU
#define enableESPNOW            // uncomment to use ESPNOW to communicate over wifi to the dome using ESP32
#define revS2S                  // uncomment to reverse the POT and S2S variables essentially reversing the current position tracking
// #define revDrive
// #define revGyro

#define driveDelay .75
#define maxS2STilt 30           // controls how far the S2S control using a joystick will it tilt... currently 25 should be max.
#define S2SEase 2.5 

/*
 * Debug Configurations
 * Comment and uncomment which function needs to be debugged
*/

// #define ESPNOWCONFIG
// #define debugESPNOW
// #define debugESPNOWSend
// #define debugESPNOWReceive
// #define debugPreferences
// #define debug32u4
// #define debugNavRemoteRight
// #define debugNavRemoteLeft
// #define debugIMU
// #define debugPOTS
// #define debugMainDrive
// #define debugFlywheel
// #define debugS2S
// #define debugS2SPot
// #define debugSounds
// #define debugEasyTransfer

/*
* Controller types - Supporting Playstation MOVE and Microsoft XBOX
*/

#define MOVECONTROLLER
//#define XBOXCONTROLLER

/*
  MAC ADDRESS DEFINITIONS
  Signifies the primary MAC of the ESP32/USB BT chip
  Bluetooth address of this ESP32 device. If you already have a Shadow system configured
  the easiest thing is reuse the address of your USB Bluetooth dongle here. Alternatively,
  you can use sixaxispair to pair your controllers with the ESP32.
  https://www.adafruit.com/product/3619

*/

#define MasterNav "7c:9e:bd:d7:63:c6" // This sets the mac address of the ESP32 to allow your Nav Controller to attach to it using SixAxisPairTool


/*
  PIN DEFINITIONS
*/

#define NeoPixel_pin 27
#define S2S_pwm 33
#define S2S_pin_1 26
#define S2S_pin_2 25
#define Drive_pwm 21
#define Drive_pin_1 4
#define Drive_pin_2 27
#define S2SPot_pin A2 //GPIO 34
#define flyWheelMotor_pwm 15
#define flyWheelMotor_pin_A 32
#define flyWheelMotor_pin_B 14
#define voltageSensor_Pin A3 //GPIO 39


/*
 * LIBRARY DEFINITIONS
*/

/*
 Modified ESP32 BT Host Library https://github.com/reeltwo/PSController
*/

#ifdef XBOXCONTROLLER
  #include <XBOXRECV.h>
#endif

#ifdef MOVECONTROLLER
  #include <PSController.h>
  #define DRIVE_CONTROLLER_MAC  nullptr
  #define DOME_CONTROLLER_MAC nullptr
#endif

#include <Arduino.h>
#include <EasyTransfer.h> // http://www.billporter.info/easytransfer-arduino-library/
//#include <Wire.h>  // Used with i2c communications
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/
#include "wiring_private.h" // pinPeripheral() function
#include <analogWrite.h>  // https://www.arduinolibraries.info/libraries/esp32-analog-write
#include <Encoder.h> // Include the necessary libraries for the encoder


/*
 Needed Libraries for ESPNOW to connect to Dome Controller
*/
#ifdef enableESPNOW
  #include <stdint.h>
  #include <esp_now.h>
  #include "WiFi.h"
#endif

/*
 * *********** IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE MAC ADDRESS ************
 * Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
 * As per the following walkthrough https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 * broadcastAddress REPLACE WITH THE MAC Address of your receiver - the other ESP32 in the body of BB8 7C:9E:BD:D7:63:C4
*/
uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4}; // DOME ESP32
// uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xD7, 0x63, 0xC4};

#ifdef MOVECONTROLLER
PSController driveController(DRIVE_CONTROLLER_MAC); //define the drive an dome Controller variable to be used against the Nav1 and Nav2 controllers.
PSController domeController(DOME_CONTROLLER_MAC);
#define driveDelay .75
#endif


/*
 * create UART object ************************
*/
EasyTransfer recIMU; 
EasyTransfer send32u4;
EasyTransfer rec32u4; 

/*
 * Setup EEPROM Saving of preferences for ESP32
 */
#include <Preferences.h>
Preferences preferences;
#include <nvs_flash.h> // for wiping NVram

/*
 * Create receive IMU from Trinket M0 Objects
*/
struct RECEIVE_DATA_STRUCTURE_IMU{
  float pitch;
  float roll; 
}receiveIMUData;

/*
 * Create Send to the 32u4 Feather Objects
*/
#ifdef XBOXCONTROLLER
struct SEND_DATA_STRUCTURE_32u4{  
  bit driveEnabled;
  int8_t domeSpin; // X only of 
  bool xboxL3;
  int8_t flywheel;
  bool xboxR3;
  int8_t leftStickX;
  int8_t leftStickY;
  int8_t rightStickX;
  int8_t rightStickY;
  bool psiFlash;
  float pitch;
  float roll;
} sendTo32u4Data;
#endif

#ifdef MOVECONTROLLER
struct SEND_DATA_STRUCTURE_32u4{  
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3; // reverseDrive
  bool moveR3; // domeServoMode
  int8_t leftStickX;
  int8_t leftStickY;
  int8_t soundcmd;
  int8_t psiFlash;
  float pitch;
  float roll; 
} sendTo32u4Data;
#endif

/*
 * Create Receive from 32u4 Objects
*/

struct RECEIVE_DATA_STRUCTURE_32u4{ // - SLAVE_DATA
  int16_t tiltAngle; 
  bool sndplaying;
} receiveFrom32u4Data;


#ifdef XBOXCONTROLLER
struct controllerButtons{
  bool a,b,x,y,up,down,left,right,xbox,back,l1,r1,start;
  int8_t rightStickX,rightStickY,l2,r2;
  int8_t leftStickX,leftStickY,l2;
} buttons;
#endif

#ifdef MOVECONTROLLER
struct controllerButtonsL{
  bool cross,circle,up,down,left,right,ps,l1,l3;
  int8_t leftStickX,leftStickY,l2;
} buttonsL;
struct controllerButtonsR{
  bool cross,circle,up,down,left,right,ps,l1,l3;
  // int8_t rightStickX,rightStickY,l2;
  int rightStickX,rightStickY,l2; // Mov controller causes false positive negative reverse when pushing 100% forward need int
} buttonsR;
int8_t joystickDeadZoneRange = 25;  // For controllers that centering problems, use the lowest number with no drift
#endif

/*
 * Define variables to store readings to be sent
*/
int sendPSI = 0;
byte sendHP = 0;
float sendBAT = 0;    
int sendDIS = 0;

/* 
 *  Define variables to store incoming readings
*/
int incomingPSI = 0;
byte incomingBTN = 0;
float incomingBAT = 0;      

/*
 * Variable to store if sending data was successful
*/
String success;

/*
 * Create Send Data for ESPNOW
 * must match the receiver structure
*/
typedef struct struct_message {
    int psi;
    byte btn;
    float bat;
    int dis;
} struct_message;

// Create a struct_message called outgoingESPNOW to hold sensor readings
struct_message outgoingESPNOW;

// Create a struct_message to hold incoming sensor readings
struct_message incomingESPNOW;

/*
 * Create Serial 2 to send to the 32u4 / ESP32-S2 (qwiic)
*/
#define SERIAL2_BAUD_RATE 74880  // 74880 57600 115200
#define SERIAL2_RX_PIN 13
#define SERIAL2_TX_PIN 12

/*
 * Create Serial 3 to send to debug or free pins as needed
*/
#include <SoftwareSerial.h>
SoftwareSerial Serial3;
#define SERIAL3_BAUD_RATE 115200
#define SERIAL3_RX_PIN 22 //SCL - General purpose IO pin #22
#define SERIAL3_TX_PIN 23 //SDA - General purpose IO pin #23

unsigned long currentMillis, IMUmillis, lastLoopMillis, lastDomeMillis, rec32u4Millis, lastPrintMillis; 
bool EnableFlywheel, IMUconnected, controllerConnected, Send_Rec, feather2Connected, drivecontrollerConnected, domecontrollerConnected, DomeServoMode, enableDrive, reverseDrive, enabledDrive;
bool autoDisableDoubleCheck, autoDisable, autoDisableState;
unsigned long autoDisableMotorsMillis, autoDisableDoubleCheckMillis;

int current_pos_drive;  // variables for smoothing main drive
int target_pos_drive;

float Drive_IMUDeadzone = 2;   // target fluctuating between for IMU
float S2S_potDeadzone = 5; // 3

int diff_drive; // difference of position  (diff_drive = target_pos_drive - current_pos_drive;  // Work out the required travel.)
double easing_drive;
float IMUFiltered, pitchOffset, rollOffset;
double S2S_pot, potOffsetS2S;
int current_pos_S2S;  // variables for smoothing S2S
int target_pos_S2S;
int diff_S2S; // difference of position
double easing_S2S;

int Output_flywheel_pwm; // variables for PWM controls of flywheel
int Output_domeSpin_pwm;
int flywheelRotation;
double flywheel;
double flywheelEase;
// double Setpoint_Drive, Input_Drive, Output_Drive, Setpoint_Drive_In;

double Pk1 = 14;  //13 14
double Ik1 = 0;   //0
double Dk1 = 0.1;   //0.1
double Setpoint1, Input1, Output1, Output1_S2S_pwm;    // PID variables - S2S Joystick control
PID PID1_S2S(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - S2S Drive

double Pk2 = 1.5; // 10 and then .5
double Ik2 = 0; //0
double Dk2 = .01; //0 .03
double Setpoint2, Input2, Output2, Output2_S2S_pwm;    // PID variables - S2S stability
PID PID2_S2S(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - S2S stability

double Pk3 = 20; // 13 5
double Ik3 = 0;
double Dk3 = 0.0; // 0
double Setpoint3, Input3, Output3, Setpoint_Drive_In, Output_Drive_pwm;    // PID variables - Main drive motor
PID PID_Drive(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // Main drive motor

void setup() {
  
  delay(5000); // Wait for Boot and for serial to become ready
  currentMillis = millis();
  Serial.begin(115200);
  Serial1.begin(115200); //115200 74880
  Serial2.begin(SERIAL2_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
//  Serial3.begin(SERIAL3_BAUD_RATE, SWSERIAL_8N1, SERIAL3_RX_PIN, SERIAL3_TX_PIN,false,256);

  preferences.begin("JoeDriveV2", false); // open preferences channel
  
#ifdef MOVECONTROLLER
  PSController::startListening(MasterNav);
  String address = PSController::getDeviceAddress();
  Serial.print("Please write down the following MAC and Assign to your Nav Controller(s): ");
  Serial.println(address);
  Serial.println("Bluetooth Ready.");
#endif

  recIMU.begin(details(receiveIMUData), &Serial1); 
  rec32u4.begin(details(receiveFrom32u4Data), &Serial2);
  send32u4.begin(details(sendTo32u4Data), &Serial2);

#ifdef ESPNOWCONFIG
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("Please write down the following WIFI MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("WIFI ESPNOW Ready.");
#endif

  PID1_S2S.SetMode(AUTOMATIC);              // PID Setup - S2S SERVO
  PID1_S2S.SetOutputLimits(-255, 255);
  PID1_S2S.SetSampleTime(20);

  PID2_S2S.SetMode(AUTOMATIC);              // PID Setup - S2S Stability
  PID2_S2S.SetOutputLimits(-255, 255);
  PID2_S2S.SetSampleTime(20);

  PID_Drive.SetMode(AUTOMATIC);              // PID Setup - main drive motor
  PID_Drive.SetOutputLimits(-255, 255);
  PID_Drive.SetSampleTime(20);
    
// Lets do some spot checks on the connections to other chips/CPUs
  #ifdef debug32u4
   if(!rec32u4.receiveData()){
        feather2Connected = false; 
        Serial.println("32u4 Not Connected");
   } else {
      if(feather2Connected == false){
        rec32u4Millis = currentMillis;
        feather2Connected = true;
        Serial.println("32u4 Connected");
      }
    }
  #endif
  
  #ifndef IMU_Bypass
    for (int i = 0; i <= 10000; i++) {
      IMUmillis = currentMillis; 
      IMUconnected = false; 
      Serial.println("IMU Not Connected");
      if (recIMU.receiveData()){
        IMUmillis = currentMillis;
        IMUconnected = true;
        Serial.println("IMU Connected");
        break;
      }
    }
  #endif

  /* Set the pins to correct method for use for the DFRobot Motor Driver */
  pinMode(S2S_pwm, OUTPUT);  // Speed Of Motor2 on Motor Driver 1 
  pinMode(S2S_pin_1, OUTPUT);  // Direction
  pinMode(S2S_pin_2, OUTPUT);
  pinMode(Drive_pwm, OUTPUT);  // Speed of Motor2 on Motor Driver 2 
  pinMode(Drive_pin_1, OUTPUT);  // Direction
  pinMode(Drive_pin_2, OUTPUT);
  pinMode(flyWheelMotor_pwm, OUTPUT);  // Speed of Motor2 on Motor Driver 2 
  pinMode(flyWheelMotor_pin_A, OUTPUT);  // Direction
  pinMode(flyWheelMotor_pin_B, OUTPUT);

 #ifdef enableESPNOW
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) { // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  } else Serial.println("Finished initializing ESP-NOW");

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

#endif

// *********  Read offsets from EEPROM  **********
    
  pitchOffset = preferences.getFloat("pitchOffset", 0);
  rollOffset = preferences.getFloat("rollOffset", 0);
  potOffsetS2S = preferences.getInt("potOffsetS2S", 0);

// commented the below out because when its turned off mid S2S tilt or while tilting forward and turned back on
// it will save those settings while booting up... NOT GOOD
  // if (abs(rollOffset) + abs(pitchOffset) + abs(potOffsetS2S) == 0 ){ // If the offsets are empty, soft set them dont store them until asked.
  //   setOffsetsONLY();
  // }
}


void loop() {
  currentMillis = millis(); 
  if(currentMillis - lastLoopMillis >= 10) {
    lastLoopMillis = currentMillis; 
      #ifndef IMU_Bypass
        receiveIMU();
      #endif
  }
  receiveRemote();
  S2S_Movement(); 
  drive_Movement(); 
  spinFlywheel();
  #ifdef enableESPNOW
    sendESPNOW();
  #endif
  // autoDisableMotors();
  sendDataTo32u4();
  if(currentMillis - lastPrintMillis >= 70) {
    lastPrintMillis = currentMillis;
    debugRoutines();
  }

}

void receiveIMU(){
  recIMU.receiveData();
  sendTo32u4Data.roll = receiveIMUData.roll + rollOffset;
  sendTo32u4Data.pitch = receiveIMUData.pitch + pitchOffset;
}


void sendDataTo32u4(){
  send32u4.sendData(); 
  if(rec32u4.receiveData()){
    rec32u4Millis = currentMillis;
    if(feather2Connected == false){
      feather2Connected = true;
      #ifdef debug32u4
        DEBUG_PRINTLN("32u4 Connected");
      #endif
    } else {
        feather2Connected = false; 
        #ifdef debug32u4
          DEBUG_PRINTLN("32u4 Not Connected");
        #endif
    }
   } else if((currentMillis - rec32u4Millis) > 100){
      feather2Connected = false; 
   }       
}

// void sendDataTo32u4(){
//   send32u4.sendData(); 
//   if(rec32u4.receiveData()){
//     rec32u4Millis = currentMillis;
//     if(feather2Connected == false){
//       feather2Connected = true;
//       #ifdef debug32u4
//         DEBUG_PRINTLN("32u4 Connected");
//       #endif
//     } else {
//         feather2Connected = false; 
//         #ifdef debug32u4
//           DEBUG_PRINTLN("32u4 Not Connected");
//         #endif
//     }
//    } else if((currentMillis - rec32u4Millis) > 100){
//       feather2Connected = false; 
//    }       
// }

/* 
 *  ESPNOW Callback when data is sent
*/
#ifdef enableESPNOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef debugESPNOWSend
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

/* 
 *  ESPNOW Callback when data is received
*/
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingESPNOW, incomingData, sizeof(incomingESPNOW));
  #ifdef debugESPNOWReceive
    Serial.print("Bytes received: ");
    Serial.println(len);
  #endif
  incomingPSI = incomingESPNOW.psi;
  incomingBTN = incomingESPNOW.btn;
  incomingBAT = incomingESPNOW.bat;
}

/* 
 *  ESPNOW Callback when data is sent
*/

void sendESPNOW() {
    // if (receiveFrom32u4Data.sndplaying == 1) {
    //   sendPSI = 1;
    // } else {
    //   sendPSI = 0;
    // }
    outgoingESPNOW.psi = sendPSI;
    outgoingESPNOW.btn = sendHP;
    outgoingESPNOW.bat = sendBAT;
    outgoingESPNOW.dis = sendDIS;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingESPNOW, sizeof(outgoingESPNOW));
  #ifdef debugESPNOWSend
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  #endif
}
#endif
//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}

//void readBatteryVoltage() {    // read the value at analog input
//  batt_Voltage = (((analogRead(voltageSensor_Pin)) * 3.28) / 1024.0) / (R2/(R1+R2));
//  #ifdef debugVS
//    Serial.print(F("Battery Voltage: ")); Serial.println(batt_Voltage); 
//  #endif
//}



