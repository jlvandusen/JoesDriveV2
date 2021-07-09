/*
 * Joe's Drive  - V2 7/7/2021
 * Primary ESP32 HUZZAH32
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need libraries: 
 * HUZZAH32 From Adafruit: https://www.adafruit.com/product/3619
 * ESp32 Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
 * Adafruit VS1053 Featherwing MusicPlayer: https://github.com/adafruit/Adafruit_VS1053_Library
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
*/

/*
 * Debug Configurations
 * Comment and uncomment which function needs to be debugged
*/

#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)
//#define debugDome // Validate and feedback what is being received from Dome (PSI, HP and Bat)
//#define debugDomeSpin
//#define debugFlywheel
//#define debugRemote
//#define debugIMU
//#define debugMainDrive
//#define debugS2S
#define debugSounds
//#define debugESPNOW // Configure the system to display its wifi mac and bluetooth addresses for insert into Dome code.
//#define debugdebugESPNOWConfig  // Configure the system to display its wifi mac and bluetooth addresses for insert into Body code.


/*
 * Controller Configurations
 * Comment and uncomment which controller you wish to use (only 1 type can be used at any given time)
*/

#define MOVECONTROLLER
//#define XBOXCONTROLLER

/*
 * Music Controller
 * Used to enable or disable use of VS1053 Musicplayer Featherwing
*/

#define MUSICPLAYER_FEATHERWING

/*
 * Communication Functions of ESP32
 * Used to enable or disable use of communication methods to ESP32, can you multiple
*/

//#define WIFIACCESSPOINT  // BB8 Main Controls are exposed over a Wifi AccessPoint with its own network
//#define BTSerialMode // Allow Main Controls exposed via BlueTooth alongside Controller support
#define ESPNOW // Allow ESP communications over Wifi to Dome ESP32


/*
 * MAC ADDRESS DEFINITIONS
 * Signifies the primary MAC of the ESP32/USB BT chip
 * Bluetooth address of this ESP32 device. If you already have a Shadow system configured
 * the easiest thing is reuse the address of your USB Bluetooth dongle here. Alternatively,
 * you can use sixaxispair to pair your controllers with the ESP32.
 * https://www.adafruit.com/product/3619
*/
#define MasterNav "7c:9e:bd:d7:63:c6" 
   
/*
 * PIN DEFINITIONS
*/

#ifdef MOVECONTROLLER //If using a Move, PS3,4 or 5 controller joined to the ESP32
#define NeoPixel_pin 27
#define S2S_pwm 22  // SCL 22 and SDA 23
#define S2S_pin_1 26 //A0
#define S2S_pin_2 25 //A1
#define Drive_pwm 23  // SCL 22 and SDA 23
#define Drive_pin_1 4
#define Drive_pin_2 21
#define S2SPot_pin A2 //GPIO 34
#define S2S_offset -120
#endif

/*
 * Creates a WiFi access point and provides a web server on it.
 * Steps:
 *    1. Connect to the access point "bb8v2"
 *    2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
 * OR
 * 
 * Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port
 * Created for arduino-esp32 on 04 July, 2018 by Elochukwu Ifediora (fedy0)
*/

#ifdef WIFIACCESSPOINT
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define LED_BUILTIN 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED

// Set these to your desired credentials.
const char *ssid = "bb8v2";
const char *password = "greatcheese061";
WiFiServer server(80);
#endif

#ifdef BTSerialMode
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
  //String MACadd = "AA:BB:CC:11:22:33";
  //uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
  String name = "DESKTOP-9IKITLF";
  const char *pin = "1234"; //<- standard pin would be provided by default
  bool connected;
#endif

#ifdef ESPNOW
// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
// As per the following walkthrough https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
#include <esp_now.h>
#include "WiFi.h"
// broadcastAddress REPLACE WITH THE MAC Address of your receiver - the other ESP32 in BB8 Dome (loopback {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};)
uint8_t broadcastAddress[] = {0xF1, 0xFF, 0xF1, 0xFF, 0xFF, 0xFF};

// Define variables to store incoming readings
int incomingPSI = 0;
byte incomingHP = 0;
float incomingBAT = 0;      

// Define variables to store readings to be sent
int sendPSI = 0;
byte sendBTN = 0;
float sendBAT = 0;    

// Variable to store if sending data was successful
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

#endif


/*
 * ESP32 FEATHER INFORMATION
 * Chip used: Adafruit ESP32 HUZZAH32 with stacking headers https://www.adafruit.com/product/3619
A0 - this is an analog input A0 and also an analog output DAC2. It can also be used as a GPIO #26. It uses ADC #2
A1 - this is an analog input A1 and also an analog output DAC1. It can also be used as a GPIO #25. It uses ADC #2
A2 - this is an analog input A2 and also GPI #34. Note it is not an output-capable pin! It uses ADC #1
A3 - this is an analog input A3 and also GPI #39. Note it is not an output-capable pin! It uses ADC #1
A4 - this is an analog input A4 and also GPI #36. Note it is not an output-capable pin! It uses ADC #1
A5 - this is an analog input A5 and also GPIO #4. It uses ADC #2
21 - General purpose IO pin #21
SCL - General purpose IO pin #22
SDA - General purpose IO pin #23
 */


/*
 * LIBRARY DEFINITIONS
*/
//#ifdef XBOXCONTROLLER
//#include <XBOXRECV.h>
//#endif

#ifdef MOVECONTROLLER
#include <analogWrite.h>
#include <PSController.h>
#define DRIVE_CONTROLLER_MAC  nullptr
#define DOME_CONTROLLER_MAC nullptr
#endif

#include <Arduino.h>
#include <EasyTransfer.h>
#include <Wire.h>
#include <SPI.h>

/*
 * FORMATTING and preparing the SD card with Sound files
 * Press WIN+R key combination to open Run dialogue, input “cmd” and press “Enter” to open the Command Prompt window. 
 * Then execute the following commands in turn:
 * diskpart
 * list disk
 * select disk m, where m is the SD card number.
 * clean
 * create partition primary
 * format fs=fat32 quick
 * 
 */
#ifdef MUSICPLAYER_FEATHERWING
  #include <SD.h>
  #include <Adafruit_VS1053.h>
#endif

#include <PID_v1.h>
//#include "wiring_private.h" // pinPeripheral() function

#define BUFFER_LENGTH 64
#define TWI_BUFFER_LENGTH 64
#define driveDelay .75

//#ifdef XBOXCONTROLLER
//USBHost UsbH;
//XBOXRECV Xbox(&UsbH);
//#endif

#ifdef MOVECONTROLLER
PSController driveController(DRIVE_CONTROLLER_MAC); //define the driveController variable to be used against the Nav1 and Nav2 controllers.
PSController domeController(DOME_CONTROLLER_MAC);
#endif

/*
 * Create UART Functions
*/
EasyTransfer recIMU; 
EasyTransfer send32u4;
EasyTransfer rec32u4; 

/*
 * Create receive IMU from Trinket M0 Objects
*/
struct RECEIVE_DATA_STRUCTURE_IMU {
  float pitch;
  float roll; 
};
RECEIVE_DATA_STRUCTURE_IMU receiveIMUData;


/*
 * Create Send to the 32u4 Feather Objects
*/
struct SEND_DATA_STRUCTURE_32u4{  
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3; // xbox L3 equivilent
  int8_t flywheel;
  bool moveps; // xboxR3 equivilent
  int8_t leftStickX;
  int8_t leftStickY;
//  int8_t rightStickX;
//  int8_t rightStickY;
  bool psiFlash;
  float pitch;
  float roll; 
};
SEND_DATA_STRUCTURE_32u4 sendTo32u4Data;

/*
 * Create Receive from 32u4 Objects
*/
struct RECEIVE_DATA_STRUCTURE_32u4 {
  int16_t tiltAngle; 
};
RECEIVE_DATA_STRUCTURE_32u4 receiveFrom32u4Data;

//#ifdef XBOXCONTROLLER
//struct controllerButtons{
//  bool a,b,x,y,up,down,left,right,xbox,back,l1,r1,start;
//  int8_t rightStickX,rightStickY,l2,r2;
//};
//controllerButtons buttons;
//#endif

#ifdef MOVECONTROLLER
struct controllerButtonsL {
  bool cross,circle,up,down,left,right,ps,l1,l3;
  int8_t leftStickX,leftStickY,l2;
};
struct controllerButtonsR {
  bool cross,circle,up,down,left,right,ps,l1,l3;
  int8_t rightStickX,rightStickY,l2;
};
controllerButtonsL buttonsL;
controllerButtonsR buttonsR;
int8_t joystickDeadZoneRange = 25;  // For controllers that centering problems, use the lowest number with no drift
#endif

#ifdef MUSICPLAYER_FEATHERWING
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)
#define VS1053_CS      32     // VS1053 chip select pin (output)
#define VS1053_DCS     33     // VS1053 Data/command select pin (output)
#define CARDCS         14     // Card chip select pin
// DREQ should be an Int pin *if possible* (not possible on 32u4)
#define VS1053_DREQ    15     // VS1053 Data request, ideally an Interrupt pin
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);
#endif

#define SERIAL2_BAUD_RATE 74880
#define SERIAL2_RX_PIN 27
#define SERIAL2_TX_PIN 12

/*
 * Do we need an additional Serial to perform debugging?
*/
#ifdef debugSerial
  #include <SoftwareSerial.h>
  #define SERIAL3_BAUD_RATE 74880
  #define SERIAL3_RX_PIN 21
  #define SERIAL3_TX_PIN 20
  SoftwareSerial Serial3;
#endif



#define SerialDebug Serial

bool enableDrive,reverseDrive; 

unsigned long currentMillis, IMUmillis, lastLoopMillis, lastDomeMillis, rec32u4Millis, lastPrintMillis; 
bool IMUconnected, controllerConnected, Send_Rec, feather2Connected, drivecontrollerConnected, domecontrollerConnected; 
//Define Variables we'll be connecting to
double Setpoint_S2S_Servo, Input_S2S_Servo, Output_S2S_Servo;
double Setpoint_S2S_Stabilization, Input_S2S_Stabilization, Output_S2S_Stabilization;
double Setpoint_Drive, Input_Drive, Output_Drive, Setpoint_Drive_In;

#ifdef MUSICPLAYER_FEATHERWING
int numberOfTracks = 55; 
int i; 
char *soundNames[]={"ACK0000.ogg", "FUN0000.ogg",  "FUN0001.ogg",  "BEEP0000.ogg",  "ACK0001.ogg",  "BEEP0001.ogg",  "BEEP0002.ogg",  "BEEP0003.ogg",  "BEEP0004.ogg",  "BB80010.ogg",  
"BB80011.ogg",  "BB80012.ogg",  "BB80013.ogg",  "BB80014.ogg",  "BB80015.ogg",  "BB80016.ogg",  "BB80017.ogg",  "BB80018.ogg",  "BB80019.ogg",  "BB80021.ogg",  
"BB80022.ogg",  "BB80023.ogg",  "BB80024.ogg",  "BB80025.ogg",  "BB80026.ogg",  "BB80027.ogg",  "BB80028.ogg",  "BB80029.ogg",  "BB80030.ogg",  "BB80031.ogg",  "BB80032.ogg",  
"BB80033.ogg",  "BB80034.ogg",  "BB80035.ogg",  "BB80036.ogg",  "BB80037.ogg",  "BB80038.ogg",  "BB80039.ogg",  "BB80040.ogg",  "BB80041.ogg",  "BB80042.ogg",  "BB80043.ogg",  
"BB80044.ogg",  "BB80045.ogg",  "BB80046.ogg",  "BB80047.ogg",  "BB80048.ogg",  "BB80049.ogg",  "BB80050.ogg",  "BB80051.ogg",  "BB80052.ogg",  "BB80053.ogg",  "BB80054.ogg",  
"BB80055.ogg",  "BB80056.ogg"};
#endif

  //Specify the links and initial tuning parameters
double Kp_S2S_Servo=.5, Ki_S2S_Servo=0, Kd_S2S_Servo=0;
PID myPID_S2S_Servo(&Input_S2S_Servo, &Output_S2S_Servo, &Setpoint_S2S_Servo, Kp_S2S_Servo, Ki_S2S_Servo, Kd_S2S_Servo, DIRECT);

  //Specify the links and initial tuning parameters
double Kp_S2S_Stabilization=10, Ki_S2S_Stabilization=0, Kd_S2S_Stabilization=0;
PID myPID_S2S_Stabilization(&Input_S2S_Stabilization, &Output_S2S_Stabilization, &Setpoint_S2S_Stabilization, Kp_S2S_Stabilization, Ki_S2S_Stabilization, Kd_S2S_Stabilization, DIRECT);

  //Specify the links and initial tuning parameters
double Kp_Drive=4, Ki_Drive=0, Kd_Drive=0;
PID myPID_Drive(&Input_Drive, &Output_Drive, &Setpoint_Drive, Kp_Drive, Ki_Drive, Kd_Drive, DIRECT);

bool enabledDrive;
  
void setup() {
  currentMillis = millis();
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(SERIAL2_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
//  Serial3.begin(SERIAL3_BAUD_RATE, SWSERIAL_8N1, SERIAL3_RX_PIN, SERIAL3_TX_PIN,false,256);
  delay(1000);

#ifdef WIFIACCESSPOINT
  Serial.println();
  Serial.println("Configuring access point...");
  WiFi.softAP(ssid, password);   // You can remove the password parameter if you want the AP to be open.
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
#endif

#ifdef BTSerialMode
// connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
// to resolve name to address first, but it allows to connect to different devices with the same name.
// Set CoreDebugLevel to Info to view devices bluetooth address and device names
//  SerialBT.begin("BB8"); //Bluetooth device name
  SerialBT.begin("BB8", true);  
  Serial.println("The device started in master mode, make sure remote BT device is on!");

  connected = SerialBT.connect(name);
//connected = SerialBT.connect(address);
  if (connected) {
    while(!connected) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
    Serial.println("Connected Succesfully!");
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
#endif

#ifdef debugESPNOWConfig
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("Please write down the following WIFI MAC: ");
  Serial.println(WiFi.macAddress());
#endif

#ifdef ESPNOW
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station
  Serial.println("WIFI ESPNOW preparing...");
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else Serial.println("Success initializing ESP-NOW");

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
  } else Serial.println("Found dome ESP32 peer and added over ESPNOW");
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
#endif

#ifdef MOVECONTROLLER
  PSController::startListening(MasterNav);
  String address = PSController::getDeviceAddress();
  Serial.print("Please write down the following MAC and Assign to your Nav Controller(s): ");
  Serial.println(address);
  Serial.println("Bluetooth Ready.");
#endif
  
#ifdef MUSICPLAYER_FEATHERWING
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
  } else {
      Serial.println(F("VS1053 found"));
      musicPlayer.setVolume(1,1);
      musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
   }
  
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
  } else Serial.println("SD OK!");
#endif

  myPID_S2S_Servo.SetMode(AUTOMATIC);
  myPID_S2S_Servo.SetOutputLimits(-255, 255);
  
  myPID_S2S_Stabilization.SetMode(AUTOMATIC);
  myPID_S2S_Stabilization.SetOutputLimits(-255, 255);

  myPID_Drive.SetMode(AUTOMATIC);
  myPID_Drive.SetOutputLimits(-255, 255);

  recIMU.begin(details(receiveIMUData), &Serial1); 
  rec32u4.begin(details(receiveFrom32u4Data), &Serial2);
  send32u4.begin(details(sendTo32u4Data), &Serial2);

// Lets do some spot checks on the connections to other chips/CPUs
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
  if(!recIMU.receiveData()){
    IMUmillis = currentMillis; 
    IMUconnected = false; 
    Serial.println("IMU Not Connected");
  } else {
    if(IMUconnected == false){
      IMUmillis = currentMillis;
      IMUconnected = true;
      Serial.println("IMU Connected");
    }
    if((currentMillis - IMUmillis) > 25) {
      IMUconnected = false; 
      Serial.println("IMU Not Connected");
    }
  }

  /* Set the pins to correct method for use for the DFRobot Motor Driver */
  pinMode(S2S_pwm, OUTPUT);  // Speed Of Motor2 on Motor Driver 1 
  pinMode(S2S_pin_1, OUTPUT);  // Direction
  pinMode(S2S_pin_2, OUTPUT);
  pinMode(Drive_pwm, OUTPUT);  // Speed of Motor2 on Motor Driver 2 
  pinMode(Drive_pin_1, OUTPUT);  // Direction
  pinMode(Drive_pin_2, OUTPUT);

}

void loop() {
  currentMillis = millis(); 
  receiveIMU();
  if(currentMillis - lastLoopMillis >= 10) {
    lastLoopMillis = currentMillis; 
    wificlient();
    sendReadings();
    receiveRemote();
    S2S_Movement(); 
    drive_Movement(); 
    sendDataTo32u4();
    sounds(); 
    if(currentMillis - lastPrintMillis >= 70) {
      lastPrintMillis = currentMillis;
      debugRoutines();
    }
  }
}

/*  
 *  ESPNOW Send readings back to the Body ESP32
 *  See walkthrough: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 */
 
void sendReadings() {
  outgoingReadings.psi = sendPSI;
  outgoingReadings.btn = sendBTN;
  outgoingReadings.bat = sendBAT;
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
  #ifdef debugESPNOW
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else Serial.println("Error sending the data");
  #endif
}


/*  
 *  ESPNOW Callback when data is received
 *  See walkthrough: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  #ifdef debugESPNOW
  Serial.print("Bytes received: ");
  Serial.println(len);
  #endif
  incomingPSI = incomingReadings.psi;
  incomingHP  = incomingReadings.btn;
  incomingBAT = incomingReadings.bat;
}


/* 
 *  ESPNOW Callback when data is sent
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef debugESPNOW
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

void wificlient() {
  #ifdef WIFIACCESSPOINT
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    Serial.println("New Client.");          // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    if (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to turn ON the LED.<br>");
            client.print("Click <a href=\"/L\">here</a> to turn OFF the LED.<br>");

            // The HTTP response ends with another blank line:
            client.println();
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
        }
      }
    } else {
        // close the connection:
        client.stop();
        Serial.println("Client Disconnected.");
      }
  }
  #endif
}

void receiveRemote() {
  #ifdef MOVECONTROLLER
  if (driveController.isConnected()) {
    if (!drivecontrollerConnected){ // notify us of the connection as long as the status is false
      Serial.println("We have our Drive Nav Controller");
      batterycheck();
    }
    drivecontrollerConnected = true; // Set the status to true
  }  else drivecontrollerConnected = false; // Set the status back to false
  if (domeController.isConnected()) {
    if (!domecontrollerConnected){ // notify us of the connection as long as the status is false
      Serial.println("We have our Dome Nav Controller");
      batterycheck();
    }
    domecontrollerConnected = true; // Set the status to true
  }
  else domecontrollerConnected = false; // Set the status to false
  if ((drivecontrollerConnected) || (domecontrollerConnected) ) {
    controllerConnected = true;
    controllerButtonsR previousStateR = buttonsR;
    controllerButtonsL previousStateL = buttonsL;
    buttonsR.l1 = driveController.state.button.l1;
    buttonsR.l2 = constrain(map(driveController.state.analog.button.l2,0,255,0,100),0,100);
    buttonsR.l3 = driveController.state.button.l3;
    buttonsL.l1 = domeController.state.button.l1;
    buttonsL.l2 = constrain(map(domeController.state.analog.button.l2,0,255,0,100),0,100);
    buttonsL.l3 = domeController.state.button.l3;
    buttonsL.circle = domeController.state.button.circle;
    buttonsL.cross = domeController.state.button.cross;
    buttonsL.up = domeController.state.button.up;
    buttonsL.down = domeController.state.button.down;
    buttonsL.left = domeController.state.button.left;
    buttonsL.right = domeController.state.button.right;
    buttonsL.ps = domeController.state.button.ps;
    buttonsR.circle = driveController.state.button.circle;
    buttonsR.cross = driveController.state.button.cross;
    buttonsR.up = driveController.state.button.up;
    buttonsR.down = driveController.state.button.down;
    buttonsR.left = driveController.state.button.left;
    buttonsR.right = driveController.state.button.right;
    buttonsR.ps = driveController.state.button.ps;
    buttonsR.rightStickY = driveController.state.analog.stick.ly;
    buttonsR.rightStickX = driveController.state.analog.stick.lx;
    buttonsL.leftStickY = domeController.state.analog.stick.ly;
    buttonsL.leftStickX = domeController.state.analog.stick.lx;    
    #define CHECK_BUTTON_PRESSEDR(btn) (previousStateR.btn != buttonsR.btn && buttonsR.btn)
    #define CHECK_BUTTON_PRESSEDL(btn) (previousStateL.btn != buttonsL.btn && buttonsL.btn)

//    if(CHECK_BUTTON_PRESSEDR(l1)){
    if(buttonsR.l1){
      sendTo32u4Data.domeSpin = 0;
      if(buttonsR.rightStickX > joystickDeadZoneRange){
        sendTo32u4Data.flywheel = map(buttonsR.rightStickX,5,100,0,-100);
      }else if(buttonsR.rightStickX < -(joystickDeadZoneRange)){
        sendTo32u4Data.flywheel = map(buttonsR.rightStickX,5,100,0,100);
      }else{
        sendTo32u4Data.flywheel = 0; 
      }
      #ifdef debugFlywheel
        Serial.print("Flywheel Enabled: ");
        Serial.println(sendTo32u4Data.flywheel);
      #endif
    } 
    if(buttonsL.l1){
        sendTo32u4Data.flywheel = 0;
        if(buttonsL.leftStickX > joystickDeadZoneRange){
//          sendTo32u4Data.domeSpin = map(buttonsL.leftStickX,5,100,0,-100);
          sendTo32u4Data.domeSpin = buttonsL.leftStickX;
        }else if(buttonsL.leftStickX < -(joystickDeadZoneRange)){
//          sendTo32u4Data.domeSpin = map(buttonsL.leftStickX,5,100,0,100);
          sendTo32u4Data.domeSpin = buttonsL.leftStickX;
        }else{
          sendTo32u4Data.domeSpin = 0; 
        }
        #ifdef debugDomeSpin
          Serial.print("Domespin Enabled: ");
          Serial.println(sendTo32u4Data.domeSpin);
        #endif
      }

    if(CHECK_BUTTON_PRESSEDR(l3)){
      if (enableDrive == false) {
        enableDrive = true;
        DEBUG_PRINTLN("Drive Enabled");
        DEBUG_PRINTLN(enableDrive);
      } else {
        enableDrive = false; 
        DEBUG_PRINTLN("Drive Disabled");
        DEBUG_PRINTLN(enableDrive);
      }
      sendTo32u4Data.driveEnabled = enableDrive; 
    }
    if(CHECK_BUTTON_PRESSEDL(l3)){
      if (reverseDrive == false) {
        reverseDrive = true;
        DEBUG_PRINTLN("Drive Reversed");
      } else {
        reverseDrive = false; 
        DEBUG_PRINTLN("Drive Forward");
      }
      sendTo32u4Data.moveL3 = reverseDrive; 
    }
    if((buttonsL.leftStickX < -(joystickDeadZoneRange)) || (buttonsL.leftStickX > (joystickDeadZoneRange))){
//      sendTo32u4Data.leftStickX = constrain(map((buttonsL.leftStickX),-128,128,0,100),0,100); 
      sendTo32u4Data.leftStickX = buttonsL.leftStickX;
      DEBUG_PRINT("Dome LeftStickX: ");
      DEBUG_PRINTLN(sendTo32u4Data.leftStickX);
    }else{
      sendTo32u4Data.leftStickX = 0; 
    }
    if((buttonsL.leftStickY < -(joystickDeadZoneRange)) || (buttonsL.leftStickY > (joystickDeadZoneRange))){
//      sendTo32u4Data.leftStickY = constrain(map((buttonsL.leftStickY),-128,128,0,100),0,100); 
      sendTo32u4Data.leftStickY = buttonsL.leftStickY;
      DEBUG_PRINT("Dome LeftStickY: ");
      DEBUG_PRINTLN(sendTo32u4Data.leftStickY);
    }else{
      sendTo32u4Data.leftStickY = 0; 
    }  
    if((buttonsR.rightStickX < -(joystickDeadZoneRange)) || (buttonsR.rightStickX > (joystickDeadZoneRange))){
//      sendTo32u4Data.rightStickX = constrain(map((buttonsL.rightStickX),-128,128,0,100),0,100); 
      buttonsR.rightStickX = buttonsR.rightStickX;
      DEBUG_PRINT("Drive RightStickX: ");
      DEBUG_PRINTLN(buttonsR.rightStickX);
    }else{
      buttonsR.rightStickX = 0; 
    }
    if((buttonsR.rightStickY < -(joystickDeadZoneRange)) || (buttonsR.rightStickY > (joystickDeadZoneRange))){
//      sendTo32u4Data.rightStickY = constrain(map((buttonsL.rightStickY),-128,128,0,100),0,100); 
      buttonsR.rightStickY = buttonsR.rightStickY;
      DEBUG_PRINT("Drive RightStickY: ");
      DEBUG_PRINTLN(buttonsR.rightStickY);
    }else{
      buttonsR.rightStickY = 0; 
    }  
  }  else controllerConnected = false;    
#endif

}
  
void receiveIMU(){
  if (IMUconnected) {
    if(recIMU.receiveData()){
     IMUmillis = currentMillis; 
     sendTo32u4Data.roll = receiveIMUData.roll;
     sendTo32u4Data.pitch = receiveIMUData.pitch;
     DEBUG_PRINT("IMU ROLL: ");
     DEBUG_PRINT(receiveIMUData.roll);
     DEBUG_PRINT(" IMU PITCH: ");
     DEBUG_PRINT(receiveIMUData.pitch);
      if(IMUconnected == false) {
        IMUconnected = true;
        DEBUG_PRINTLN("IMU Connected");
      }
      else if((currentMillis - IMUmillis) > 25) {
        IMUconnected = false; 
        DEBUG_PRINTLN("IMU Not Connected");
      }
    } else {
        IMUconnected = false; 
    }
  }
}


void sendDataTo32u4(){
 // SerialDebug.println(receiveIMUData.pitch);
  send32u4.sendData(); 
  if(rec32u4.receiveData()){
    rec32u4Millis = currentMillis;
    if(feather2Connected == false){
      feather2Connected = true;
      DEBUG_PRINTLN("32u4 Connected");
    } else {
        feather2Connected = false; 
        DEBUG_PRINTLN("32u4 Not Connected");
    }
   } else if((currentMillis - rec32u4Millis) > 100){
      feather2Connected = false; 
   }       
}



void S2S_Movement(){
//  Using the DFRobot Motor Driver we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop

  if (IMUconnected){
  Input_S2S_Servo = receiveIMUData.roll *-1;
  } else {
    Input_S2S_Servo = Input_S2S_Servo;
  }
  if(sendTo32u4Data.moveL3 == false){
    buttonsR.rightStickX *= -1; 
  }
  
  Setpoint_S2S_Servo = buttonsR.rightStickX; 
  
  myPID_S2S_Servo.Compute();
  
  Input_S2S_Stabilization = (map(analogRead(S2SPot_pin),0,1023,0,270)+S2S_offset)*-1;
  //Input_S2S_Stabilization = (map(analogRead(S2SPot_pin),0,1023,0,270)-160)*-1;
  
  Setpoint_S2S_Stabilization = Output_S2S_Servo;
  myPID_S2S_Stabilization.Compute(); 
  
  if(Output_S2S_Stabilization > 5 && controllerConnected && enableDrive){
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, HIGH); // Motor 1 Forward
    analogWrite(S2S_pwm, map(abs(Output_S2S_Stabilization),0,255,0,220));
//    analogWrite(S2S_pin_2, 0); 
  }else if(Output_S2S_Stabilization < -5 && controllerConnected && enableDrive){
    digitalWrite(S2S_pin_1, HIGH);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 Backwards
//    analogWrite(S2S_pin_1,  0);
    analogWrite(S2S_pwm, map(Output_S2S_Stabilization,-255,0,220,0));
    
  }else{
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 stopped
  }

}

void drive_Movement(){
//  Using the DFRobot Motor Driver we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop

  Input_Drive = receiveIMUData.pitch; 
  if(reverseDrive == false){
    buttonsR.rightStickY *= -1;
  }
  if(Setpoint_Drive_In > buttonsR.rightStickY){
    Setpoint_Drive_In-=driveDelay; 
  } else if(Setpoint_Drive_In < buttonsR.rightStickY){
    Setpoint_Drive_In+=driveDelay; 
  }
  
  Setpoint_Drive = map(Setpoint_Drive_In,-100,100,-40,40); 
  
  //Setpoint_Drive = buttons.rightStickY * -1; 
  myPID_Drive.Compute(); 
  
  if (Output_Drive > 5 && controllerConnected && enableDrive) {
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, HIGH); // Motor 2 Forward
    analogWrite(Drive_pwm, map(abs(Output_Drive),0,255,0,220));
//    analogWrite(Drive_pin_1,0); 
  } else if(Output_Drive < -5 && controllerConnected && enableDrive) {
    digitalWrite(Drive_pin_1, HIGH);
    digitalWrite(Drive_pin_2, LOW); // Motor 2 Backwards
//      analogWrite(Drive_pin_2, 0);
    analogWrite(Drive_pwm, map(abs(Output_Drive),0,255,0,220));
  
   } else {
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, LOW); // Motor 2 stopped
      }

}

void sounds() {
#ifdef MUSICPLAYER_FEATHERWING
  if (musicPlayer.playingMusic) {
    sendPSI = 1;
  }
  if(buttonsL.circle == 1  && musicPlayer.stopped()){
  //if(musicPlayer.stopped()){
    musicPlayer.startPlayingFile(soundNames[i]);
    #ifdef debugSounds 
        SerialDebug.print("Playing Track: "); SerialDebug.println(soundNames[i]); 
    #endif
    i++;
    if(i >= numberOfTracks){
      i=0;
    }
  }
  if(buttonsL.cross){
    musicPlayer.stopPlaying();
  }
  if(buttonsL.l1){
    if(buttonsL.up){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("ACK0000.ogg"); 
      #endif
      musicPlayer.startPlayingFile("ACK0000.ogg");  //Play Acknowlege 0000
    } else if(buttonsL.right){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("ACK0001.ogg"); 
      #endif
      musicPlayer.startPlayingFile("ACK0001.ogg");  //Play Acknowlege 0001
    } else if(buttonsL.down){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("ACK0002.ogg"); 
      #endif
      musicPlayer.startPlayingFile("ACK0002.ogg");  //Play Acknowlege 0002
    } else if(buttonsL.left){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("ACK0003.ogg"); 
      #endif
      musicPlayer.startPlayingFile("ACK0003.ogg");  //Play Acknowlege 0003
    } else if (buttonsR.up){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("FUN0000.ogg"); 
      #endif
      musicPlayer.startPlayingFile("FUN0000.ogg");  //Play Acknowlege 0000
    } else if(buttonsR.right){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("FUN0001.ogg"); 
      #endif
      musicPlayer.startPlayingFile("FUN0001.ogg");  //Play Acknowlege 0001
    } else if(buttonsR.down){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("FUN0002.ogg"); 
      #endif
      musicPlayer.startPlayingFile("FUN0002.ogg");  //Play Acknowlege 0002
    } else if(buttonsR.left){
      #ifdef debugSounds 
      SerialDebug.print("Playing Track: "); 
      SerialDebug.println("FUN0003.ogg"); 
      #endif
      musicPlayer.startPlayingFile("FUN0003.ogg");  //Play Acknowlege 0003
    }
  }
#endif

}

void BatteryLevel(){
  Serial.print("Nav1 Controller battery level is: ");
  switch (driveController.state.status.battery) {
    case PSController::kHigh:
    Serial.println("High");
    break;
    case PSController::kFull:
    Serial.println("High");
    break;
    case PSController::kCharging:
    Serial.println("Charging");
    break;
    case PSController::kLow:
    Serial.println("Low");
    break;
    case PSController::kDying:
    Serial.println("Critical");
    break;
    case PSController::kShutdown:
    Serial.println("Shutting Down");
    break;
    default:
    Serial.println("Checking...");
    BatteryLevel();
    break;
  }
  switch (driveController.state.status.battery) {
    case PSController::kHigh:
    Serial.println("High");
    break;
    case PSController::kFull:
    Serial.println("High");
    break;
    case PSController::kCharging:
    Serial.println("Charging");
    break;
    case PSController::kLow:
    Serial.println("Low");
    break;
    case PSController::kDying:
    Serial.println("Critical");
    break;
    case PSController::kShutdown:
    Serial.println("Shutting Down");
    break;
    default:
    Serial.println("Checking...");
    BatteryLevel();
    break;
  }
}

/*
 * Battery Check Subroutine to check Move Controllers as they connect - displaying the battery status on Serial (will also display in web site)
*/
void batterycheck() {
  if (driveController.isConnected()) {
    if (driveController.state.status.battery == PSController::kCharging ) Serial.println("The drive controller battery charging");
    else if (driveController.state.status.battery == PSController::kFull ) Serial.println("The drive controller battery charge is FULL");
    else if (driveController.state.status.battery == PSController::kHigh ) Serial.println("The drive controller battery charge is HIGH");
    else if (driveController.state.status.battery == PSController::kLow ) Serial.println("The drive controller battery charge is LOW");
    else if (driveController.state.status.battery == PSController::kDying ) Serial.println("The drive controller battery charge is DYING");
    else if (driveController.state.status.battery == PSController::kShutdown ) Serial.println("The drive controller battery charge is SHUTDOWN");
    else {
      Serial.print("The drive controller battery charge is UNDEFINED ");
      Serial.println(driveController.state.status.battery);
      batterycheck();
    }
  }
    if (domeController.isConnected()) {
    if (domeController.state.status.battery == PSController::kCharging ) Serial.println("The dome controller battery charging");
    else if (domeController.state.status.battery == PSController::kFull ) Serial.println("The dome controller battery charge is FULL");
    else if (domeController.state.status.battery == PSController::kHigh ) Serial.println("The dome controller battery charge is HIGH");
    else if (domeController.state.status.battery == PSController::kLow ) Serial.println("The dome controller battery charge is LOW");
    else if (domeController.state.status.battery == PSController::kDying ) Serial.println("The dome controller battery charge is DYING");
    else if (domeController.state.status.battery == PSController::kShutdown ) Serial.println("The dome controller battery charge is SHUTDOWN");
    else {
      Serial.print("The dome controller battery charge is UNDEFINED ");
      Serial.println(domeController.state.status.battery);
      batterycheck();
    }
  }
}



void debugRoutines(){
#ifdef debugRemote
  SerialDebug.print(sendTo32u4Data.leftStickY); SerialDebug.print('\t');
  SerialDebug.print(sendTo32u4Data.leftStickX); SerialDebug.print('\t');
  SerialDebug.print(buttons.rightStickY); SerialDebug.print('\t');
  SerialDebug.print(buttons.rightStickX); SerialDebug.print('\t');
  SerialDebug.print(buttons.l1); SerialDebug.print('\t');
  SerialDebug.print(buttons.l2); SerialDebug.print('\t');
  SerialDebug.print(sendTo32u4Data.xboxL3); SerialDebug.print('\t');
  SerialDebug.print(buttons.r1); SerialDebug.print('\t');
  SerialDebug.print(buttons.r2); SerialDebug.print('\t');
  SerialDebug.print(sendTo32u4Data.xboxR3); SerialDebug.print('\t');
  SerialDebug.print(buttons.a); SerialDebug.print('\t');
  SerialDebug.print(buttons.b); SerialDebug.print('\t');
  SerialDebug.print(buttons.x); SerialDebug.print('\t');
  SerialDebug.print(buttons.y); SerialDebug.print('\t');
  SerialDebug.print(buttons.up); SerialDebug.print('\t');
  SerialDebug.print(buttons.down); SerialDebug.print('\t');
  SerialDebug.print(buttons.left); SerialDebug.print('\t');
  SerialDebug.print(buttons.right); SerialDebug.print('\t');
  SerialDebug.print(buttons.back); SerialDebug.print('\t');
  SerialDebug.print(enableDrive); SerialDebug.print('\t');
  SerialDebug.println(buttons.xbox); 
#endif

//
#ifdef debugIMU
  SerialDebug.print("IMU Pitch: ");
  SerialDebug.print(receiveIMUData.pitch); SerialDebug.print('\t');
  SerialDebug.print("IMU Roll: "); SerialDebug.print('\t');
  SerialDebug.println(receiveIMUData.roll);
#endif


#ifdef debugMainDrive
  SerialDebug.print(F("In/Pitch (Input_Drive): ")); SerialDebug.print(Input_Drive); SerialDebug.print('\t'); 
  SerialDebug.print(F("Set/Joy (Setpoint_Drive): ")); SerialDebug.print(Setpoint_Drive); SerialDebug.print('\t');
  SerialDebug.print(F("Out (Output_Drive): ")); SerialDebug.print(Output_Drive); SerialDebug.print('\t');
  SerialDebug.print(F("Cont. Conn. (controllerConnected): ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
  SerialDebug.print(F("En (enableDrive): ")); SerialDebug.println(enableDrive); 
#endif

#ifdef debugS2S
  SerialDebug.print(F("Servo: In/Roll: ")); SerialDebug.print(Input_S2S_Servo); SerialDebug.print('\t'); 
  SerialDebug.print(F("Set/Joy: ")); SerialDebug.print(Setpoint_S2S_Servo); SerialDebug.print('\t');
  SerialDebug.print(F("Out: ")); SerialDebug.print(Output_S2S_Servo); SerialDebug.print('\t');
  
  SerialDebug.print(F("Stab: In/Pot: ")); SerialDebug.print(Input_S2S_Stabilization); SerialDebug.print('\t'); 
  SerialDebug.print(F("Set/Servo Out: ")); SerialDebug.print(Output_S2S_Servo); SerialDebug.print('\t');
  SerialDebug.print(F("Out: ")); SerialDebug.print(Output_S2S_Stabilization); SerialDebug.print('\t');
  SerialDebug.print(F("Cont. Conn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
  SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
#endif

     
     
  
  }
