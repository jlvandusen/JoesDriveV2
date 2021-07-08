   
#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)
//#define debugRemote
#define debugIMU
//#define debugMainDrive
//#define debugS2S
//#define debugSounds
#define MOVECONTROLLER
//#define XBOXCONTROLLER   

/*
  MAC ADDRESS DEFINITIONS
  Signifies the primary MAC of the ESP32/USB BT chip
*/
// Bluetooth address of this ESP32 device. If you already have a Shadow system configured
// the easiest thing is reuse the address of your USB Bluetooth dongle here. Alternatively,
// you can use sixaxispair to pair your controllers with the ESP32.
// https://www.adafruit.com/product/3619

#define MasterNav "7c:9e:bd:d7:63:c6" 
   
/*
  PIN DEFINITIONS
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
 Modified USB Host library: https://github.com/gdsports/USB_Host_Library_SAMD
 Modified ESP32 BT Host Library https://github.com/reeltwo/PSController
*/

/*
  LIBRARY DEFINITIONS
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
//#include <i2cSimpleTransfer.h>
//#include <EasyTransferI2C_NL.h>
//#include <EasyTransferI2C.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
#include <PID_v1.h>
#include "wiring_private.h" // pinPeripheral() function

#define BUFFER_LENGTH 64
#define TWI_BUFFER_LENGTH 64
#define driveDelay .75

//#ifdef XBOXCONTROLLER
//USBHost UsbH;
//XBOXRECV Xbox(&UsbH);
//#endif

//#ifdef MOVECONTROLLER
//USBHost UsbH;
//BTD Btd(&UsbH); // You have to create the Bluetooth Dongle instance like so
//PS3BT *PS3[2]; // We will use this pointer to store the two instance, you can easily make it larger if you like, but it will use a lot of RAM!
//#endif

#ifdef MOVECONTROLLER
PSController driveController(DRIVE_CONTROLLER_MAC); //define the driveController variable to be used against the Nav1 and Nav2 controllers.
PSController domeController(DOME_CONTROLLER_MAC);
#endif

//create UART object *************************************
EasyTransfer recIMU; 
EasyTransfer send32u4;
EasyTransfer rec32u4; 
//EasyTransferI2C_NL recIMU;
//EasyTransferI2C_NL send32u4;
//EasyTransferI2C_NL rec32u4; 
//EasyTransferI2C recIMU;
//EasyTransferI2C send32u4;
//EasyTransferI2C rec32u4; 

struct RECEIVE_DATA_STRUCTURE_IMU{
  float pitch;
  float roll; 
};

RECEIVE_DATA_STRUCTURE_IMU receiveIMUData;


//create SEND object  *************************************

//#ifdef XBOXCONTROLLER
//struct SEND_DATA_STRUCTURE_32u4{  
//  bool driveEnabled;
//  int8_t domeSpin;
//  bool xboxL3;
//  int8_t flywheel;
//  bool xboxR3;
//  int8_t leftStickX;
//  int8_t leftStickY;
//  bool psiFlash;
//  float pitch;
//  float roll; 
//};
//#endif

#ifdef MOVECONTROLLER
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
#endif

//give a name to the group of data
SEND_DATA_STRUCTURE_32u4 sendTo32u4Data; // - slave_config

struct RECEIVE_DATA_STRUCTURE_32u4{ // - SLAVE_DATA
  int16_t tiltAngle; 
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE_32u4 receiveFrom32u4Data; // - slave_data

//#ifdef XBOXCONTROLLER
//struct controllerButtons{
//  bool a,b,x,y,up,down,left,right,xbox,back,l1,r1,start;
//  int8_t rightStickX,rightStickY,l2,r2;
//};
//controllerButtons buttons;
//#endif

#ifdef MOVECONTROLLER
struct controllerButtonsL{
  bool cross,circle,up,down,left,right,ps,l1,l3;
  int8_t leftStickX,leftStickY,l2;
};
struct controllerButtonsR{
  bool cross,circle,up,down,left,right,ps,l1,l3;
  int8_t rightStickX,rightStickY,l2;
};
controllerButtonsL buttonsL;
controllerButtonsR buttonsR;
int8_t joystickDeadZoneRange = 25;  // For controllers that centering problems, use the lowest number with no drift
#endif

//*** Set up Music Maker Featherwing *******************************************
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)
#define VS1053_CS      32     // VS1053 chip select pin (output)
#define VS1053_DCS     33     // VS1053 Data/command select pin (output)
#define CARDCS         14     // Card chip select pin
// DREQ should be an Int pin *if possible* (not possible on 32u4)
#define VS1053_DREQ    15     // VS1053 Data request, ideally an Interrupt pin
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);
  
//#ifndef MOVECONTROLLER
//  Uart Serial2 (&sercom1, 12, 11, SERCOM_RX_PAD_3, UART_TX_PAD_0);
//  void SERCOM1_Handler()
//  {
//    Serial2.IrqHandler();
//  }
//
//  Uart Serial3 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0);
//  void SERCOM3_Handler()
//  {
//    Serial3.IrqHandler();
//  }
//#endif

#ifdef MOVECONTROLLER
#define SERIAL2_BAUD_RATE 74880
#define SERIAL2_RX_PIN 27
#define SERIAL2_TX_PIN 12
//#include <SoftwareSerial.h>
//SoftwareSerial Serial3;
//#define SERIAL3_BAUD_RATE 74880
//#define SERIAL3_RX_PIN 21
//#define SERIAL3_TX_PIN 20
#endif


#define SerialDebug Serial

bool enableDrive,reverseDrive; 

unsigned long currentMillis, IMUmillis, lastLoopMillis, lastDomeMillis, rec32u4Millis, lastPrintMillis; 
bool IMUconnected, controllerConnected, Send_Rec, feather2Connected, drivecontrollerConnected, domecontrollerConnected; 
//Define Variables we'll be connecting to
double Setpoint_S2S_Servo, Input_S2S_Servo, Output_S2S_Servo;
double Setpoint_S2S_Stabilization, Input_S2S_Stabilization, Output_S2S_Stabilization;
double Setpoint_Drive, Input_Drive, Output_Drive, Setpoint_Drive_In;

int numberOfTracks = 55; 
int i; 
char *soundNames[]={"ACK0000.ogg", "FUN0000.ogg",  "FUN0001.ogg",  "BEEP0000.ogg",  "ACK0001.ogg",  "BEEP0001.ogg",  "BEEP0002.ogg",  "BEEP0003.ogg",  "BEEP0004.ogg",  "BB80010.ogg",  
"BB80011.ogg",  "BB80012.ogg",  "BB80013.ogg",  "BB80014.ogg",  "BB80015.ogg",  "BB80016.ogg",  "BB80017.ogg",  "BB80018.ogg",  "BB80019.ogg",  "BB80021.ogg",  
"BB80022.ogg",  "BB80023.ogg",  "BB80024.ogg",  "BB80025.ogg",  "BB80026.ogg",  "BB80027.ogg",  "BB80028.ogg",  "BB80029.ogg",  "BB80030.ogg",  "BB80031.ogg",  "BB80032.ogg",  
"BB80033.ogg",  "BB80034.ogg",  "BB80035.ogg",  "BB80036.ogg",  "BB80037.ogg",  "BB80038.ogg",  "BB80039.ogg",  "BB80040.ogg",  "BB80041.ogg",  "BB80042.ogg",  "BB80043.ogg",  
"BB80044.ogg",  "BB80045.ogg",  "BB80046.ogg",  "BB80047.ogg",  "BB80048.ogg",  "BB80049.ogg",  "BB80050.ogg",  "BB80051.ogg",  "BB80052.ogg",  "BB80053.ogg",  "BB80054.ogg",  
"BB80055.ogg",  "BB80056.ogg"};

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
  #ifdef MOVECONTROLLER
  PSController::startListening(MasterNav);
  String address = PSController::getDeviceAddress();
  Serial.print("Please write down the following MAC and Assign to your Nav Controller(s): ");
  Serial.println(address);
  Serial.println("Bluetooth Ready.");
  #endif
  
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
      DEBUG_PRINT("Flywheel Enabled: ");
      DEBUG_PRINTLN(sendTo32u4Data.flywheel);
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
        DEBUG_PRINT("Domespin Enabled: ");
        DEBUG_PRINTLN(sendTo32u4Data.domeSpin);
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

#ifdef MOVECONTROLLER //******Move Controller
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
  switch (driveController.state.status.battery){
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
  }
}

   //------------------ Battery -------------------------
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
//
//     #ifdef debugRemote
//        
//            SerialDebug.print(sendTo32u4Data.leftStickY); SerialDebug.print('\t');
//            SerialDebug.print(sendTo32u4Data.leftStickX); SerialDebug.print('\t');
//            SerialDebug.print(buttons.rightStickY); SerialDebug.print('\t');
//            SerialDebug.print(buttons.rightStickX); SerialDebug.print('\t');
//            SerialDebug.print(buttons.l1); SerialDebug.print('\t');
//            SerialDebug.print(buttons.l2); SerialDebug.print('\t');
//            SerialDebug.print(sendTo32u4Data.xboxL3); SerialDebug.print('\t');
//            SerialDebug.print(buttons.r1); SerialDebug.print('\t');
//            SerialDebug.print(buttons.r2); SerialDebug.print('\t');
//            SerialDebug.print(sendTo32u4Data.xboxR3); SerialDebug.print('\t');
//            SerialDebug.print(buttons.a); SerialDebug.print('\t');
//            SerialDebug.print(buttons.b); SerialDebug.print('\t');
//            SerialDebug.print(buttons.x); SerialDebug.print('\t');
//            SerialDebug.print(buttons.y); SerialDebug.print('\t');
//            SerialDebug.print(buttons.up); SerialDebug.print('\t');
//            SerialDebug.print(buttons.down); SerialDebug.print('\t');
//            SerialDebug.print(buttons.left); SerialDebug.print('\t');
//            SerialDebug.print(buttons.right); SerialDebug.print('\t');
//            SerialDebug.print(buttons.back); SerialDebug.print('\t');
//            SerialDebug.print(enableDrive); SerialDebug.print('\t');
//            SerialDebug.println(buttons.xbox); 
//      #endif

//
      #ifdef debugIMU
            SerialDebug.print(receiveIMUData.pitch); SerialDebug.print('\t');
            SerialDebug.println(receiveIMUData.roll);

      #endif


//      #ifdef debugMainDrive
//
//            SerialDebug.print(F("In/Pitch: ")); SerialDebug.print(Input_Drive); SerialDebug.print('\t'); 
//            SerialDebug.print(F("Set/Joy: ")); SerialDebug.print(Setpoint_Drive); SerialDebug.print('\t');
//            SerialDebug.print(F("Out: ")); SerialDebug.print(Output_Drive); SerialDebug.print('\t');
//            SerialDebug.print(F("Cont. Conn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
//            SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
//            
//      #endif

//      #ifdef debugS2S
//
//            SerialDebug.print(F("Servo: In/Roll: ")); SerialDebug.print(Input_S2S_Servo); SerialDebug.print('\t'); 
//            SerialDebug.print(F("Set/Joy: ")); SerialDebug.print(Setpoint_S2S_Servo); SerialDebug.print('\t');
//            SerialDebug.print(F("Out: ")); SerialDebug.print(Output_S2S_Servo); SerialDebug.print('\t');
//            
//            SerialDebug.print(F("Stab: In/Pot: ")); SerialDebug.print(Input_S2S_Stabilization); SerialDebug.print('\t'); 
//            SerialDebug.print(F("Set/Servo Out: ")); SerialDebug.print(Output_S2S_Servo); SerialDebug.print('\t');
//            SerialDebug.print(F("Out: ")); SerialDebug.print(Output_S2S_Stabilization); SerialDebug.print('\t');
//            SerialDebug.print(F("Cont. Conn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
//            SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
//            
//      #endif

     
     
  
  }
