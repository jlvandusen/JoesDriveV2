/*
 * Joe's Drive  - V2 1/2022
 * Primary ESP32 HUZZAH32
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need libraries: 
 * HUZZAH32 From Adafruit: https://www.adafruit.com/product/3619 used for Dome and Primary Feather
 * https://github.com/ERROPiX/ESP32_AnalogWrite for proper analogWrite library working with ESP32
 * ESp32 Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * On Line 117 - replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 * Modified ESP32 BT Host Library https://github.com/reeltwo/PSController
 * 
*/

/*
 * Debug Configurations
 * Comment and uncomment which function needs to be debugged
*/

#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)
#define SerialDebug Serial

//#define debug32u4
//#define debugRemote
//#define debugIMU
//#define debugPOTS
//#define debugMainDrive
#define debugFlywheel
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


#define NeoPixel_pin 27
#define S2S_pwm 33 //22  // SCL 22 and SDA 23
#define S2S_pin_1 26 //A0
#define S2S_pin_2 25 //A1
#define Drive_pwm 21
#define Drive_pin_1 4 //A5
#define Drive_pin_2 27
#define S2SPot_pin A2 //GPIO 34
#define flyWheelMotor_pwm 15
#define flyWheelMotor_pin_A 32
#define flyWheelMotor_pin_B 14
#define voltageSensor_Pin A3 //A4 or 13

#define S2S_offset -135 // -120 (divided the Input_S2S_Stabilization with a 200k OHM WH148-B200K it is 0-4095 to 0-270 so its -135 to 135)
#define S2S_maxTilt 20  // max tilt using the joystick; max is 25

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
  LIBRARY DEFINITIONS
*/

//#ifdef XBOXCONTROLLER
//#include <XBOXRECV.h>
//#endif

#ifdef MOVECONTROLLER
/*
 Modified ESP32 BT Host Library https://github.com/reeltwo/PSController
*/
#include <analogWrite.h>  // https://www.arduinolibraries.info/libraries/esp32-analog-write
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
//#include <SPI.h>
//#include <SD.h>
//#include <Adafruit_VS1053.h>
#include <PID_v1.h>
#include "wiring_private.h" // pinPeripheral() function

#define BUFFER_LENGTH 64
#define TWI_BUFFER_LENGTH 64
#define driveDelay .75

#ifdef MOVECONTROLLER
PSController driveController(DRIVE_CONTROLLER_MAC); //define the driveController variable to be used against the Nav1 and Nav2 controllers.
PSController domeController(DOME_CONTROLLER_MAC);
#endif

/*
 * create UART object ************************
*/
EasyTransfer recIMU; 
EasyTransfer send32u4;
EasyTransfer rec32u4; 

/*
 * Create receive IMU from Trinket M0 Objects
*/

struct RECEIVE_DATA_STRUCTURE_IMU{
  float pitch;
  float roll; 
};
RECEIVE_DATA_STRUCTURE_IMU receiveIMUData;


/*
 * Create Send to the 32u4 Feather Objects
*/
//#ifdef XBOXCONTROLLER
//struct SEND_DATA_STRUCTURE_32u4{  
//  bool driveEnabled;
//  int8_t domeSpin;
//  bool xboxL3;
//  int8_t flywheel;
//  bool xboxR3;
//  int8_t leftStickX;
//  int8_t leftStickY;
//  int8_t rightStickX;
//  int8_t rightStickY;
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

/*
 * Create Receive from 32u4 Objects
*/
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

/*
 * Create Serial 2 to send to the 32u4 / ESP32-S2 (qwiic)
*/

#define SERIAL2_BAUD_RATE 74880
#define SERIAL2_RX_PIN 13
#define SERIAL2_TX_PIN 12

//SCL - General purpose IO pin #22
//SDA - General purpose IO pin #23
#include <SoftwareSerial.h>
SoftwareSerial Serial3;
#define SERIAL3_BAUD_RATE 115200
#define SERIAL3_RX_PIN 22
#define SERIAL3_TX_PIN 23

bool enableDrive,reverseDrive; 

unsigned long currentMillis, IMUmillis, lastLoopMillis, lastDomeMillis, rec32u4Millis, lastPrintMillis; 
bool IMUconnected, controllerConnected, Send_Rec, feather2Connected, drivecontrollerConnected, domecontrollerConnected; 
//Define Variables we'll be connecting to
double Setpoint_S2S_Servo, Input_S2S_Servo, Output_S2S_Servo;
double Setpoint_S2S_Stabilization, Input_S2S_Stabilization, Output_S2S_Stabilization;
double Setpoint_Drive, Input_Drive, Output_Drive, Setpoint_Drive_In;
double S2SPots;

//int numberOfTracks = 55; 
//int i; 
//char *soundNames[]={"ACK0000.ogg", "FUN0000.ogg",  "FUN0001.ogg",  "BEEP0000.ogg",  "ACK0001.ogg",  "BEEP0001.ogg",  "BEEP0002.ogg",  "BEEP0003.ogg",  "BEEP0004.ogg",  "BB80010.ogg",  
//"BB80011.ogg",  "BB80012.ogg",  "BB80013.ogg",  "BB80014.ogg",  "BB80015.ogg",  "BB80016.ogg",  "BB80017.ogg",  "BB80018.ogg",  "BB80019.ogg",  "BB80021.ogg",  
//"BB80022.ogg",  "BB80023.ogg",  "BB80024.ogg",  "BB80025.ogg",  "BB80026.ogg",  "BB80027.ogg",  "BB80028.ogg",  "BB80029.ogg",  "BB80030.ogg",  "BB80031.ogg",  "BB80032.ogg",  
//"BB80033.ogg",  "BB80034.ogg",  "BB80035.ogg",  "BB80036.ogg",  "BB80037.ogg",  "BB80038.ogg",  "BB80039.ogg",  "BB80040.ogg",  "BB80041.ogg",  "BB80042.ogg",  "BB80043.ogg",  
//"BB80044.ogg",  "BB80045.ogg",  "BB80046.ogg",  "BB80047.ogg",  "BB80048.ogg",  "BB80049.ogg",  "BB80050.ogg",  "BB80051.ogg",  "BB80052.ogg",  "BB80053.ogg",  "BB80054.ogg",  
//"BB80055.ogg",  "BB80056.ogg"};

  //Specify the links and initial tuning parameters
//double Kp_S2S_Servo=.5, Ki_S2S_Servo=0, Kd_S2S_Servo=0;
double Kp_S2S_Servo=10, Ki_S2S_Servo=0, Kd_S2S_Servo=0;
PID myPID_S2S_Servo(&Input_S2S_Servo, &Output_S2S_Servo, &Setpoint_S2S_Servo, Kp_S2S_Servo, Ki_S2S_Servo, Kd_S2S_Servo, DIRECT);

  //Specify the links and initial tuning parameters
//double Kp_S2S_Stabilization=10, Ki_S2S_Stabilization=0, Kd_S2S_Stabilization=0;
double Kp_S2S_Stabilization=5, Ki_S2S_Stabilization=0, Kd_S2S_Stabilization=0;
PID myPID_S2S_Stabilization(&Input_S2S_Stabilization, &Output_S2S_Stabilization, &Setpoint_S2S_Stabilization, Kp_S2S_Stabilization, Ki_S2S_Stabilization, Kd_S2S_Stabilization, DIRECT);

//double Kp_Drive=6, Ki_Drive=0, Kd_Drive=0;
double Kp_Drive=10, Ki_Drive=0, Kd_Drive=0;
PID myPID_Drive(&Input_Drive, &Output_Drive, &Setpoint_Drive, Kp_Drive, Ki_Drive, Kd_Drive, DIRECT);

bool enabledDrive;
  
void setup() {
  delay(10000); // Wait for Boot
  currentMillis = millis();
  Serial.begin(115200);
  Serial1.begin(115200); //115200 74880
  Serial2.begin(SERIAL2_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
//  Serial3.begin(SERIAL3_BAUD_RATE, SWSERIAL_8N1, SERIAL3_RX_PIN, SERIAL3_TX_PIN,false,256);
  #ifdef MOVECONTROLLER
  PSController::startListening(MasterNav);
  String address = PSController::getDeviceAddress();
  Serial.print("Please write down the following MAC and Assign to your Nav Controller(s): ");
  Serial.println(address);
  Serial.println("Bluetooth Ready.");
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
//  if(!recIMU.receiveData()){
//    IMUmillis = currentMillis; 
//    IMUconnected = false; 
//    Serial.println("IMU Not Connected");
//  } else {
//    if(IMUconnected == false){
//      IMUmillis = currentMillis;
//      IMUconnected = true;
//      Serial.println("IMU Connected");
//    }
//    if((currentMillis - IMUmillis) > 25) {
//      IMUconnected = false; 
//      Serial.println("IMU Not Connected");
//    }
//  }


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

}





  
void loop() {
  currentMillis = millis(); 
  if(currentMillis - lastLoopMillis >= 10) {
    lastLoopMillis = currentMillis; 
    receiveIMU();
    receiveRemote();
    S2S_Movement(); 
    drive_Movement(); 
    sendDataTo32u4();
    spinFlywheel();
//    sounds(); 
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
    if(buttonsL.l1){
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
//    if(buttonsL.l1){
//        sendTo32u4Data.flywheel = 0;
//        if(buttonsL.leftStickX > joystickDeadZoneRange){
////          sendTo32u4Data.domeSpin = map(buttonsL.leftStickX,5,100,0,-100);
//          sendTo32u4Data.domeSpin = buttonsL.leftStickX;
//        }else if(buttonsL.leftStickX < -(joystickDeadZoneRange)){
////          sendTo32u4Data.domeSpin = map(buttonsL.leftStickX,5,100,0,100);
//          sendTo32u4Data.domeSpin = buttonsL.leftStickX;
//        }else{
//          sendTo32u4Data.domeSpin = 0; 
//        }
//        DEBUG_PRINT("Domespin Enabled: ");
//        DEBUG_PRINTLN(sendTo32u4Data.domeSpin);
//      }

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
        #ifdef debugRemote
        DEBUG_PRINTLN("Drive Reversed");
        #endif
      } else {
        reverseDrive = false; 
        #ifdef debugRemote
        DEBUG_PRINTLN("Drive Forward");
        #endif
      }
      sendTo32u4Data.moveL3 = reverseDrive; 
    }
    if((buttonsL.leftStickX < -(joystickDeadZoneRange)) || (buttonsL.leftStickX > (joystickDeadZoneRange))){
//      sendTo32u4Data.leftStickX = constrain(map((buttonsL.leftStickX),-128,128,0,100),0,100); 
      sendTo32u4Data.leftStickX = buttonsL.leftStickX;
      #ifdef debugRemote
      DEBUG_PRINT("Dome LeftStickX: ");
      DEBUG_PRINTLN(sendTo32u4Data.leftStickX);
      #endif
    }else{
      sendTo32u4Data.leftStickX = 0; 
    }
    if((buttonsL.leftStickY < -(joystickDeadZoneRange)) || (buttonsL.leftStickY > (joystickDeadZoneRange))){
//      sendTo32u4Data.leftStickY = constrain(map((buttonsL.leftStickY),-128,128,0,100),0,100); 
      sendTo32u4Data.leftStickY = buttonsL.leftStickY;
      #ifdef debugRemote
      DEBUG_PRINT("Dome LeftStickY: ");
      DEBUG_PRINTLN(sendTo32u4Data.leftStickY);
      #endif
    }else{
      sendTo32u4Data.leftStickY = 0; 
    }  
    if((buttonsR.rightStickX < -(joystickDeadZoneRange)) || (buttonsR.rightStickX > (joystickDeadZoneRange))){
//      sendTo32u4Data.rightStickX = constrain(map((buttonsL.rightStickX),-128,128,0,100),0,100); 
      buttonsR.rightStickX = buttonsR.rightStickX;
      #ifdef debugRemote
      DEBUG_PRINT("Drive RightStickX: ");
      DEBUG_PRINTLN(buttonsR.rightStickX);
      #endif
    }else{
      buttonsR.rightStickX = 0; 
    }
    if((buttonsR.rightStickY < -(joystickDeadZoneRange)) || (buttonsR.rightStickY > (joystickDeadZoneRange))){
//      sendTo32u4Data.rightStickY = constrain(map((buttonsL.rightStickY),-128,128,0,100),0,100); 
      buttonsR.rightStickY = buttonsR.rightStickY;
      #ifdef debugRemote
      DEBUG_PRINT("Drive RightStickY: ");
      DEBUG_PRINTLN(buttonsR.rightStickY);
      #endif
    }else{
      buttonsR.rightStickY = 0; 
    }  
  }  else controllerConnected = false;    
#endif

}
  
void receiveIMU(){
  recIMU.receiveData();
  sendTo32u4Data.roll = receiveIMUData.roll;
  sendTo32u4Data.pitch = receiveIMUData.pitch;
   #ifdef debugIMU
     DEBUG_PRINT("IMU ROLL: ");
     DEBUG_PRINT(sendTo32u4Data.roll);
     DEBUG_PRINT(" IMU PITCH: ");
     DEBUG_PRINTLN(sendTo32u4Data.pitch);
   #endif
}


void sendDataTo32u4(){
 // SerialDebug.println(receiveIMUData.pitch);
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
  if(reverseDrive){
    buttonsR.rightStickX *= -1; 
  }
  if(!buttonsL.l1){
//  Setpoint_S2S_Servo = buttonsR.rightStickX; 
    Setpoint_S2S_Servo = map(buttonsR.rightStickX,5,100,0,-100);
    
    myPID_S2S_Servo.Compute();
    S2SPots = analogRead(S2SPot_pin); // For debugging raw values storing them in S2SPots as float
    Input_S2S_Stabilization = (map(analogRead(S2SPot_pin),0,4095,0,270)+S2S_offset)*-1;  
    Setpoint_S2S_Stabilization = constrain(Output_S2S_Servo, -S2S_maxTilt,S2S_maxTilt);
    myPID_S2S_Stabilization.Compute(); 

    if((Output_S2S_Stabilization >= -5) && (Input_S2S_Stabilization >= -S2S_maxTilt) && controllerConnected && enableDrive) {
      digitalWrite(S2S_pin_1, LOW);
      digitalWrite(S2S_pin_2, HIGH); // Motor 1 Forward
      
    } else if((Output_S2S_Stabilization <= 5)  && (Input_S2S_Stabilization <= S2S_maxTilt) && controllerConnected && enableDrive) {
      digitalWrite(S2S_pin_1, HIGH);
      digitalWrite(S2S_pin_2, LOW); // Motor 1 Backwards
    
    } else {
      digitalWrite(S2S_pin_1, LOW);
      digitalWrite(S2S_pin_2, LOW); // Motor 1 stopped
    }
    analogWrite(S2S_pwm, map(abs(Output_S2S_Stabilization),0,255,0,255));
  }
 
}


void drive_Movement(){
//  Using the DFRobot Motor Driver we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop

  Input_Drive = receiveIMUData.pitch;
  Setpoint_Drive_In = buttonsR.rightStickY;
  if(reverseDrive){
    buttonsR.rightStickY *= -1;
  }
  if(Setpoint_Drive_In > buttonsR.rightStickY){
    Setpoint_Drive_In-=driveDelay; 
  } else if(Setpoint_Drive_In < buttonsR.rightStickY){
    Setpoint_Drive_In+=driveDelay; 
  }
  
  Setpoint_Drive = map(Setpoint_Drive_In,-100,100,-40,40); 
  
  myPID_Drive.Compute(); 
  
  if (Output_Drive > 5 && controllerConnected && enableDrive) {
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, HIGH); // Motor 2 Forward
  } else if(Output_Drive < -5 && controllerConnected && enableDrive) {
    digitalWrite(Drive_pin_1, HIGH);
    digitalWrite(Drive_pin_2, LOW); // Motor 2 Backwards

   } else {
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, LOW); // Motor 2 stopped
      }
//  analogWrite(Drive_pwm, map(abs(Output_Drive),255,0,220,0));
  analogWrite(Drive_pwm,abs(Output_Drive));
}

void spinFlywheel() {
  if (sendTo32u4Data.flywheel >= 10 && enableDrive) {
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, HIGH); // Motor 1 Forward
    
  } else if (sendTo32u4Data.flywheel <= -10 && enableDrive) {
    digitalWrite(flyWheelMotor_pin_A, HIGH);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Backward

  } else {
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Stopped
  }
  analogWrite(flyWheelMotor_pwm,abs(sendTo32u4Data.flywheel));
}

//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}

//void drive_Movement(){
////  Using the DFRobot Motor Driver we need 3 pins
////  VCC and GND to power the driver logic
////  First pin is PWM for speed control 0 - 255
////  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop
////  Setpoint_Drive_In = buttonsR.rightStickY
////  Input_Drive = Pitch from IMU
////  Setpoint_Drive = mapping to lower numbers
////  Output_Drive = PID controlled motor values
//
//  Input_Drive = receiveIMUData.pitch; // read latest PITCH angle from MPU/IMU
//  if(reverseDrive){
//    buttonsR.rightStickY *= 1;
//  } else buttonsR.rightStickY *= -1; // reverse drive check
//  if(Setpoint_Drive_In > buttonsR.rightStickY){
//    Setpoint_Drive_In-=driveDelay; 
//  } else if(Setpoint_Drive_In < buttonsR.rightStickY){
//    Setpoint_Drive_In+=driveDelay; 
//  }
//  
//  Setpoint_Drive = map(Setpoint_Drive_In,-255,255,-40,40); 
//  
//  //Setpoint_Drive = buttons.rightStickY * -1; 
//  myPID_Drive.Compute(); 
//
//  if (Output_Drive > 5 && controllerConnected && enableDrive) {
//    digitalWrite(Drive_pin_1, LOW);
//    digitalWrite(Drive_pin_2, 1); // Motor 2 Forward
//    analogWrite(Drive_pwm, map(abs(Output_Drive),-255,255,0,255));
//  } else if (Output_Drive < -5 && controllerConnected && enableDrive) {
//    digitalWrite(Drive_pin_1, 1);
//    digitalWrite(Drive_pin_2, 0); // Motor 2 Backwards
//    analogWrite(Drive_pwm, map(abs(Output_Drive),-255,255,0,255));
//   } else {
//    digitalWrite(Drive_pin_1, 0);
//    digitalWrite(Drive_pin_2, 0); // Motor 2 stopped
//    }
//
//}

//void sounds() {
//
//#ifdef MOVECONTROLLER //******Move Controller
//  if(buttonsL.circle == 1  && musicPlayer.stopped()){
//  //if(musicPlayer.stopped()){
//    musicPlayer.startPlayingFile(soundNames[i]);
//    #ifdef debugSounds 
//        SerialDebug.print("Playing Track: "); SerialDebug.println(soundNames[i]); 
//    #endif
//    i++;
//    if(i >= numberOfTracks){
//      i=0;
//    }
//  }
//  
//  if(buttonsL.cross){
//    musicPlayer.stopPlaying();
//  }
//  if(buttonsL.l1){
//    if(buttonsL.up){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("ACK0000.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("ACK0000.ogg");  //Play Acknowlege 0000
//    } else if(buttonsL.right){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("ACK0001.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("ACK0001.ogg");  //Play Acknowlege 0001
//    } else if(buttonsL.down){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("ACK0002.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("ACK0002.ogg");  //Play Acknowlege 0002
//    } else if(buttonsL.left){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("ACK0003.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("ACK0003.ogg");  //Play Acknowlege 0003
//    } else if (buttonsR.up){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("FUN0000.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("FUN0000.ogg");  //Play Acknowlege 0000
//    } else if(buttonsR.right){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("FUN0001.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("FUN0001.ogg");  //Play Acknowlege 0001
//    } else if(buttonsR.down){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("FUN0002.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("FUN0002.ogg");  //Play Acknowlege 0002
//    } else if(buttonsR.left){
//      #ifdef debugSounds 
//      SerialDebug.print("Playing Track: "); 
//      SerialDebug.println("FUN0003.ogg"); 
//      #endif
//      musicPlayer.startPlayingFile("FUN0003.ogg");  //Play Acknowlege 0003
//    }
//  }
//#endif
//
//}

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
    SerialDebug.print(IMUconnected); SerialDebug.print('\t');
    SerialDebug.print(receiveIMUData.pitch); SerialDebug.print('\t');
    SerialDebug.println(receiveIMUData.roll);
  
  #endif
  #ifdef debugPOTS
    SerialDebug.println(Input_S2S_Stabilization); SerialDebug.print('\t');
    SerialDebug.println(S2SPots);
  #endif

  #ifdef debugS2S
    SerialDebug.print(F("S2S_pin_1: ")); SerialDebug.print(S2S_pin_1); SerialDebug.print('\t'); 
    SerialDebug.print(F("S2S_pin_2: ")); SerialDebug.print(S2S_pin_2); SerialDebug.print('\t'); 
    SerialDebug.print(F("Servo: In/Roll: ")); SerialDebug.print(Input_S2S_Servo); SerialDebug.print('\t'); 
    SerialDebug.print(F("Set/Joy: ")); SerialDebug.print(Setpoint_S2S_Servo); SerialDebug.print('\t');
    SerialDebug.print(F("Out: ")); SerialDebug.print(Output_S2S_Servo); SerialDebug.print('\t');
    
    SerialDebug.print(F("Stab: In/Pot: ")); SerialDebug.print(Input_S2S_Stabilization); SerialDebug.print('\t'); 
    SerialDebug.print(F("Set/Servo Out: ")); SerialDebug.print(Output_S2S_Servo); SerialDebug.print('\t');
    SerialDebug.print(F("Out: ")); SerialDebug.print(Output_S2S_Stabilization); SerialDebug.print('\t');
    SerialDebug.print(F("Controller_Conn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
    SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
  #endif



#ifdef debugFlywheel

    SerialDebug.print(F("flywheel: ")); SerialDebug.print(sendTo32u4Data.flywheel); SerialDebug.print('\t'); 
    SerialDebug.print(F("ControllerConn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
    SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
      
#endif

#ifdef debugMainDrive

    SerialDebug.print(F("In/Pitch: ")); SerialDebug.print(Input_Drive); SerialDebug.print('\t'); 
    SerialDebug.print(F("Set/Joy: ")); SerialDebug.print(Setpoint_Drive); SerialDebug.print('\t');
    SerialDebug.print(F("Out: ")); SerialDebug.print(Output_Drive); SerialDebug.print('\t');
    SerialDebug.print(F("ControllerConn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
    SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
      
#endif

     
     
  
}
