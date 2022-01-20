/*
 * Joe's Drive  - V2 1/2022
 * Primary CPU uses ESP32 HUZZAH32
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need ESP32 Hardware: 
 * ESP32 HUZZAH From Adafruit: https://www.adafruit.com/product/3619 used for Dome and Primary Feather
 * ESp32 Board Libraries: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts?view=all#using-with-arduino-ide
 * 
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * On Line 117 - replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 * Modified ESP32 BT Host Library https://github.com/reeltwo/PSController
 * 
 * Libraries Required
 * https://github.com/netlabtoolkit/VarSpeedServo - Servo Controls for Feather (Non ESP32)
 * https://github.com/PaulStoffregen/Encoder - Encoder for the Dome Spin motor
 * https://github.com/reeltwo/PSController - Modified USBhost Controller for PS3 Move Nav/Xbox Controllers
 * https://github.com/ERROPiX/ESP32_AnalogWrite - Used for Proper PWM with ESP32 devices
 * https://github.com/br3ttb/Arduino-PID-Library/ - PID Library Official
 * http://www.billporter.info/easytransfer-arduino-library/ - Easy Transfer
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
//#define debugFlywheel
//#define debugS2S
//#define debugSounds
#define debugEasyTransfer

#define MP3SOUNDS // Enable qwiic/i2c communications to MP3 trigger
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

// Reverse Drives based on pin connections (reverse Polartity)
#define revS2S
//#define revDrive
#define revGyro

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

  #include <PSController.h>
  #define DRIVE_CONTROLLER_MAC  nullptr
  #define DOME_CONTROLLER_MAC nullptr
#endif

#include <Arduino.h>
#include <EasyTransfer.h> // http://www.billporter.info/easytransfer-arduino-library/
#include <Wire.h>
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/
#include "wiring_private.h" // pinPeripheral() function
#include <analogWrite.h>  // https://www.arduinolibraries.info/libraries/esp32-analog-write
  
#define BUFFER_LENGTH 128 // 64
#define TWI_BUFFER_LENGTH 128 //64
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
#ifdef XBOXCONTROLLER
struct SEND_DATA_STRUCTURE_32u4{  
  bool driveEnabled;
  int8_t domeSpin;
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
};
#endif

#ifdef MOVECONTROLLER
struct SEND_DATA_STRUCTURE_32u4{  
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3; // xbox L3 equivilent
  bool moveR3; // xboxR3 equivilent
  int8_t leftStickX;
  int8_t leftStickY;
  int8_t soundcmd;
  int8_t psiFlash;
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

#ifdef XBOXCONTROLLER
struct controllerButtons{
  bool a,b,x,y,up,down,left,right,xbox,back,l1,r1,start;
  int8_t rightStickX,rightStickY,l2,r2;
};
controllerButtons buttons;
#endif

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

#define SERIAL2_BAUD_RATE 57600  // 74880 57600 78440
#define SERIAL2_RX_PIN 13
#define SERIAL2_TX_PIN 12

/*
 * Create Serial 3 to send to debug
*/

#include <SoftwareSerial.h>
SoftwareSerial Serial3;
#define SERIAL3_BAUD_RATE 115200
#define SERIAL3_RX_PIN 22 //SCL - General purpose IO pin #22
#define SERIAL3_TX_PIN 23 //SDA - General purpose IO pin #23

unsigned long currentMillis, IMUmillis, lastLoopMillis, lastDomeMillis, rec32u4Millis, lastPrintMillis; 
bool IMUconnected, controllerConnected, Send_Rec, feather2Connected, drivecontrollerConnected, domecontrollerConnected, DomeServoMode, enableDrive, reverseDrive, enabledDrive;


//int pot_S2S;   // target position/inout

int current_pos_drive;  // variables for smoothing main drive
int target_pos_drive;
//int pot_drive;   // target position/inout
int diff_drive; // difference of position
double easing_drive;

float IMUDeadzone = 0;  //3
int S2S_potDeadzone = 3; // 3
int current_pos_S2S;  // variables for smoothing S2S
int target_pos_S2S;
int S2S_pot;
int diff_S2S; // difference of position
double easing_S2S;

int Output_flywheel_pwm; // variables for PWM controls of flywheel
int Output_domeSpin_pwm;
int8_t flywheel;

double Pk1 = 13;  //13
double Ik1 = 0;   //0
double Dk1 = 0;   //0
double Setpoint1, Input1, Output1, Output1_S2S_pwm;    // PID variables - S2S Joystick control
PID PID1_S2S(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - S2S Drive

double Pk2 =10; // 10
double Ik2 = 0; //0
double Dk2 = .01; //0 .03
double Setpoint2, Input2, Output2, Output2_S2S_pwm;    // PID variables - S2S stability
PID PID2_S2S(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - S2S stability

double Pk3 = 13; // 4
double Ik3 = 0;
double Dk3 = 0;
double Setpoint3, Input3, Output3, Output_Drive_pwm;    // PID variables - Main drive motor
PID PID_Drive(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // Main drive motor

void setup() {
  
  delay(5000); // Wait for Boot
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

  #ifdef MP3SOUNDS

  #endif

  recIMU.begin(details(receiveIMUData), &Serial1); 
  rec32u4.begin(details(receiveFrom32u4Data), &Serial2);
  send32u4.begin(details(sendTo32u4Data), &Serial2);

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
    if(currentMillis - lastPrintMillis >= 70) {
      lastPrintMillis = currentMillis;
      debugRoutines();
    }
  }
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
      Serial.println("Checking drive controller battery charge");
//      Serial.println(driveController.state.status.battery);
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
      Serial.println("Checking dome controller battery charge");
//      Serial.println(domeController.state.status.battery);
      batterycheck();
    }
  }
}
