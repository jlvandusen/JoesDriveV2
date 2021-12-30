/*
 * Joe's Drive  - V2 01/2022
 * Secondary 32u4 Dome Controls and PSI Body lights
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * Utilizes Feather 32u4 Basic Proto From Adafruit: https://www.adafruit.com/product/2771
 * 
 * Libraries Required
 * Feather 32u4 Board Libraries: https://learn.adafruit.com/adafruit-feather-32u4-basic-proto/using-with-arduino-ide
 * Encoder for the ENC on Dome Spin: https://github.com/PaulStoffregen/Encoder
 * Adafruit VS1053 Featherwing MusicPlayer: https://github.com/adafruit/Adafruit_VS1053_Library
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 * Servo library - https://github.com/netlabtoolkit/VarSpeedServo
 * 
*/

/*
 * Debug Configurations
 * Comment and uncomment which function needs to be debugged
*/
//#define debugDomeAndFly
//#define printRemote
//#define debugVS
//#define debugPSI
//#define printServoPositions
//#define printPitchAndRoll
//#define debugServos
//#define debugEasyTransfer



/*  Controller types: Currently PS3 Move Controllers and PS3/4 Joystick supported
 *  Still to do: Xbox Controller, PS3,4 and 5 Controllers
*/

#define MOVECONTROLLER
//#define XBOXCONTROLLER   

/* Debug Printlines */
#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)
#define SerialDebug Serial  

/*
 * PIN DEFINITIONS
*/
#define motorEncoder_pin_A 18 //3
#define motorEncoder_pin_B 19 //2
#define domeMotor_pwm 21 //5
#define domeMotor_pin_A 22 //10
#define domeMotor_pin_B 23 //13
#define hallEffectSensor_Pin 20 //19
#define leftServo_pin 12
#define rightServo_pin 11
#define NeoPixel_pin 13

#define servoSpeed 500
#define servoEase 10
#define domeTiltYAxis_MaxAngle 20
#define domeTiltXAxis_MaxAngle 20
#define printMillis 5

#define leftServoOffset -7
#define rightServoOffset 0

#include <EasyTransfer.h>
#include <PID_v1.h>

EasyTransfer recESP32;
EasyTransfer sendESP32;

struct RECEIVE_DATA_STRUCTURE { 
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3;
  int8_t flywheel;
  bool moveR3;
  int8_t leftStickX;
  int8_t leftStickY;
  bool psiFlash;
  float pitch;
  float roll;
//  int8_t soundcmd; 
};
RECEIVE_DATA_STRUCTURE receiveFromESP32Data; //give a name to the group of data for recieving

struct SEND_DATA_STRUCTURE { 
  int16_t tiltAngle;
};
SEND_DATA_STRUCTURE sendToM0Data; //give a name to the group of data for sending

String readString;

int16_t leftServo_0_Position = 70 + leftServoOffset;
int16_t rightServo_0_Position = 110 + rightServoOffset;
double leftServoPosition = leftServo_0_Position;
double rightServoPosition = rightServo_0_Position;
double leftDifference, leftOldPosition = leftServo_0_Position, rightDifference, rightOldPosition = rightServo_0_Position;
double domeTiltAngle_X_Axis, domeTiltAngle_Y_Axis, leftStickY, leftStickX, encPos, batt_Voltage;
float R1 = 30000.0; //30k
float R2 = 7500.0; //7k5
float pitch, roll; 
  
bool domeCenterSet = false, domeServoMode = false, r3Flag = false, psiFlash = false, xboxR3Was = false, moveR3Was = false, moveL3Was = false, enableDrive; 

unsigned long currentMillis, receiveMillis, lastPrintMillis, lastVSMillis, lastServoUpdateMillis; 

int psiValue;

/* VarSpeedServo Library - Basic Example
 * https://github.com/netlabtoolkit/VarSpeedServo
 *
 * This example code is in the public domain.
 */
#include <VarSpeedServo.h> 
VarSpeedServo myservo1; // create servo object to control a left servo 
VarSpeedServo myservo2; // create servo object to control a right servo 

  
/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <Encoder.h>
Encoder myEnc(motorEncoder_pin_A, motorEncoder_pin_B);
  
long oldPosition  = -999;
double Setpoint_domeSpinServoPid, Input_domeSpinServoPid, Output_domeSpinServoPid;

//Specify the links and initial tuning parameters for MyPID
double Kp_domeSpinServoPid=4, Ki_domeSpinServoPid=0, Kd_domeSpinServoPid=0;
PID myPID_domeSpinServoPid(&Input_domeSpinServoPid, &Output_domeSpinServoPid, &Setpoint_domeSpinServoPid, Kp_domeSpinServoPid, Ki_domeSpinServoPid, Kd_domeSpinServoPid, DIRECT);
  
void setup(){
 
  myservo2.attach(leftServo_pin);
  myservo1.attach(rightServo_pin);

  myservo2.write(70 + leftServoOffset, 10);     
  myservo1.write(110 + rightServoOffset, 10);

  Serial.begin(115200);
  Serial1.begin(78440); // 74880 78440
  
  recESP32.begin(details(receiveFromESP32Data), &Serial1); 
  sendESP32.begin(details(sendToM0Data), &Serial1); 

  myPID_domeSpinServoPid.SetMode(AUTOMATIC);
  myPID_domeSpinServoPid.SetOutputLimits(-255, 255);

  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);
  pinMode(domeMotor_pwm, OUTPUT);  // Speed Of Motor1 on Motor Driver 1 
  pinMode(domeMotor_pin_A, OUTPUT);  // Direction
  pinMode(domeMotor_pin_B, OUTPUT);
//  pinMode(flyWheelMotor_pwm, OUTPUT);  // Speed of Motor1 on Motor Driver 2 
//  pinMode(flyWheelMotor_pin_A, OUTPUT);  // Direction
//  pinMode(flyWheelMotor_pin_B, OUTPUT);
}
  
void loop() {

  checkIncomingData(); 
//  encoder(); 
  Servos();
//  spinStuff();
//  setDomeCenter();
  Timechecks(); 
  debugRoutines();
}

void Timechecks() {
  currentMillis = millis();
  receiveMillis = currentMillis;
  if (currentMillis - lastPrintMillis >= printMillis) { // Check for last debug print, if past, print debugs if enabled.
    lastPrintMillis = currentMillis;
    debugRoutines();
  }
  if (currentMillis - lastVSMillis > 10000) { // Check for last voltage check of 10secs, then check again.
    lastVSMillis = currentMillis; 
//    readBatteryVoltage();
  }
  if (currentMillis - receiveMillis >= 250 && enableDrive) {  // Check for delays in recieving ESP32 data, if so disable the drives automatically.
    enableDrive = !enableDrive; 
  }
}

void checkIncomingData() {
  if (recESP32.receiveData()) {
    receiveMillis = currentMillis; 
    if (enableDrive == false) {
      enableDrive = !enableDrive; 
    }
    sendESP32.sendData(); 
//  } else enableDrive = !enableDrive;
  } else {
    if(currentMillis - receiveMillis >= 250 && enableDrive){
      enableDrive = !enableDrive; 
    }  
  }
}
 
void encoder() {
  encPos = myEnc.read(); 
  if (receiveFromESP32Data.moveR3 && moveR3Was == false) {
    moveR3Was = true;
    myEnc.write(encPos - 1497/2);
    
  } else if (receiveFromESP32Data.moveR3 == false && moveR3Was == true) {
    moveR3Was = false;
    myEnc.write(encPos + 1497/2);
  }
  if (encPos > 1497) {
    encPos -= 1497; 
    myEnc.write(encPos);
  } else if (encPos < 0) {
    encPos += 1497; 
    myEnc.write(encPos);
  }

  if (receiveFromESP32Data.moveL3 == 1 && r3Flag == false) {
    r3Flag = true; 
    domeServoMode = !domeServoMode; 
  } else if (receiveFromESP32Data.moveL3 == 0 && r3Flag == true) {
    r3Flag = false; 
  }
  
}
  
/*
 *   Turn the dome spin clockwise till it finds the hallsensor then set to domeCenterSet 1
 */
void setDomeCenter() {
  if (domeCenterSet == false) {
    analogWrite(domeMotor_pin_B,75);
    analogWrite(domeMotor_pin_A,0);
    if (digitalRead(hallEffectSensor_Pin) == 0) {
      domeCenterSet = true; 
      myEnc.write(749);
    }
  }
}

void spinStuff() {
  if(domeServoMode) {
    domeServoMovement();
  } else {
    spinDome();
  }
//  spinFlywheel();
}
    

void spinDome() {
  if (receiveFromESP32Data.domeSpin > 3 && receiveFromESP32Data.driveEnabled) {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    analogWrite(domeMotor_pwm,map(receiveFromESP32Data.domeSpin,3,100,0,255));
  } else if (receiveFromESP32Data.domeSpin < -3 && receiveFromESP32Data.driveEnabled) {
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,map(receiveFromESP32Data.domeSpin,-100,-3,255,0));
  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
}
      

void domeServoMovement() {
  if (receiveFromESP32Data.driveEnabled) {
      Setpoint_domeSpinServoPid = map(receiveFromESP32Data.domeSpin, -100, 100, 75, -75); 
    } else {
      Setpoint_domeSpinServoPid = 0; 
    }
  
  Input_domeSpinServoPid = map(encPos,0,1497,180, -180); 

  myPID_domeSpinServoPid.Compute(); 
    
  if (Output_domeSpinServoPid >2 && receiveFromESP32Data.driveEnabled) {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
  } else if (Output_domeSpinServoPid <-2 && receiveFromESP32Data.driveEnabled) {
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
}

//void spinFlywheel() {
//  if (receiveFromESP32Data.flywheel > 1 && receiveFromESP32Data.driveEnabled) {
//    digitalWrite(flyWheelMotor_pin_A, LOW);
//    digitalWrite(flyWheelMotor_pin_B, HIGH); // Motor 1 Forward
//    analogWrite(flyWheelMotor_pwm,map(receiveFromESP32Data.flywheel,1,100,0,255));
//  } else if (receiveFromESP32Data.flywheel < -1 && receiveFromESP32Data.driveEnabled) {
//    digitalWrite(flyWheelMotor_pin_A, HIGH);
//    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Backward
//    analogWrite(flyWheelMotor_pwm,map(receiveFromESP32Data.flywheel,-100,-1,255,0));
//  } else {
//    digitalWrite(flyWheelMotor_pin_A, LOW);
//    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Stopped
//  }
//}

//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}





  
