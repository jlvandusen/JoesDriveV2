/*
 * Joe's Drive  - V2 01/2022
 * Secondary 32u4 Dome Controls and PSI Body lights v7.1 PCB Board
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
//#define debugDOME
#define debugEasyTransfer
//#define debugHALLFull
//#define debugHALL
//#define debugENC
//#define debugSounds

/*  Controller types: Currently PS3 Move Controllers and PS3/4 Joystick supported
 *  Still to do: Xbox Controller, PS3,4 and 5 Controllers
*/

#define MOVECONTROLLER
//#define XBOXCONTROLLER   

/*  MP3 Trigger types supported
 *  Still to do: VS105 and Zio
*/
#define MP3Sparkfun // Enable qwiic/i2c communications to MP3 trigger for Sparkfun
//#define MP3Zio // Enable qwiic/i2c communications to MP3 trigger for Zio
//#define MP3VS105 // Enable qwiic/i2c communications to MP3 trigger for Adafruit Feather VS105

/* Debug Printlines */
#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)
#define SerialDebug Serial  

/*
 * PIN DEFINITIONS
*/
#define motorEncoder_pin_A 14 //3 18 - 32u4 RF, 14 - 32u4 Proto M0 Feather M0
#define motorEncoder_pin_B 15 //2 19 - 32u4 RF, 15 - 32u4 Proto M0 Feather M0
#define domeMotor_pwm 10  // 10 - M0 Proto | 21 - 32u4
#define domeMotor_pin_A 9 //  9 - M0 Proto | 22 - 32u4
#define domeMotor_pin_B 6 //  6 - M0 Proto | 23 - 32u4
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

#ifdef MP3Zio
#include <Wire.h>
//These are the commands we can send
#define COMMAND_STOP 0x00
#define COMMAND_PLAY_TRACK 0x01 //Play a given track number like on a CD: regardless of file names plays 2nd file in dir.
#define COMMAND_PLAY_FILENUMBER 0x02 //Play a file # from the root directory: 3 will play F003xxx.mp3
#define COMMAND_PAUSE 0x03 //Will pause if playing, or starting playing if paused
#define COMMAND_PLAY_NEXT 0x04
#define COMMAND_PLAY_PREVIOUS 0x05
#define COMMAND_SET_EQ 0x06
#define COMMAND_SET_VOLUME 0x07
#define COMMAND_GET_SONG_COUNT 0x08 //Note: This causes song to stop playing
#define COMMAND_GET_SONG_NAME 0x09 //Fill global array with 8 characters of the song name
#define COMMAND_GET_PLAY_STATUS 0x0A
#define COMMAND_GET_CARD_STATUS 0x0B
#define COMMAND_GET_VERSION 0x0C
#define COMMAND_SET_ADDRESS 0xC7
  byte mp3Address = 0x37; // default address for Qwiic MP3
  byte adjustableNumber = 1;
  int randomsound = random(1,55);
  int8_t sound;
#endif

#ifdef MP3Sparkfun
  #include <Wire.h>
  #include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h" // http://librarymanager/All#SparkFun_MP3_Trigger
  MP3TRIGGER mp3;
  byte mp3Address = 0x37; // default address for Qwiic MP3
  byte adjustableNumber = 1;
  int randomsound = random(1,55);
  int8_t sound;
#endif

#include <EasyTransfer.h>
#include <PID_v1.h>

EasyTransfer recESP32;
EasyTransfer sendESP32;

struct RECEIVE_DATA_STRUCTURE { 
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
float R2 = 7500.0; //7.5k5 or 7k5
float pitch, roll; 
  
bool domeCenterSet = false, domeServoMode = false, r3Flag = false, psiFlash = false, xboxR3Was = false, moveR3Was = false, moveL3Was = false, enableDrive; 

unsigned long currentMillis, receiveMillis, lastPrintMillis, lastVSMillis, lastServoUpdateMillis; 
int8_t soundcmd = 0; 
int psiValue, domeServoPWM;

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
long newPosition;

double Setpoint_domeSpinServoPid, Input_domeSpinServoPid, Output_domeSpinServoPid;

//Specify the links and initial tuning parameters for MyPID
double Kp_domeSpinServoPid=4, Ki_domeSpinServoPid=0, Kd_domeSpinServoPid=0;
PID myPID_domeSpinServoPid(&Input_domeSpinServoPid, &Output_domeSpinServoPid, &Setpoint_domeSpinServoPid, Kp_domeSpinServoPid, Ki_domeSpinServoPid, Kd_domeSpinServoPid, DIRECT);
  
void setup(){
#ifdef MP3Sparkfun
  Wire.begin();
  //Check to see if MP3 is present
  if(mp3.begin() == false)
  {
    Serial.println("Qwiic/i2c MP3 failed to respond. Please check wiring and possibly the I2C address. Freezing...");
    while(1);
  }

  if(mp3.hasCard() == false)
  {
    Serial.println("SD card missing. Freezing...");
    while(1);
  }

  mp3.setVolume(10); //Volume can be 0 (off) to 31 (max)

  Serial.print("Song count: ");
  Serial.println(mp3.getSongCount());

  Serial.print("Firmware version: ");
  Serial.println(mp3.getVersion());
#endif

  myservo2.attach(leftServo_pin);
  myservo1.attach(rightServo_pin);

  myservo2.write(70 + leftServoOffset, 10);     
  myservo1.write(110 + rightServoOffset, 10);

  Serial.begin(115200);
  Serial1.begin(57600); // 74880 78440 57600
  
  recESP32.begin(details(receiveFromESP32Data), &Serial1); 
  sendESP32.begin(details(sendToM0Data), &Serial1); 

  myPID_domeSpinServoPid.SetMode(AUTOMATIC);
  myPID_domeSpinServoPid.SetOutputLimits(-255, 255);

  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);
  pinMode(domeMotor_pwm, OUTPUT);  // Speed Of Motor1 on Motor Driver 1 
  pinMode(domeMotor_pin_A, OUTPUT);  // Direction
  pinMode(domeMotor_pin_B, OUTPUT);
  setDomeCenter();
}
  
void loop() {
  SendRecieveData(); 
  encoder();
  Servos();
//  spinStuff();
  spinDome();
  setDomeCenter();
//  if(domeCenterSet == false){
//    setDomeCenter(); 
//   }
//  domeServoMovement();
  Timechecks(); 
  debugRoutines();
  mp3play();
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

void SendRecieveData() {
  if (recESP32.receiveData()) {
    receiveMillis = currentMillis; 
    enableDrive = receiveFromESP32Data.driveEnabled;
    domeServoMode = receiveFromESP32Data.moveR3;
    sendESP32.sendData(); 
    if(currentMillis - receiveMillis >= 300 && enableDrive){
      enableDrive = !enableDrive; 
    }  
  }
}

void spinStuff() {
  if(!domeServoMode) {
    domeServoMovement();
  } else {
    spinDome();
  }
//  spinFlywheel();
}
    
//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}





  
