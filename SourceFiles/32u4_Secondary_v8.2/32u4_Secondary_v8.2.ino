/*
 * Joe's Drive  - V8.2 05/2024
 * Secondary 32u4 Dome movement and PSI Body lights v8.2 PCB Board
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * Utilizes Adafruit Feather 32u4 Basic Proto: 
 * https://www.adafruit.com/product/2771
 * https://learn.adafruit.com/adafruit-feather-32u4-basic-proto/pinouts
 * Or Feather 32u4 Feather M0 Basic Proto
 * https://www.adafruit.com/product/2772
 * https://learn.adafruit.com/adafruit-feather-m0-basic-proto/pinouts
 * 
 * Libraries Required
 * Feather 32u4 Board Libraries: https://learn.adafruit.com/adafruit-feather-32u4-basic-proto/using-with-arduino-ide
 * Encoder for the ENC on Dome Spin: https://github.com/PaulStoffregen/Encoder
 * Adafruit VS1053 Featherwing MusicPlayer: https://github.com/adafruit/Adafruit_VS1053_Library
 * Servo library - https://github.com/netlabtoolkit/VarSpeedServo
 * Sparkfun MP3 qwiic/i2c Trigger -  https://github.com/sparkfun/SparkFun_Qwiic_MP3_Trigger_Arduino_Library 
 * DF Robots DFPlayer Mini (SKU:DFR0299) Onboard MP3 trigger - https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299
 * Utilizes ESP32NOW technology over WiFi to talk between Dome and Body - need to capture the Wifi MAC during bootup.
 * replace this with the mac of dome uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
 * 
*/

/*
 * Debug Configurations
 * Comment and uncomment which function needs to be debugged
*/
// #define debugDomeAndFly
// #define printRemote
// #define debugVS
// #define debugPSI
// #define printServoPositions
// #define printPitchAndRoll
// #define debugServos
// #define debugDOME
// #define debugEasyTransfer
// #define debugHALLFull
// #define debugHALL
// #define debugENC
// #define debugSounds

/*  Controller types: Currently PS3 Move Controllers and PS3/4 Joystick supported
 *  Still to do: Xbox Controller, PS3,4 and 5 Controllers
*/

#define MOVECONTROLLER
//#define XBOXCONTROLLER

/*  MP3 Trigger types supported
 *  Still to do: VS105 and Zio
*/
// #define NoMP3 // Dont want to use MP3 services on this board
#define MP3Sparkfun  // Enable qwiic/i2c communications to MP3 trigger for Sparkfun
// #define MP3Zio // Enable qwiic/i2c communications to MP3 trigger for Zio
// #define MP3VS105 // Enable qwiic/i2c communications to MP3 trigger for Adafruit Featherwing VS105
// #define MP3DFPlayer // Enable onboard use of the DF Player Mini from DF Robot

#define UseHallMonitor  // Allow use of hall monitor installed to set forward direction of the dome otherwise set via Pref save on Controllers
// #define EnableFilters // Providing filtering against raw data reads from ESP32 over serial UNDER_CONSTRUCTION
// #define UseNEO // If not using RX capabilities from DFplayer can use for NEO Pixel controls of body UNDER_CONSTRUCTION

/* Debug Printlines */
#define SerialDebug Serial
#define DEBUG_PRINTLN(s) SerialDebug.println(s)
#define DEBUG_PRINT(s) SerialDebug.print(s)


/*
 * PIN DEFINITIONS
*/
#define motorEncoder_pin_A 18    //18 A0 - 32u4 Basic Proto/32u4 RF, 14 - 32u4 Proto M0 Feather M0
#define motorEncoder_pin_B 19    //19 A1 - 32u4 Basic Proto/32u4 RF, 15 - 32u4 Proto M0 Feather M0
#define domeMotor_pwm 10         // 10 - 32u4 Basic Proto/M0 Proto/32u4 RF
#define domeMotor_pin_A 9        //  9 - 32u4 Basic Proto/M0 Proto/32u4 RF
#define domeMotor_pin_B 6        //  6 - 32u4 Basic Proto/M0 Proto/32u4 RF
#define hallEffectSensor_Pin 20  //19
#define leftServo_pin 12
#define rightServo_pin 11
#define NeoPixel_pin 13

#define servoSpeed 500
#define servoEase 5  // 10
#define domeTiltYAxis_MaxAngle 25 // 20
#define domeTiltXAxis_MaxAngle 25 // 20
#define printMillis 5

#define leftServoOffset -7
#define rightServoOffset 0
#define PIN_MP3_TX 5  // Connects to 32u4's 5
#define PIN_MP3_RX 13  // Connects to 32u4's A4 22 or 13 (NEO)

#ifdef EnableFilters
  #include <Ewma.h>
  Ewma adcFiltermoveR3(0.01);  // More smoothing - less prone to noise, but slower to detect changes
  Ewma adcFiltermoveL3(0.01);  // More smoothing - less prone to noise, but slower to detect changes
  Ewma adcFilterdriveEnabled(0.01);  // More smoothing - less prone to noise, but slower to detect changes
  Ewma adcFilterdomeSpin(0.01);  // More smoothing - less prone to noise, but slower to detect changes
  Ewma adcFiltersoundcmd(0.01);  // More smoothing - less prone to noise, but slower to detect changes
#endif

#ifdef MP3Sparkfun
#include <Wire.h>
#include "SparkFun_Qwiic_MP3_Trigger_Arduino_Library.h"  // http://librarymanager/All#SparkFun_MP3_Trigger
#include "Arduino.h"
MP3TRIGGER mp3;
byte mp3Address = 0x37;  // default address for Qwiic MP3
byte adjustableNumber = 1;
int randomsound = random(1, 55);
int8_t sound;
#endif

#ifdef MP3DFPlayer
  #include "Arduino.h"
  #include "DFRobotDFPlayerMini.h"
  #include <SoftwareSerial.h>
  SoftwareSerial softSerial(/*rx =*/PIN_MP3_TX, /*tx =*/PIN_MP3_RX);
  #define FPSerial softSerial
  DFRobotDFPlayerMini myDFPlayer;
  byte adjustableNumber = 1;
  int randomsound = random(1, 55);
  int8_t sound;
#endif

#ifdef MP3VS105

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

#define VS1053_RESET -1  // VS1053 reset pin (not used!)
#define VS1053_CS 6      // VS1053 chip select pin (output)
#define VS1053_DCS 10    // VS1053 Data/command select pin (output)
#define CARDCS 5         // Card chip select pin
// DREQ should be an Int pin *if possible* (not possible on 32u4)
#define VS1053_DREQ 9  // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer =
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);
int randomsound = random(1, 55);
int8_t sound;
String FileName;
#endif

#include <EasyTransfer.h>
#include <PID_v1.h>

EasyTransfer recESP32;
EasyTransfer sendESP32;

struct RECEIVE_DATA_STRUCTURE {
  bool driveEnabled;
  int8_t domeSpin;
  bool moveL3;  // xbox L3 equivilent
  bool moveR3;  // xboxR3 equivilent
  int8_t leftStickX;
  int8_t leftStickY;
  int8_t soundcmd;
  int8_t psiFlash;
  float pitch;
  float roll;
} receiveFromESP32Data;  //give a name to the group of data for recieving

struct SEND_DATA_STRUCTURE {
  int16_t tiltAngle;
  bool sndplaying;
} sendToESP32Data;  //give a name to the group of data for sending

String readString;
int domeposition;
int16_t leftServo_0_Position = 70 + leftServoOffset;
int16_t rightServo_0_Position = 110 + rightServoOffset;
double leftServoPosition = leftServo_0_Position;
double rightServoPosition = rightServo_0_Position;
double leftDifference, leftOldPosition = leftServo_0_Position, rightDifference, rightOldPosition = rightServo_0_Position;
double domeTiltAngle_X_Axis, domeTiltAngle_Y_Axis, leftStickY, leftStickX, encPos, batt_Voltage;
float R1 = 30000.0;  //30k
float R2 = 7500.0;   //7.5k5 or 7k5
float pitch, roll;

bool domeCenterSet = false, domeServoMode = false, r3Flag = false, psiFlash = false, xboxR3Was = false, moveR3Was = false, moveL3Was = false, enableDrive = false, reverseDrive = false;
bool domeServoModeFiltered = false,enableDriveFiltered = false, reverseDriveFiltered = false;

unsigned long currentMillis, receiveMillis, lastPrintMillis, lastVSMillis, lastServoUpdateMillis;
int8_t soundcmd = 0, soundcmdFiltered = 0;
bool sndplaying = false;
int psiValue, domeServoPWM, domeServoPWMFiltered;

/* VarSpeedServo Library - Basic Example
 * https://github.com/netlabtoolkit/VarSpeedServo
 *
 * This example code is in the public domain.
 */
#include <VarSpeedServo.h>
VarSpeedServo myservo1;  // create servo object to control a left servo
VarSpeedServo myservo2;  // create servo object to control a right servo


/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
// #define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
Encoder myEnc(motorEncoder_pin_A, motorEncoder_pin_B);
long oldPosition = -999;
long newPosition;
int encoderPos = 0;
int aLastState = LOW;
int domePrevPWM;
double Setpoint_domeSpinServoPid, Input_domeSpinServoPid, Output_domeSpinServoPid;
// int domecenter;



//Specify the links and initial tuning parameters for MyPID
double Kp_domeSpinServoPid = 4, Ki_domeSpinServoPid = 0, Kd_domeSpinServoPid = 0;
PID myPID_domeSpinServoPid(&Input_domeSpinServoPid, &Output_domeSpinServoPid, &Setpoint_domeSpinServoPid, Kp_domeSpinServoPid, Ki_domeSpinServoPid, Kd_domeSpinServoPid, DIRECT);

void setup() {
  Serial.begin(115200);
  Serial1.begin(74880);  // 74880 78440 57600

#ifdef MP3DFPlayer
  FPSerial.begin(9600);
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  delay(3000);
  if (myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true)) {  //Use serial to communicate with mp3.
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.volume(20);  //Set volume value. From 0 to 30
    myDFPlayer.play(1);     //Play the first mp3
  } else if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
#endif

#ifdef MP3Sparkfun
  delay(5000);
  Wire.begin();
  //Check to see if MP3 is present
  if (mp3.begin() == false) {
    Serial.println("Qwiic/i2c MP3 failed to respond. Please check wiring and possibly the I2C address. Freezing...");
    while (1)
      ;
  } else {
    Serial.println("Qwiic/i2c MP3 found");
  }

  // if(mp3.hasCard() == false)
  // {
  //   Serial.println("SD card missing. Freezing...");
  //   while(1);
  // }

  mp3.setVolume(25);  //Volume can be 0 (off) to 31 (max)

  Serial.print("Song count: ");
  Serial.println(mp3.getSongCount());

  Serial.print("Firmware version: ");
  Serial.println(mp3.getVersion());
#endif

#ifdef MP3VS105

  if (!musicPlayer.begin()) {  // initialise the music player
    Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    while (1)
      ;
  }

  Serial.println(F("VS1053 found"));

  musicPlayer.sineTest(0x44, 500);  // Make a tone to indicate VS1053 is working

  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1)
      ;  // don't do anything more
  }
  Serial.println("SD OK!");

  // list files
  printDirectory(SD.open("/"), 0);

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(10, 10);

  // Timer interrupts are not suggested, better to use DREQ interrupt!
  // but we don't have them on the 32u4 feather...
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);  // timer int

// If DREQ is on an interrupt pin we can do background
// audio playing
//  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
#endif

  myservo2.attach(leftServo_pin);
  myservo1.attach(rightServo_pin);

  myservo2.write(70 + leftServoOffset, 10);
  myservo1.write(110 + rightServoOffset, 10);


  recESP32.begin(details(receiveFromESP32Data), &Serial1);
  sendESP32.begin(details(sendToESP32Data), &Serial1);
  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);
  
  myPID_domeSpinServoPid.SetMode(AUTOMATIC);
  myPID_domeSpinServoPid.SetOutputLimits(-255, 255);


  pinMode(domeMotor_pwm, OUTPUT);    // Speed Of Motor1 on Motor Driver 1
  pinMode(domeMotor_pin_A, OUTPUT);  // Direction
  pinMode(domeMotor_pin_B, OUTPUT);
  #ifndef UseHallMonitor
    domeCenterSet = true; 
    myEnc.write(740); //740
  #endif
}

void loop() {
  Timechecks();
  SendRecieveData();
  if (enableDrive == 1) {
    encoder();
    Servos();
    spinStuff();
    if(!domeCenterSet){
      setDomeCenter(); 
    }
  }

  Timechecks();
  mp3play();
}

