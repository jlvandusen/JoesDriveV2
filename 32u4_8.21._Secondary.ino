/*
 * Joe's Drive  - V2 8/7/2021
 * Secondary 32u4
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
//#define debugDomeAndFly
//#define printRemote
//#define debugVS
//#define debugPSI
//#define printServoPositions
//#define printPitchAndRoll
//#define debugServos



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
 * Music Controller
 * Used to enable or disable use of VS1053 Musicplayer Featherwing
*/

//#define MUSICPLAYER_FEATHERWING


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
//#define psiSensor_pin_L A0
//#define psiSensor_pin_R A2
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

#ifdef MUSICPLAYER_FEATHERWING
  #include <SD.h>
  #include <Adafruit_VS1053.h>
#endif

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
//  int8_t sound;
};
RECEIVE_DATA_STRUCTURE receiveFromESP32Data; //give a name to the group of data for recieving

struct SEND_DATA_STRUCTURE { 
  int16_t tiltAngle;
//  int8_t playingMusic;
//  int8_t sendPSI;
};
SEND_DATA_STRUCTURE sendToESP32Data; //give a name to the group of data for sending

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
int8_t playingMusic = 0;
int8_t sendPSI = 0;
int psiValue;


#include <VarSpeedServo.h> 
VarSpeedServo myservo1; // create servo object to control a left servo 
VarSpeedServo myservo2; // create servo object to control a right servo 

#ifdef MUSICPLAYER_FEATHERWING
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)
#define VS1053_CS      32     // VS1053 chip select pin (output)
#define VS1053_DCS     33     // VS1053 Data/command select pin (output)
#define CARDCS         14     // Card chip select pin
// DREQ should be an Int pin *if possible* (not possible on 32u4)
#define VS1053_DREQ    15     // VS1053 Data request, ideally an Interrupt pin
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);
#endif

#ifdef MUSICPLAYER_FEATHERWING
int numberOfTracks = 9; 
int i; 
char *soundNames[]={"ACK0000.ogg", "FUN0000.ogg",  "FUN0001.ogg",  "BEEP0000.ogg",  "ACK0001.ogg",  "BEEP0001.ogg",  "BEEP0002.ogg",  "BEEP0003.ogg",  "BEEP0004.ogg"};
#endif
  
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
  Serial1.begin(78440); 
  
  recESP32.begin(details(receiveFromESP32Data), &Serial1); 
  sendESP32.begin(details(sendToESP32Data), &Serial1); 

#ifdef MUSICPLAYER_FEATHERWING
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
  } else {
      Serial.println(F("VS1053 found"));
      musicPlayer.setVolume(1,1); // Set volume for left, right channels. lower numbers == louder volume!
//      musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
      musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
   }
  
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
  } else Serial.println("SD OK!");
#endif

  myPID_domeSpinServoPid.SetMode(AUTOMATIC);
  myPID_domeSpinServoPid.SetOutputLimits(-255, 255);

  pinMode(hallEffectSensor_Pin, INPUT_PULLUP);
  pinMode(domeMotor_pwm, OUTPUT);  // Speed Of Motor1 on Motor Driver 1 
  pinMode(domeMotor_pin_A, OUTPUT);  // Direction
  pinMode(domeMotor_pin_B, OUTPUT);
}
  
void loop() {

  checkIncomingData();
  #ifdef MUSICPLAYER_FEATHERWING
    sounds();
  #endif
  encoder(); 
  Servos();
  spinStuff();
  if(domeCenterSet == false) {
    setDomeCenter(); 
    }
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
//  if (currentMillis - lastVSMillis > 10000) { // Check for last voltage check of 10secs, then check again.
//    lastVSMillis = currentMillis; 
//    readBatteryVoltage();
//  }
  if (currentMillis - receiveMillis >= 250 && enableDrive) {  // Check for delays in recieving ESP32 data, if so disable the drives automatically.
    enableDrive = !enableDrive; 
  }
}

//void checkIncomingData() {
//  if (recESP32.receiveData()) {
//    if (enableDrive == false) {
//      enableDrive = !enableDrive; 
//    }
//    sendESP32.sendData(); 
//  } else enableDrive = !enableDrive;
//}

void checkIncomingData() {
  if(recESP32.receiveData()){
    receiveMillis = currentMillis; 
    if(enableDrive == false){
      enableDrive = !enableDrive; 
    }
    delay(5);
    sendESP32.sendData(); 
  }else if(currentMillis - receiveMillis >= 250 && enableDrive) {
    enableDrive = !enableDrive; 
  }
}

void Servos() {
//  Controls the tilt and triangular positioning of the dome mast
//  Based on James Bruton's excellent examples on triangulation and positioning using 2 BB sized Servos.
  int domeTurnPercent; 
  int y_Axis, x_Axis; 
  if(receiveFromESP32Data.driveEnabled) {
      if(domeServoMode){
        domeTurnPercent = map(Setpoint_domeSpinServoPid,-90,90,100,-100);
        if(domeTurnPercent>0){
          if(receiveFromESP32Data.leftStickY >= 0){
            y_Axis = map(receiveFromESP32Data.leftStickY,0,100,0,(100-domeTurnPercent));
            x_Axis = map(receiveFromESP32Data.leftStickY,0,100,0,domeTurnPercent); 
          }else if(receiveFromESP32Data.leftStickY<0){
            y_Axis = map(receiveFromESP32Data.leftStickY,-100,0,(-100+domeTurnPercent),0); 
            x_Axis = map(receiveFromESP32Data.leftStickY,-100,0,domeTurnPercent*-1,0); 
          }
    
          if(receiveFromESP32Data.leftStickX >= 0){
            y_Axis -= map(receiveFromESP32Data.leftStickX,0,100,0,domeTurnPercent); 
            x_Axis += map(receiveFromESP32Data.leftStickX,0,100,0,(100-domeTurnPercent)); 
          }else if(receiveFromESP32Data.leftStickX < 0){
            y_Axis += map(receiveFromESP32Data.leftStickX,-100,0,domeTurnPercent,0); 
            x_Axis += map(receiveFromESP32Data.leftStickX,-100,0,(-100+domeTurnPercent),0); 
          }
          
          }else if(domeTurnPercent == 0){
            y_Axis = receiveFromESP32Data.leftStickY; 
            x_Axis = receiveFromESP32Data.leftStickX; 
            
          }else if(domeTurnPercent< 0){
            if(receiveFromESP32Data.leftStickY >= 0){
              y_Axis = map(receiveFromESP32Data.leftStickY,0,100,0,(100+domeTurnPercent));
              x_Axis = map(receiveFromESP32Data.leftStickY,0,100,0,domeTurnPercent); 
            }else if(receiveFromESP32Data.leftStickY<0){
              y_Axis = map(receiveFromESP32Data.leftStickY,-100,0,(-100-domeTurnPercent),0); 
              x_Axis = map(receiveFromESP32Data.leftStickY,-100,0,domeTurnPercent*-1,0);
            }
    
            if(receiveFromESP32Data.leftStickX >= 0){
              y_Axis -= map(receiveFromESP32Data.leftStickX,0,100,0,domeTurnPercent); 
              x_Axis += map(receiveFromESP32Data.leftStickX,0,100,0,(100+domeTurnPercent)); 
            }else if(receiveFromESP32Data.leftStickX < 0){
              y_Axis += map(receiveFromESP32Data.leftStickX,-100,0,domeTurnPercent,0); 
              x_Axis -= map(receiveFromESP32Data.leftStickX,-100,0,(100+domeTurnPercent),0); 
            }
         } 
      }else{
        x_Axis = receiveFromESP32Data.leftStickX;
        y_Axis = receiveFromESP32Data.leftStickY;
      }
    
      if(receiveFromESP32Data.moveR3){
        y_Axis *= -1;
        x_Axis *= -1;
      }
      
      if(y_Axis < 0) { //Scales the Y values from -100/100 to 40deg toward the 'back' and 35deg toward the 'front'
        leftStickY = map(y_Axis,-100,0,-domeTiltYAxis_MaxAngle,0);
      } else if(y_Axis > 0) {
        leftStickY = map(y_Axis,0,100,0,domeTiltYAxis_MaxAngle);
      } else {
        leftStickY = 0;
      }
  
  
      if(x_Axis < 0){ //Scales the X values from -100/100 to 29def left/right
        leftStickX  = map(x_Axis,-100,0,-domeTiltXAxis_MaxAngle,0);
      }else if(x_Axis > 0){
        leftStickX  = map(x_Axis,0,100,0,domeTiltXAxis_MaxAngle);
      }else{
        leftStickX  = 0;
      }
  
      if(currentMillis - lastServoUpdateMillis >= 10) { //Eases Y values by adding/subtracting the ease values every 10 millis
        lastServoUpdateMillis = currentMillis; 
        if(leftStickY < domeTiltAngle_Y_Axis - servoEase) {
          domeTiltAngle_Y_Axis -= servoEase; 
        } else if(leftStickY > domeTiltAngle_Y_Axis + servoEase) {
          domeTiltAngle_Y_Axis += servoEase; 
        } else {
         domeTiltAngle_Y_Axis = leftStickY;
        }
  
        
        if(leftStickX < domeTiltAngle_X_Axis - servoEase) { //Eases X values by adding/subtracting the ease values every 10 millis
          domeTiltAngle_X_Axis -= servoEase; 
        } else if(leftStickX > domeTiltAngle_X_Axis + servoEase) {
          domeTiltAngle_X_Axis += servoEase; 
        } else {
         domeTiltAngle_X_Axis = leftStickX;
        }
      }
        
      if(domeTiltAngle_Y_Axis < 0) { //Turns the scaled angles from the y axis to servo positions
        leftServoPosition = leftServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch), -40,0,55,0);
        rightServoPosition = rightServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch), -40,0,-55,0);
        
      } else if(domeTiltAngle_Y_Axis > 0) {
        leftServoPosition = leftServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch, 0,35,0,-50);
        rightServoPosition = rightServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch, 0,35,0,50);
      } else {
        if(receiveFromESP32Data.pitch <= 0) {
          leftServoPosition = leftServo_0_Position - map(receiveFromESP32Data.pitch,-40,0,55,0);
          rightServoPosition = rightServo_0_Position + map(receiveFromESP32Data.pitch,-40,0,55,0);
        } else {
          leftServoPosition = leftServo_0_Position - map(receiveFromESP32Data.pitch,0,35,0,-50);
          rightServoPosition = rightServo_0_Position + map(receiveFromESP32Data.pitch,0,35,0,-50);
        } 
      }
  
        
      if(domeTiltAngle_X_Axis < 0) { //Turns the scaled angles from the x axis to servo positions and adds them to what we have from Y
        leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,30,0);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,50,0 );
        
      } else if(domeTiltAngle_X_Axis > 0) {
        leftServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-50)+map(receiveFromESP32Data.roll,0,29,0,-30));
        rightServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-30)+map(receiveFromESP32Data.roll,0,29,0,-50));
         
      } else {
        if(receiveFromESP32Data.roll <= 0) {
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,30,0);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,50,0 );
        } else {
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,0,29,0,-30);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,0,29,0,-50);
        } 
      }
  
  
      leftDifference = abs(leftOldPosition - leftServoPosition); 
      rightDifference = abs(rightOldPosition - rightServoPosition); 
      
      if(leftDifference > rightDifference){
        if(leftOldPosition < leftServoPosition){ 
          leftOldPosition ++; 
        }else if(leftOldPosition > leftServoPosition){
          leftOldPosition --; 
        }
  
        if(rightOldPosition < rightServoPosition){
          rightOldPosition += rightDifference / leftDifference;
        }else{
          rightOldPosition -= rightDifference / leftDifference;
        }
        
      }else {
         if(rightOldPosition < rightServoPosition) { 
          rightOldPosition ++; 
        } else if (rightOldPosition > rightServoPosition) {
          rightOldPosition --; 
        }
  
        if(leftOldPosition < leftServoPosition) {
          leftOldPosition += leftDifference / rightDifference;
        } else if (leftOldPosition > leftServoPosition) {
          leftOldPosition -= leftDifference / rightDifference;
        }  
      }
           
      myservo2.write(constrain(leftOldPosition,leftServo_0_Position-45,leftServo_0_Position+55), servoSpeed);      //Sends positions to the servos
      myservo1.write(constrain(rightOldPosition,rightServo_0_Position-55,rightServo_0_Position+45), servoSpeed);   //Sends positions to the servos
  }
}

/*
 * https://dronebotworkshop.com/rotary-encoders-arduino/
 * For more information on encoder usage for tracking dirction
 */
 
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
//    analogWrite(domeMotor_pin_B,75);
//    analogWrite(domeMotor_pin_A,0);
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Dome Motor Forward
    analogWrite(domeMotor_pwm,75);
    if (digitalRead(hallEffectSensor_Pin) == 0) {
      domeCenterSet = true; 
      myEnc.write(749);  // Set Dome Center Position (facing forward)
    }
  }
}

void spinStuff() {
  if(domeServoMode) {
    domeServoMovement();
  } else {
    spinDome();
  }
//  spinFlywheel();  // need to determine domeservomode over Easy Transfer now
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

void sounds() {
#ifdef MUSICPLAYER_FEATHERWING
  if (musicPlayer.playingMusic) {
    musicPlayer.feedBuffer();
    sendPSI = 1;
  } else sendPSI = 0;
  
  switch (receiveFromESP32Data.sound) {
    case 0:
      musicPlayer.stopPlaying();
      break;
    case 1:
      if(musicPlayer.stopped()){
        musicPlayer.startPlayingFile(soundNames[i]);
        i++;
        if(i >= numberOfTracks){
          i=0;
        }
      }
      break; 
    case 10:
      musicPlayer.startPlayingFile("/ACK0000.ogg");  //Play Acknowlege 0000
      break;
    case 11:
      musicPlayer.startPlayingFile("/ACK0001.ogg");  //Play Acknowlege 0001
      break;
    case 12:
      musicPlayer.startPlayingFile("/ACK0002.ogg");  //Play Acknowlege 0002
      break;
    case 13:
      musicPlayer.startPlayingFile("/ACK0003.ogg");  //Play Acknowlege 0003
      break;
    case 14:
      musicPlayer.startPlayingFile("/FUN0000.ogg");  //Play Fun 0000
      break;
    case 15:
      musicPlayer.startPlayingFile("/FUN0001.ogg");  //Play Fun 0001
      break;
    case 16:
      musicPlayer.startPlayingFile("/FUN0002.ogg");  //Play Fun 0002
      break;
    case 17:
      musicPlayer.startPlayingFile("/FUN0003.ogg");  //Play Fun 0003
      break;
    case 20:
      musicPlayer.startPlayingFile("/ACK0000.ogg");  //Play Acknowlege 0000
      break;
    case 21:
      musicPlayer.startPlayingFile("/ACK0001.ogg");  //Play Acknowlege 0001
      break;
    case 22:
      musicPlayer.startPlayingFile("/ACK0002.ogg");  //Play Acknowlege 0002
      break;
    case 23:
      musicPlayer.startPlayingFile("/ACK0003.ogg");  //Play Acknowlege 0003
      break;
    case 24:
      musicPlayer.startPlayingFile("/FUN0000.ogg");  //Play Fun 0000
      break;
    case 25:
      musicPlayer.startPlayingFile("/FUN0001.ogg");  //Play Fun 0001
      break;
    case 26:
      musicPlayer.startPlayingFile("/FUN0002.ogg");  //Play Fun 0002
      break;
    case 27:
      musicPlayer.startPlayingFile("/FUN0003.ogg");  //Play Fun 0003
      break;      
  }

  #ifdef debugSounds 
    SerialDebug.print("PSI State: "); SerialDebug.println(sendPSI); 
    SerialDebug.print("Playing Track: "); SerialDebug.println(musicPlayer.currentTrack); 
  #endif
  
  #endif

}

//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}


void debugRoutines(){

#ifdef printRemote
  Serial.print("\t"); Serial.print("DriveEnabled:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.driveEnabled);
  Serial.print("\t"); Serial.print(",DomeSpin:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.domeSpin);
  Serial.print("\t"); Serial.print(",MoveL3:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.moveL3);
  Serial.print("\t"); Serial.print(",Flywheel:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.flywheel);
  Serial.print("\t"); Serial.print(",MoveR3:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.moveR3);
  Serial.print("\t"); Serial.print(",leftStickX:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.leftStickX);
  Serial.print("\t"); Serial.print(",leftStickY:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.leftStickY);
  Serial.print("\t"); Serial.print(",psiFlash:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.psiFlash);
  Serial.print("\t"); Serial.print(",pitch:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.pitch);
  Serial.print("\t"); Serial.print(",roll:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.roll);
  Serial.print("\t"); Serial.print(",sound:" );
  Serial.print("\t"); Serial.print(receiveFromESP32Data.sound);
  Serial.println();
#endif

#ifdef debugDomeAndFly
  Serial.print("EN: "); Serial.print(receiveFromESP32Data.driveEnabled);
  if(receiveFromESP32Data.xboxL1 == 0 && receiveFromESP32Data.xboxR1 == 0){
    if(domeServoMode){
      Serial.print(" ServoMode: In: "); Serial.print(Input_domeSpinServoPid);
      Serial.print("  Set: "); Serial.print(Setpoint_domeSpinServoPid); 
      Serial.println("  Out: "); Serial.println(Output_domeSpinServoPid); 
    } else {
      Serial.print(" Spin L: "); Serial.print(receiveFromESP32Data.xboxL2); 
      Serial.println("R: "); Serial.println(receiveFromESP32Data.xboxR2); 
    }
  } else {
    Serial.print(" Fly L: "); Serial.print(receiveFromESP32Data.xboxL2); 
    Serial.println("R: "); Serial.println(receiveFromESP32Data.xboxR2); 
  }
#endif


//#ifdef debugPSI
//  Serial.print("L: "); Serial.print(analogRead(psiSensor_pin_L));
//  Serial.println("    R: "); Serial.println(analogRead(psiSensor_pin_R));
//#endif

#ifdef printPitchAndRoll
  Serial.print("Pitch: "); Serial.print(receiveFromESP32Data.pitch); 
  Serial.print(" actual pitch: "); Serial.print(pitch); 
  Serial.print("    Roll: "); Serial.println(receiveFromESP32Data.roll); 
  Serial.print(" Pitch/LeftServo: ");
  Serial.print(receiveFromESP32Data.pitch);  Serial.print("  "); 
  Serial.println(constrain(leftOldPosition,leftServo_0_Position-45,leftServo_0_Position+55));
#endif

#ifdef printServoPositions
  Serial.print(leftServoPosition); 
  Serial.println("\t");Serial.println(rightServoPosition);  
#endif

#ifdef debugServos
  Serial.print("leftStickX  "); Serial.print(leftStickX); 
  Serial.print("  leftStickY  "); Serial.print(leftStickY); 
  Serial.print("  domeTiltAngle_X_Axis  "); Serial.print(leftStickY); 
  Serial.print("  domeTiltAngle_Y_Axis  "); Serial.print(domeTiltAngle_Y_Axis);
  Serial.print("  leftOldPosition  "); Serial.print(leftOldPosition);
  Serial.println("  rightOldPosition  "); Serial.println(rightOldPosition);  
#endif
  
}
