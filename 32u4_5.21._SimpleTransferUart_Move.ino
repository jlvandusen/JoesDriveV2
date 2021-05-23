
#define DEBUG_PRINTLN(s) Serial.println(s)
#define DEBUG_PRINT(s) Serial.print(s)

//#define debugDomeAndFly
//#define printRemote
//#define debugVS
//#define debugPSI
//#define printServoPositions
//#define printPitchAndRoll
//#define debugServos
//#define MOVECONTROLLER
#define MOVECONTROLLER_ESP32
//#define XBOXCONTROLLER   
  
/*
  PIN DEFINITIONS
*/
#define motorEncoder_A_Pin 3
#define motorEncoder_B_Pin 2
#define domeMotor_A_Pin 10
#define domeMotor_B_Pin 13
#define flyWheelMotor_A_Pin 9 
#define flyWheelMotor_B_Pin 6
#define hallEffectSensor_Pin 19
#define voltageSensor_Pin A4
//#define S2SPot_pin A0
//#define bodyNeoPixels_Pin A4
#define leftServoPin 12
#define rightServoPin 11
#define psiSensor_Pin_L A0
#define psiSensor_Pin_R A2

#define servoSpeed 500
#define servoEase 10
#define domeTiltYAxis_MaxAngle 20
#define domeTiltXAxis_MaxAngle 20
#define printMillis 5

#define leftServoOffset -7
#define rightServoOffset 0

#include <EasyTransfer.h>
#include <PID_v1.h>

 
EasyTransfer recM0;
EasyTransfer sendM0;

#ifndef MOVECONTROLLER_ESP32
struct RECEIVE_DATA_STRUCTURE{ 
  bool driveEnabled;
  int8_t domeSpin;
  bool xboxL3;
  int8_t flywheel;
  bool xboxR3;
  int8_t leftStickX;
  int8_t leftStickY;
  bool psiFlash;
  float pitch;
  float roll;  
};
#endif

#ifdef MOVECONTROLLER_ESP32
struct RECEIVE_DATA_STRUCTURE{ 
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
};
#endif


//give a name to the group of data
RECEIVE_DATA_STRUCTURE receiveFromM0Data;

struct SEND_DATA_STRUCTURE{ 
  int16_t tiltAngle;
};
//give a name to the group of data
SEND_DATA_STRUCTURE sendToM0Data;

String readString;

int16_t leftServo_0_Position = 70 + leftServoOffset;
int16_t rightServo_0_Position = 110 + rightServoOffset;
double leftServoPosition = leftServo_0_Position;
double rightServoPosition = rightServo_0_Position;
double leftDifference, leftOldPosition = leftServo_0_Position, rightDifference, rightOldPosition = rightServo_0_Position;
double domeTiltAngle_X_Axis, domeTiltAngle_Y_Axis, leftStickY, leftStickX;
float pitch, roll; 
  

bool domeCenterSet = false, domeServoMode = false, r3Flag = false, psiFlash = false, xboxR3Was = false; 
#ifdef MOVECONTROLLER_ESP32
bool moveR3Was = false, moveL3Was = false;
#endif

unsigned long currentMillis, receiveMillis, lastPrintMillis, lastVSMillis, lastServoUpdateMillis; 

bool enableDrive;

int psiValue;

double encPos; 

double batt_Voltage; 
float R1 = 30000.0; //30k
float R2 = 7500.0; //7k5

#include <VarSpeedServo.h> 

// create servo object to control a servo 
VarSpeedServo myservo1;
VarSpeedServo myservo2;

  
  /* Encoder Library - Basic Example
   * http://www.pjrc.com/teensy/td_libs_Encoder.html
   *
   * This example code is in the public domain.
   */
  #include <Encoder.h>
  
  Encoder myEnc(motorEncoder_A_Pin, motorEncoder_B_Pin);
  
  long oldPosition  = -999;
  double Setpoint_domeSpinServoPid, Input_domeSpinServoPid, Output_domeSpinServoPid;
  
  //Specify the links and initial tuning parameters
  double Kp_domeSpinServoPid=4, Ki_domeSpinServoPid=0, Kd_domeSpinServoPid=0;
  PID myPID_domeSpinServoPid(&Input_domeSpinServoPid, &Output_domeSpinServoPid, &Setpoint_domeSpinServoPid, Kp_domeSpinServoPid, Ki_domeSpinServoPid, Kd_domeSpinServoPid, DIRECT);

  
  void setup(){
   
    myservo2.attach(leftServoPin);
    myservo1.attach(rightServoPin);

    myservo2.write(70 + leftServoOffset, 10);     
    myservo1.write(110 + rightServoOffset, 10);

    Serial.begin(115200);
    Serial1.begin(78440); 
    
    recM0.begin(details(receiveFromM0Data), &Serial1); 
    sendM0.begin(details(sendToM0Data), &Serial1); 
    pinMode(hallEffectSensor_Pin, INPUT_PULLUP);

    myPID_domeSpinServoPid.SetMode(AUTOMATIC);
    myPID_domeSpinServoPid.SetOutputLimits(-255, 255);

     
  }
  
  void loop() {
    currentMillis = millis(); 

    checkIncomingData(); 
    encoder(); 
    Servos();
    spinStuff();

     Serial.print(receiveFromM0Data.pitch);  Serial.print("  "); 

        Serial.println(constrain(leftOldPosition,leftServo_0_Position-45,leftServo_0_Position+55));

    if(currentMillis - lastPrintMillis >= printMillis){
      lastPrintMillis = currentMillis;
      debugRoutines(); 
    }

    if(domeCenterSet == false){
      setDomeCenter(); 
     }

     if(currentMillis - lastVSMillis > 10000){
      lastVSMillis = currentMillis; 
      readBatteryVoltage();
     }
  }

  void checkIncomingData(){
      
      if(recM0.receiveData()){

        receiveMillis = currentMillis; 
        
        if(enableDrive == false){
          enableDrive = !enableDrive; 
        }
        delay(5);
        sendM0.sendData(); 
      }else if(currentMillis - receiveMillis >= 250 && enableDrive){
        enableDrive = !enableDrive; 
      }
  }
  



 


  void Servos(){

      int domeTurnPercent; 
      int y_Axis, x_Axis; 
      if(receiveFromM0Data.driveEnabled){
      
        
     // Serial.println(millis()); 
        if(domeServoMode){
          domeTurnPercent = map(Setpoint_domeSpinServoPid,-90,90,100,-100);
  
          if(domeTurnPercent>0){
            if(receiveFromM0Data.leftStickY >= 0){
              y_Axis = map(receiveFromM0Data.leftStickY,0,100,0,(100-domeTurnPercent));
              x_Axis = map(receiveFromM0Data.leftStickY,0,100,0,domeTurnPercent); 
            }else if(receiveFromM0Data.leftStickY<0){
              y_Axis = map(receiveFromM0Data.leftStickY,-100,0,(-100+domeTurnPercent),0); 
              x_Axis = map(receiveFromM0Data.leftStickY,-100,0,domeTurnPercent*-1,0); 
            }
    
            if(receiveFromM0Data.leftStickX >= 0){
              y_Axis -= map(receiveFromM0Data.leftStickX,0,100,0,domeTurnPercent); 
              x_Axis += map(receiveFromM0Data.leftStickX,0,100,0,(100-domeTurnPercent)); 
            }else if(receiveFromM0Data.leftStickX < 0){
              y_Axis += map(receiveFromM0Data.leftStickX,-100,0,domeTurnPercent,0); 
              x_Axis += map(receiveFromM0Data.leftStickX,-100,0,(-100+domeTurnPercent),0); 
            }
            
            }else if(domeTurnPercent == 0){
              y_Axis = receiveFromM0Data.leftStickY; 
              x_Axis = receiveFromM0Data.leftStickX; 
              
            }else if(domeTurnPercent< 0){
              if(receiveFromM0Data.leftStickY >= 0){
                y_Axis = map(receiveFromM0Data.leftStickY,0,100,0,(100+domeTurnPercent));
                x_Axis = map(receiveFromM0Data.leftStickY,0,100,0,domeTurnPercent); 
              }else if(receiveFromM0Data.leftStickY<0){
                y_Axis = map(receiveFromM0Data.leftStickY,-100,0,(-100-domeTurnPercent),0); 
                x_Axis = map(receiveFromM0Data.leftStickY,-100,0,domeTurnPercent*-1,0);
              }
    
              if(receiveFromM0Data.leftStickX >= 0){
                y_Axis -= map(receiveFromM0Data.leftStickX,0,100,0,domeTurnPercent); 
                x_Axis += map(receiveFromM0Data.leftStickX,0,100,0,(100+domeTurnPercent)); 
              }else if(receiveFromM0Data.leftStickX < 0){
                y_Axis += map(receiveFromM0Data.leftStickX,-100,0,domeTurnPercent,0); 
                x_Axis -= map(receiveFromM0Data.leftStickX,-100,0,(100+domeTurnPercent),0); 
              }
           } 
        }else{
          x_Axis = receiveFromM0Data.leftStickX;
          y_Axis = receiveFromM0Data.leftStickY;
        }
  
        
          if(receiveFromM0Data.xboxR3){
            y_Axis *= -1;
            x_Axis *= -1; 
          }

  
          //Scales the Y values from -100/100 to 40deg toward the 'back' and 35deg toward the 'front'
          if(y_Axis < 0){
            leftStickY = map(y_Axis,-100,0,-domeTiltYAxis_MaxAngle,0);
          }else if(y_Axis > 0){
            leftStickY = map(y_Axis,0,100,0,domeTiltYAxis_MaxAngle);
          }else{
            leftStickY = 0;
          }
  
          //Scales the X values from -100/100 to 29def left/right
          if(x_Axis < 0){
            leftStickX  = map(x_Axis,-100,0,-domeTiltXAxis_MaxAngle,0);
          }else if(x_Axis > 0){
            leftStickX  = map(x_Axis,0,100,0,domeTiltXAxis_MaxAngle);
          }else{
            leftStickX  = 0;
          }
  
          //Eases Y values by adding/subtracting the ease values every 10 millis
          if(currentMillis - lastServoUpdateMillis >= 10){
            lastServoUpdateMillis = currentMillis; 
            if(leftStickY < domeTiltAngle_Y_Axis - servoEase){
              domeTiltAngle_Y_Axis -= servoEase; 
            }else if(leftStickY > domeTiltAngle_Y_Axis + servoEase){
              domeTiltAngle_Y_Axis += servoEase; 
            }else{
             domeTiltAngle_Y_Axis = leftStickY;
            }
    
            //Eases X values by adding/subtracting the ease values every 10 millis
            if(leftStickX < domeTiltAngle_X_Axis - servoEase){
              domeTiltAngle_X_Axis -= servoEase; 
            }else if(leftStickX > domeTiltAngle_X_Axis + servoEase){
              domeTiltAngle_X_Axis += servoEase; 
            }else{
             domeTiltAngle_X_Axis = leftStickX;
            }
          }
  
  
          
  
  
        //Turns the scaled angles from the y axis to servo positions
        if(domeTiltAngle_Y_Axis < 0){
          leftServoPosition = leftServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromM0Data.pitch), -40,0,55,0);
          rightServoPosition = rightServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromM0Data.pitch), -40,0,-55,0);
          
        }else if(domeTiltAngle_Y_Axis > 0){
          leftServoPosition = leftServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromM0Data.pitch, 0,35,0,-50);
          rightServoPosition = rightServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromM0Data.pitch, 0,35,0,50);
        }else{
          if(receiveFromM0Data.pitch <= 0){
            leftServoPosition = leftServo_0_Position - map(receiveFromM0Data.pitch,-40,0,55,0);
            rightServoPosition = rightServo_0_Position + map(receiveFromM0Data.pitch,-40,0,55,0);
          }else{
            leftServoPosition = leftServo_0_Position - map(receiveFromM0Data.pitch,0,35,0,-50);
            rightServoPosition = rightServo_0_Position + map(receiveFromM0Data.pitch,0,35,0,-50);
          } 
          
        }
  
        //Turns the scaled angles from the x axis to servo positions and adds them to what we have from Y
        if(domeTiltAngle_X_Axis < 0){
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromM0Data.roll,-29,0,30,0);
          rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromM0Data.roll,-29,0,50,0 );
          
        }else if(domeTiltAngle_X_Axis > 0){
          leftServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-50)+map(receiveFromM0Data.roll,0,29,0,-30));
          rightServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-30)+map(receiveFromM0Data.roll,0,29,0,-50));
          
          
        }else{
          if(receiveFromM0Data.roll <= 0){
            leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromM0Data.roll,-29,0,30,0);
          rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromM0Data.roll,-29,0,50,0 );
          }else{
            leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromM0Data.roll,0,29,0,-30);
          rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromM0Data.roll,0,29,0,-50);
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
          
        }else { //if(leftDifference < rightDifference){
           if(rightOldPosition < rightServoPosition){ 
            rightOldPosition ++; 
          }else if (rightOldPosition > rightServoPosition){
            rightOldPosition --; 
          }

          if(leftOldPosition < leftServoPosition){
            leftOldPosition += leftDifference / rightDifference;
          }else if (leftOldPosition > leftServoPosition){
            leftOldPosition -= leftDifference / rightDifference;
          }
          
          
        }

  //delay(2); 
        //Sends positions to the servos
               
        myservo2.write(constrain(leftOldPosition,leftServo_0_Position-45,leftServo_0_Position+55), servoSpeed);     
        myservo1.write(constrain(rightOldPosition,rightServo_0_Position-55,rightServo_0_Position+45), servoSpeed);
      }
  }
 
  void encoder() {
    encPos = myEnc.read(); 

     if(receiveFromM0Data.xboxR3 && xboxR3Was == false){
      xboxR3Was = true;
      myEnc.write(encPos - 1497/2);
      
    }else if(receiveFromM0Data.xboxR3 == false && xboxR3Was == true){
      xboxR3Was = false;
      myEnc.write(encPos + 1497/2);
      
    }
  
  
    if(encPos > 1497){
      encPos -= 1497; 
      myEnc.write(encPos);
    }else if(encPos < 0){
      encPos += 1497; 
      myEnc.write(encPos);
    }
  
    if(receiveFromM0Data.xboxL3 == 1 && r3Flag == false){
      r3Flag = true; 
      domeServoMode = !domeServoMode; 
      
    }else if(receiveFromM0Data.xboxL3 == 0 && r3Flag == true){
      r3Flag = false; 
    }

   
    
  }
  
  void readBatteryVoltage(){
     // read the value at analog input
     batt_Voltage = (((analogRead(voltageSensor_Pin)) * 3.28) / 1024.0) / (R2/(R1+R2));
  
     #ifdef debugVS
           Serial.print(F("Battery Voltage: ")); Serial.println(batt_Voltage); 
      #endif
  }
  
  void setDomeCenter(){
  
      analogWrite(domeMotor_B_Pin,75);
      analogWrite(domeMotor_A_Pin,0);
  
    if(digitalRead(hallEffectSensor_Pin) == 0){
      domeCenterSet = 1; 
      myEnc.write(749);
    }
  }

  void spinStuff(){

      if(domeServoMode){
        domeServoMovement();
      }else{
        spinDome();
      }
      spinFlywheel();
  }
    

  void spinDome(){

    if(receiveFromM0Data.domeSpin > 3 && receiveFromM0Data.driveEnabled){
      analogWrite(domeMotor_A_Pin,map(receiveFromM0Data.domeSpin,3,100,0,255));
      analogWrite(domeMotor_B_Pin,0); 
    } else if(receiveFromM0Data.domeSpin < -3 && receiveFromM0Data.driveEnabled){
      analogWrite(domeMotor_B_Pin,map(receiveFromM0Data.domeSpin,-100,-3,255,0));
      analogWrite(domeMotor_A_Pin,0); 
    }else{
      analogWrite(domeMotor_A_Pin,0); 
      analogWrite(domeMotor_B_Pin,0); 
    }
  }
      

  void domeServoMovement(){
  
    if(receiveFromM0Data.driveEnabled){
        Setpoint_domeSpinServoPid = map(receiveFromM0Data.domeSpin, -100, 100, 75, -75); 
      }else{
        Setpoint_domeSpinServoPid = 0; 
      }
    
      Input_domeSpinServoPid = map(encPos,0,1497,180, -180); 
    
      myPID_domeSpinServoPid.Compute(); 
      
      if(Output_domeSpinServoPid >2 && receiveFromM0Data.driveEnabled){
        analogWrite(domeMotor_A_Pin,abs(Output_domeSpinServoPid));
        analogWrite(domeMotor_B_Pin,0); 
      } else if(Output_domeSpinServoPid <-2 && receiveFromM0Data.driveEnabled){
        analogWrite(domeMotor_B_Pin,abs(Output_domeSpinServoPid));
        analogWrite(domeMotor_A_Pin,0); 
      }else{
        analogWrite(domeMotor_A_Pin,0); 
        analogWrite(domeMotor_B_Pin,0); 
      }
  }

  void spinFlywheel(){
    if(receiveFromM0Data.flywheel > 1 && receiveFromM0Data.driveEnabled){
      analogWrite(flyWheelMotor_B_Pin,map(receiveFromM0Data.flywheel,1,100,0,255));
      analogWrite(flyWheelMotor_A_Pin,0); 
    } else if(receiveFromM0Data.flywheel < -1 && receiveFromM0Data.driveEnabled){
      analogWrite(flyWheelMotor_A_Pin,map(receiveFromM0Data.flywheel,-100,-1,255,0));
      analogWrite(flyWheelMotor_B_Pin,0); 
    }else{
      analogWrite(flyWheelMotor_A_Pin,0); 
      analogWrite(flyWheelMotor_B_Pin,0); 
    }
  }

  void psiTime(){
    if (receiveFromM0Data.psiFlash){
      psiValue = map(((analogRead(psiSensor_Pin_L) + analogRead(psiSensor_Pin_R)) / 2),0,1000,0,100);
    }else{
      psiValue - 0; 
    }
  }


  void debugRoutines(){

    #ifdef printRemote

      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxL1);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxL2);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxL3);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxR1);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxR2);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxR3);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxA);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxB);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxX);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxY);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxUP);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxDOWN);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxLEFT);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxRIGHT);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxBACK);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxSTART);
      Serial.print("\t"); Serial.print(receiveFromM0Data.xboxXBOX);
      Serial.print("\t"); Serial.print(receiveFromM0Data.leftStickX);
      Serial.print("\t"); Serial.print(receiveFromM0Data.leftStickY);
      Serial.print("\t"); Serial.print(receiveFromM0Data.rightStickX);
      Serial.print("\t"); Serial.print(receiveFromM0Data.rightStickY);
      Serial.println();


    #endif

    #ifdef debugDomeAndFly
      Serial.print("EN: "); Serial.print(receiveFromM0Data.driveEnabled);
      if(receiveFromM0Data.xboxL1 == 0 && receiveFromM0Data.xboxR1 == 0){
        if(domeServoMode){
          Serial.print(" ServoMode: In: "); Serial.print(Input_domeSpinServoPid);
          Serial.print("  Set: "); Serial.print(Setpoint_domeSpinServoPid); 
          Serial.print("  Out: "); Serial.println(Output_domeSpinServoPid); 
          
        } else {
          Serial.print(" Spin L: "); Serial.print(receiveFromM0Data.xboxL2); 
          Serial.print("R: "); Serial.println(receiveFromM0Data.xboxR2); 
        }
      } else {
        Serial.print(" Fly L: "); Serial.print(receiveFromM0Data.xboxL2); 
        Serial.print("R: "); Serial.println(receiveFromM0Data.xboxR2); 
      }

    #endif


    #ifdef debugPSI

      Serial.print("L: "); Serial.print(analogRead(psiSensor_Pin_L));
      Serial.print("    R: "); Serial.println(analogRead(psiSensor_Pin_R));

    #endif

    #ifdef printPitchAndRoll

      Serial.print("Pitch: "); Serial.print(receiveFromM0Data.pitch); 
      Serial.print(" actual pitch: "); Serial.print(pitch); 
      Serial.print("    Roll: "); Serial.println(receiveFromM0Data.roll); 

    #endif

    #ifdef printServoPositions
  
      Serial.print(leftServoPosition); 
      Serial.print("\t");Serial.println(rightServoPosition);  
    
    #endif

    #ifdef debugServos

      Serial.print("leftStickX  "); Serial.print(leftStickX); 
      Serial.print("  leftStickY  "); Serial.print(leftStickY); 
      Serial.print("  domeTiltAngle_X_Axis  "); Serial.print(leftStickY); 
      Serial.print("  domeTiltAngle_Y_Axis  "); Serial.print(domeTiltAngle_Y_Axis);
      Serial.print("  leftOldPosition  "); Serial.print(leftOldPosition);
      Serial.print("  rightOldPosition  "); Serial.println(rightOldPosition);
      

       
    
    #endif
    
  }


  
