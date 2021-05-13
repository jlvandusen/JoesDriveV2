   

  //#define debugRemote
  //#define debugIMU
  //#define debugMainDrive
  //#define debugS2S
  //#define debugSounds
  #define MOVECONTROLLER
//  #define XBOXCONTROLLER   
   
   
   
   
   
   /*
    PIN DEFINITIONS
  */
  //#define psiSensor_Pin_L A0
  //#define psiSensor_Pin_R A5
  #define NeoPixel_pin 19
  #define S2S_pin_1 16
  #define S2S_pin_2 15
  #define drivePin_1 18
  #define drivePin_2 17
  #define S2SPot_pin A0
  #define S2S_offset -120
  
  /*
   Modified USB Host library: https://github.com/gdsports/USB_Host_Library_SAMD
   */


  #define driveDelay .75

  #include <PID_v1.h>

  #ifdef XBOXCONTROLLER
  #include <XBOXRECV.h>
  #endif
  
  #ifdef MOVECONTROLLER
  #include <PS3BT.h>
  #include <usbhub.h>
  #endif
  
  #include <Arduino.h>
  #include <EasyTransfer.h>
  #include <SPI.h>
  #include <SD.h>
  #include <Adafruit_VS1053.h>


  #define SerialDebug Serial3

  #define BUFFER_LENGTH 64
  #define TWI_BUFFER_LENGTH 64

  #ifdef XBOXCONTROLLER
  USBHost UsbH;
  XBOXRECV Xbox(&UsbH);
  #endif

  #ifdef MOVECONTROLLER
  USBHost UsbH;
  BTD Btd(&UsbH); // You have to create the Bluetooth Dongle instance like so
  PS3BT *PS3[2]; // We will use this pointer to store the two instance, you can easily make it larger if you like, but it will use a lot of RAM!
  const uint8_t length = sizeof(PS3) / sizeof(PS3[0]); // Get the lenght of the array
  bool printAngle[length];
  bool oldControllerState[length];
  #endif

  
  //create UART object *************************************
  EasyTransfer recIMU; 
  EasyTransfer send32u4;
  EasyTransfer rec32u4; 
  
  struct RECEIVE_DATA_STRUCTURE_IMU{
    float pitch;
    float roll; 
  };

  RECEIVE_DATA_STRUCTURE_IMU receiveIMUData;
  

 //create SEND object  *************************************
  
  #ifdef XBOXCONTROLLER
  struct SEND_DATA_STRUCTURE_32u4{  
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
  
  #ifdef MOVECONTROLLER
  struct SEND_DATA_STRUCTURE_32u4{  
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
  
  //give a name to the group of data
  SEND_DATA_STRUCTURE_32u4 sendTo32u4Data; // - slave_config
  
  struct RECEIVE_DATA_STRUCTURE_32u4{ // - SLAVE_DATA
    int16_t tiltAngle; 
  };

  //give a name to the group of data
  RECEIVE_DATA_STRUCTURE_32u4 receiveFrom32u4Data; // - slave_data


  struct controllerButtons{
    bool a,b,x,y,up,down,left,right,xbox,back,l1,r1,start;
    int8_t rightStickX,rightStickY,l2,r2;
  };
  controllerButtons buttons;
  

  //*** Set up Music Maker Featherwing *******************************************
  #define VS1053_RESET   -1     // VS1053 reset pin (not used!)
  #define VS1053_CS       6     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
  #define VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin
  Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);


    #include "wiring_private.h" // pinPeripheral() function
 
    Uart Serial2 (&sercom1, 12, 11, SERCOM_RX_PAD_3, UART_TX_PAD_0);
    void SERCOM1_Handler()
    {
      Serial2.IrqHandler();
    }



    Uart Serial3 (&sercom3, 21, 20, SERCOM_RX_PAD_1, UART_TX_PAD_0);
    void SERCOM3_Handler()
    {
      Serial3.IrqHandler();
    }


    

  bool enableDrive,reverseDrive; 

  unsigned long currentMillis, IMUmillis, lastLoopMillis, lastDomeMillis, rec32u4Millis, lastPrintMillis; 
  bool IMUconnected, controllerConnected, Send_Rec, feather2Connected; 
  //Define Variables we'll be connecting to
  double Setpoint_S2S_Servo, Input_S2S_Servo, Output_S2S_Servo;
  double Setpoint_S2S_Stabilization, Input_S2S_Stabilization, Output_S2S_Stabilization;
  double Setpoint_Drive, Input_Drive, Output_Drive, Setpoint_Drive_In;

  int numberOfTracks = 55; 
  int i; 
  char *soundNames[]={"BB80001.ogg", "BB80002.ogg",  "BB80003.ogg",  "BB80004.ogg",  "BB80005.ogg",  "BB80006.ogg",  "BB80007.ogg",  "BB80008.ogg",  "BB80009.ogg",  "BB80010.ogg",  
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

 
  
  void setup() {
     
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(74880);
    Serial3.begin(74880);
    pinPeripheral(11, PIO_SERCOM);
    pinPeripheral(12, PIO_SERCOM);
    pinPeripheral(20, PIO_SERCOM);
    pinPeripheral(21, PIO_SERCOM);
    
    #ifdef XBOXCONTROLLER
    if (UsbH.Init()) {
      SerialDebug.print(F("\r\nUSB host did not start"));
      while (1); //haltv
    }
    SerialDebug.print(F("\r\nXbox Wireless Receiver Library Started"));
    #endif

    #ifdef MOVECONTROLLER
    for (uint8_t i = 0; i < length; i++) {
      PS3[i] = new PS3BT(&Btd); // Create the instances
      PS3[i]->attachOnInit(onInit); // onInit() is called upon a new connection - you can call the function whatever you like
    }
  
    SerialDebug.begin(115200);
    if (UsbH.Init()) {
      SerialDebug.print(F("\r\nUSB host did not start"));
      while (1); //halt
    }
    SerialDebug.print(F("\r\nPS3 Bluetooth Library Started"));
    #endif
    
    if (! musicPlayer.begin()) { // initialise the music player
       SerialDebug.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    }
  
    SerialDebug.println(F("VS1053 found"));
    if (!SD.begin(CARDCS)) {
      SerialDebug.println(F("SD failed, or not present"));
    }
    SerialDebug.println("SD OK!");

    musicPlayer.setVolume(1,1);

    musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int

    myPID_S2S_Servo.SetMode(AUTOMATIC);
    myPID_S2S_Servo.SetOutputLimits(-255, 255);
    
    myPID_S2S_Stabilization.SetMode(AUTOMATIC);
    myPID_S2S_Stabilization.SetOutputLimits(-255, 255);

    myPID_Drive.SetMode(AUTOMATIC);
    myPID_Drive.SetOutputLimits(-255, 255);

    recIMU.begin(details(receiveIMUData), &Serial1); 
    rec32u4.begin(details(receiveFrom32u4Data), &Serial2);
    send32u4.begin(details(sendTo32u4Data), &Serial2);
       
  }





  
  void loop() {

    currentMillis = millis(); 
    receiveIMU();
    if(currentMillis - lastLoopMillis >= 10){
      lastLoopMillis = currentMillis; 
      
      receiveRemote();
//      S2S_Movement(); 
//      drive_Movement(); 
//      sendDataTo32u4();
      sounds(); 

        if(currentMillis - lastPrintMillis >= 70){
              lastPrintMillis = currentMillis;
              debugRoutines();
        }
     }
      
    if(feather2Connected){
      digitalWrite(13, HIGH);
    }else{
      digitalWrite(13, LOW);
    }
  }


  
  void receiveRemote(){

    #ifdef XBOXCONTROLLER
    UsbH.Task();
   
    if (Xbox.XboxReceiverConnected && Xbox.Xbox360Connected[0]) {
    
      controllerConnected = true;
      
          buttons.l1 = (Xbox.getButtonPress(L1, 0));
          buttons.l2 = constrain(map(Xbox.getButtonPress(L2, 0),0,255,0,100),0,100);
          sendTo32u4Data.xboxL3 = (Xbox.getButtonPress(L3, 0));
          buttons.r1 = (Xbox.getButtonPress(R1, 0));
          buttons.r2 = constrain(map(Xbox.getButtonPress(R2, 0),0,250,0,100),0,100);
          sendTo32u4Data.xboxR3 = reverseDrive;
          buttons.a = (Xbox.getButtonPress(A, 0));
          buttons.b = (Xbox.getButtonPress(B, 0));
          buttons.x = (Xbox.getButtonPress(X, 0));
          buttons.y = (Xbox.getButtonPress(Y, 0));
          buttons.up = (Xbox.getButtonPress(UP, 0));
          buttons.down = (Xbox.getButtonPress(DOWN, 0));
          buttons.left = (Xbox.getButtonPress(LEFT, 0));
          buttons.right = (Xbox.getButtonPress(RIGHT, 0));
          buttons.back = (Xbox.getButtonPress(BACK, 0));
          //buttons.start = (Xbox.getButtonClick(START, 0));
          buttons.xbox = (Xbox.getButtonPress(XBOX, 0));

          if(buttons.l1 || buttons.r1){
            sendTo32u4Data.domeSpin = 0;
            if(buttons.l2 > 5){
              sendTo32u4Data.flywheel = map(buttons.l2,5,100,0,-100);
            }else if(buttons.r2 > 5){
              sendTo32u4Data.flywheel = map(buttons.r2,5,100,0,100);
            }else{
              sendTo32u4Data.flywheel = 0; 
            }
          }else{
            sendTo32u4Data.flywheel = 0;
            if(buttons.l2 > 5){
              sendTo32u4Data.domeSpin = map(buttons.l2,5,100,0,-100);
            }else if(buttons.r2 > 5){
              sendTo32u4Data.domeSpin = map(buttons.r2,5,100,0,100);
            }else{
              sendTo32u4Data.domeSpin = 0; 
            }
          }
          
          if(Xbox.getButtonClick(START, 0)){
            enableDrive = !enableDrive; 
            sendTo32u4Data.driveEnabled = enableDrive; 
          }
          if(Xbox.getButtonClick(R3, 0)){
            reverseDrive = !reverseDrive; 
            sendTo32u4Data.xboxR3 = reverseDrive; 
          }
          if(feather2Connected == false){
            enableDrive = false; 
            sendTo32u4Data.driveEnabled = enableDrive;
          }

          if(Xbox.getAnalogHat(LeftHatX,0) >= 6000){
            sendTo32u4Data.leftStickX = constrain(map((Xbox.getAnalogHat(LeftHatX, 0)),6000,33000,0,100),0,100); 
          }else if(Xbox.getAnalogHat(LeftHatX,0) <= -6000){
            sendTo32u4Data.leftStickX = constrain(map((Xbox.getAnalogHat(LeftHatX, 0)),-33000,-6000,-100,0),-100,0); 
          }else{
            sendTo32u4Data.leftStickX = 0; 
          }

          if(Xbox.getAnalogHat(LeftHatY,0) >= 6000){
            sendTo32u4Data.leftStickY = constrain(map((Xbox.getAnalogHat(LeftHatY, 0)),6000,33000,0,100),0,100); 
          }else if(Xbox.getAnalogHat(LeftHatY,0) <= -6000){
            sendTo32u4Data.leftStickY = constrain(map((Xbox.getAnalogHat(LeftHatY, 0)),-33000,-6000,-100,0),-100,0); 
          } else{
            sendTo32u4Data.leftStickY = 0;
          }
          
          if(Xbox.getAnalogHat(RightHatX,0) >= 8000){
            buttons.rightStickX = constrain(map((Xbox.getAnalogHat(RightHatX, 0)),6000,23000,0,30),0,30); 
           
          }else if(Xbox.getAnalogHat(RightHatX,0) <= -8000){
            buttons.rightStickX = constrain(map((Xbox.getAnalogHat(RightHatX, 0)),-25000,-6000,-30,0),-30,0); 
          
          } else{
            buttons.rightStickX = 0;
          
          }
          
          if(Xbox.getAnalogHat(RightHatY,0) >= 6000){
            buttons.rightStickY = constrain(map((Xbox.getAnalogHat(RightHatY, 0)),6000,33000,0,100),0,100); 
          }else if(Xbox.getAnalogHat(RightHatY,0) <= -6000){
            buttons.rightStickY = constrain(map((Xbox.getAnalogHat(RightHatY, 0)),-33000,-6000,-100,0),-100,0); 
          }else{
            buttons.rightStickY = 0;
          }


/* //For reference:
 *  Xbox.setLedOn(LED1, 0);
 *  Xbox.setLedOn(LED2, 0);
 *  Xbox.setLedOn(LED3, 0);
 *  Xbox.setLedOn(LED4, 0);
 *  Xbox.setLedMode(ALTERNATING, 0);
 *  Xbox.setLedBlink(ALL, 0);
 *  Xbox.setLedMode(ROTATING, 0);
 *  Xbox.setRumbleOn(X, 0); //x == 0 - 250
 *  
 */


          
    }  else {
      controllerConnected = false;    
    }
    #endif

  #ifdef MOVECONTROLLER
    
  for (uint8_t i = 0; i < length; i++) {
    if (PS3[i]->PS3Connected || PS3[i]->PS3NavigationConnected) {
      controllerConnected = true;
      if (PS3[i]->getAnalogHat(LeftHatX) > 137 || PS3[i]->getAnalogHat(LeftHatX) < 117 || PS3[i]->getAnalogHat(LeftHatY) > 137 || PS3[i]->getAnalogHat(LeftHatY) < 117 || PS3[i]->getAnalogHat(RightHatX) > 137 || PS3[i]->getAnalogHat(RightHatX) < 117 || PS3[i]->getAnalogHat(RightHatY) > 137 || PS3[i]->getAnalogHat(RightHatY) < 117) {
        SerialDebug.print(F("\r\nLeftHatX: "));
        SerialDebug.print(PS3[i]->getAnalogHat(LeftHatX));
        SerialDebug.print(F("\tLeftHatY: "));
        SerialDebug.print(PS3[i]->getAnalogHat(LeftHatY));
        if (PS3[i]->PS3Connected) { // The Navigation controller only have one joystick
          SerialDebug.print(F("\tRightHatX: "));
          SerialDebug.print(PS3[i]->getAnalogHat(RightHatX));
          SerialDebug.print(F("\tRightHatY: "));
          SerialDebug.print(PS3[i]->getAnalogHat(RightHatY));
        }
      }
      //Analog button values can be read from almost all buttons
      if (PS3[i]->getAnalogButton(L2) || PS3[i]->getAnalogButton(R2)) {
        SerialDebug.print(F("\r\nL2: "));
        SerialDebug.print(PS3[i]->getAnalogButton(L2));
        if (PS3[i]->PS3Connected) {
          SerialDebug.print(F("\tR2: "));
          SerialDebug.print(PS3[i]->getAnalogButton(R2));
        }
      }
      if (PS3[i]->getButtonClick(PS)) {
        SerialDebug.print(F("\r\nPS"));
        PS3[i]->disconnect();
        oldControllerState[i] = false; // Reset value
      }
      else {
        if (PS3[i]->getButtonClick(TRIANGLE))
          SerialDebug.print(F("\r\nTraingle"));
        if (PS3[i]->getButtonClick(CIRCLE))
          SerialDebug.print(F("\r\nCircle"));
        if (PS3[i]->getButtonClick(CROSS))
          SerialDebug.print(F("\r\nCross"));
        if (PS3[i]->getButtonClick(SQUARE))
          SerialDebug.print(F("\r\nSquare"));

        if (PS3[i]->getButtonClick(UP)) {
          SerialDebug.print(F("\r\nUp"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED4);
          }
        }
        if (PS3[i]->getButtonClick(RIGHT)) {
          SerialDebug.print(F("\r\nRight"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED1);
          }
        }
        if (PS3[i]->getButtonClick(DOWN)) {
          SerialDebug.print(F("\r\nDown"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED2);
          }
        }
        if (PS3[i]->getButtonClick(LEFT)) {
          SerialDebug.print(F("\r\nLeft"));
          if (PS3[i]->PS3Connected) {
            PS3[i]->setLedOff();
            PS3[i]->setLedOn(LED3);
          }
        }

        if (PS3[i]->getButtonClick(L1))
          SerialDebug.print(F("\r\nL1"));
        if (PS3[i]->getButtonClick(L3))
          SerialDebug.print(F("\r\nL3"));
        if (PS3[i]->getButtonClick(R1))
          SerialDebug.print(F("\r\nR1"));
        if (PS3[i]->getButtonClick(R3))
          SerialDebug.print(F("\r\nR3"));

        if (PS3[i]->getButtonClick(SELECT)) {
          SerialDebug.print(F("\r\nSelect - "));
          PS3[i]->printStatusString();
        }
        if (PS3[i]->getButtonClick(START)) {
          SerialDebug.print(F("\r\nStart"));
          printAngle[i] = !printAngle[i];
        }
      }
      if (printAngle[i]) {
        SerialDebug.print(F("\r\nPitch: "));
        SerialDebug.print(PS3[i]->getAngle(Pitch));
        SerialDebug.print(F("\tRoll: "));
        SerialDebug.print(PS3[i]->getAngle(Roll));
      }
    } else {
      controllerConnected = false;    
    }
    /* I have removed the PS3 Move code as an Uno will run out of RAM if it's included */
//    else if(PS3[i]->PS3MoveConnected) {
  }
  #endif

  }
    
  
  void receiveIMU(){
    if(recIMU.receiveData()){
     IMUmillis = currentMillis; 
     sendTo32u4Data.roll = receiveIMUData.roll;
     sendTo32u4Data.pitch = receiveIMUData.pitch; 
      if(IMUconnected == false){
        IMUconnected = true;
      }
    }else{
      if(currentMillis - IMUmillis > 25){
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
      }
     }else if(currentMillis - rec32u4Millis > 100){
        feather2Connected = false; 
     }       
  }



 void S2S_Movement(){
  Input_S2S_Servo = receiveIMUData.roll *-1;
  if(sendTo32u4Data.xboxR3 == false){
    buttons.rightStickX *= -1; 
  }
    Setpoint_S2S_Servo = buttons.rightStickX; 
  
  myPID_S2S_Servo.Compute();
 

  Input_S2S_Stabilization = (map(analogRead(S2SPot_pin),0,1023,0,270)+S2S_offset)*-1;
  //Input_S2S_Stabilization = (map(analogRead(S2SPot_pin),0,1023,0,270)-160)*-1;

  Setpoint_S2S_Stabilization = Output_S2S_Servo;
  myPID_S2S_Stabilization.Compute(); 

  if(Output_S2S_Stabilization > 5 && controllerConnected && enableDrive){
    analogWrite(S2S_pin_1, map(abs(Output_S2S_Stabilization),0,255,0,220));
    analogWrite(S2S_pin_2, 0); 
  }else if(Output_S2S_Stabilization < -5 && controllerConnected && enableDrive){
    analogWrite(S2S_pin_1,  0);
    analogWrite(S2S_pin_2,  map(Output_S2S_Stabilization,-255,0,220,0));
    
  }else{
    analogWrite(S2S_pin_2,  0);
    analogWrite(S2S_pin_1,  0);
  }
 }

 void drive_Movement(){
  Input_Drive = receiveIMUData.pitch; 
  if(reverseDrive == false){
    buttons.rightStickY *= -1;
  }
    
    if(Setpoint_Drive_In > buttons.rightStickY){
      Setpoint_Drive_In-=driveDelay; 
    }else if(Setpoint_Drive_In < buttons.rightStickY){
      Setpoint_Drive_In+=driveDelay; 
    }

    Setpoint_Drive = map(Setpoint_Drive_In,-100,100,-40,40); 
  
  //Setpoint_Drive = buttons.rightStickY * -1; 
  myPID_Drive.Compute(); 

    if(Output_Drive > 5 && controllerConnected && enableDrive){
    analogWrite(drivePin_2, map(abs(Output_Drive),0,255,0,220));
    analogWrite(drivePin_1,0); 
  }else if(Output_Drive < -5 && controllerConnected && enableDrive){
    analogWrite(drivePin_2, 0);
    analogWrite(drivePin_1, map(abs(Output_Drive),0,255,0,220));
    
  }else{
    analogWrite(drivePin_2, 0);
    analogWrite(drivePin_1,0);
  }

 }

  void sounds(){
           //******test sound

        if(buttons.a == 1  && musicPlayer.stopped()){
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

        if(buttons.back){
          musicPlayer.stopPlaying();
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


      #ifdef debugIMU
            SerialDebug.print(receiveIMUData.pitch); SerialDebug.print('\t');
            SerialDebug.println(receiveIMUData.roll);

      #endif


      #ifdef debugMainDrive

            SerialDebug.print(F("In/Pitch: ")); SerialDebug.print(Input_Drive); SerialDebug.print('\t'); 
            SerialDebug.print(F("Set/Joy: ")); SerialDebug.print(Setpoint_Drive); SerialDebug.print('\t');
            SerialDebug.print(F("Out: ")); SerialDebug.print(Output_Drive); SerialDebug.print('\t');
            SerialDebug.print(F("Cont. Conn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
            SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
            
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

  #ifdef MOVECONTROLLER
   void onInit() {
    for (uint8_t i = 0; i < length; i++) {
      if ((PS3[i]->PS3Connected || PS3[i]->PS3NavigationConnected) && !oldControllerState[i]) {
        oldControllerState[i] = true; // Used to check which is the new controller
        PS3[i]->setLedOn((LEDEnum)(i + 1)); // Cast directly to LEDEnum - see: "controllerEnums.h"
      }
    }
  }
  #endif
