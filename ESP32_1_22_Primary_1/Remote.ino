void receiveRemote() {
  
#ifdef MOVECONTROLLER
  if (driveController.isConnected()) {
    if (!drivecontrollerConnected){ // notify us of the connection as long as the status is false set it true immediately thereafter - display once
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

    if(buttonsL.l1){ // Check if the Dome Controller L1 Button is held (Flywheel engaged)
      sendTo32u4Data.domeSpin = 0;
      sendTo32u4Data.flywheel = buttonsR.rightStickX;
      Output_flywheel_pwm = map(buttonsR.rightStickX,-127,127,-255,255);
    }
    else {
      sendTo32u4Data.flywheel = 0;
      Output_flywheel_pwm = 0;
      
    }
    if(buttonsR.l1){ // Check if the Dome Controller L1 Button is held (Flywheel engaged)
      sendTo32u4Data.flywheel = 0;
      sendTo32u4Data.domeSpin = buttonsR.rightStickX;
      Output_domeSpin_pwm = map(buttonsR.rightStickX,-127,127,-255,255);
    }
    else {
      sendTo32u4Data.domeSpin = 0;
      Output_domeSpin_pwm = 0;
      
    }
    
#ifdef MP3SOUNDS
/*
* 
* MP3 SOUND CONTROLS for BB8
* 
*/

//  Example Buttons from Move Controller
//  buttonsL.down = domeController.state.button.down;
//  buttonsL.left = domeController.state.button.left;
//  buttonsL.right = domeController.state.button.right;
//  buttonsL.ps = domeController.state.button.ps;
//  buttonsR.circle = driveController.state.button.circle;
//  buttonsR.cross = driveController.state.button.cross;
//  buttonsR.up = driveController.state.button.up;
//  buttonsR.down = driveController.state.button.down;
//  buttonsR.left = driveController.state.button.left;
//  buttonsR.right = driveController.state.button.right;

if (mp3.isConnected()) {  // check if the MP3 Trigger is connected via i2c
  if (CHECK_BUTTON_PRESSEDL(up)) {
    if (mp3.isPlaying() == true) mp3.stop();  // if track is playing stop it and play next else play a track
    if (CHECK_BUTTON_PRESSEDL(l1)) {
      sound = 1;  // Play surprised!
    } else if (CHECK_BUTTON_PRESSEDR(l1)){
      sound = 40;  // Play surprised!
    } else {
    sound = randomsound;  // Play surprised!
    }
  } else if (CHECK_BUTTON_PRESSEDL(right)) { // check for key press (once)
    if (mp3.isPlaying() == true) mp3.stop();
    sound = 2;  // Play surprised!
  } else if (CHECK_BUTTON_PRESSEDL(down)) { 
    if (mp3.isPlaying() == true) mp3.stop();
    sound = 5;  // Play angry!
  } else if (CHECK_BUTTON_PRESSEDL(left)) { 
    if (mp3.isPlaying() == true) mp3.stop();;
    sound = 9;  // Play happy
  } else if (CHECK_BUTTON_PRESSEDL(up)) { 
    if (mp3.isPlaying() == true) mp3.stop();
    sound = 10;  // Play happy{
  } else if (CHECK_BUTTON_PRESSEDL(right)) { 
    if (mp3.isPlaying() == true) mp3.stop();
    sound = 2;  // Play surprised!
  } else if (CHECK_BUTTON_PRESSEDL(down)) { 
    if (mp3.isPlaying() == true) mp3.stop();
    sound = 5;  // Play angry!
  } else if (CHECK_BUTTON_PRESSEDL(left)) { 
    if (mp3.isPlaying() == true) mp3.stop();
    sound = 9;  // Play happy
  }
//  if (mp3.isPlaying() == true) mp3.stop(); // if track is playing stop it and play next else play a track
  mp3.playTrack(sound);
  
}

#endif
//  if(CHECK_BUTTON_PRESSEDL(up)){
//    
//  }
//    if (CHECK_BUTTON_PRESSEDR(up)) { // check for key press (once)
//      if (mp3IsPlaying() == true) {
//        mp3Stop();
//        
//      } else {
//        sound = randomsound;
//      }
//    }

//    buttonsL.down = domeController.state.button.down;
//    buttonsL.left = domeController.state.button.left;
//    buttonsL.right = domeController.state.button.right;
//    buttonsL.ps = domeController.state.button.ps;
//    buttonsR.circle = driveController.state.button.circle;
//    buttonsR.cross = driveController.state.button.cross;
//    buttonsR.up = driveController.state.button.up;
//    buttonsR.down = driveController.state.button.down;
//    buttonsR.left = driveController.state.button.left;
//    buttonsR.right = driveController.state.button.right;

//  if(CHECK_BUTTON_PRESSEDR(l1)){

//    if(buttonsR.l1){
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

//    if(CHECK_BUTTON_PRESSEDR(l3)){
    if(CHECK_BUTTON_PRESSEDR(ps)){
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
    
    if(CHECK_BUTTON_PRESSEDR(l3)){
      if (DomeServoMode == false) {
        DomeServoMode = true;
        DEBUG_PRINTLN("Dome Servo Mode");
        DEBUG_PRINTLN(DomeServoMode);
      } else {
        DomeServoMode = false; 
        DEBUG_PRINTLN("Dome Servo Disabled");
        DEBUG_PRINTLN(DomeServoMode);
      }
      sendTo32u4Data.moveR3 = DomeServoMode;
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
      sendTo32u4Data.leftStickX = buttonsL.leftStickX;
      #ifdef debugRemote
      DEBUG_PRINT("Dome LeftStickX: ");
      DEBUG_PRINTLN(sendTo32u4Data.leftStickX);
      #endif
    }else{
      sendTo32u4Data.leftStickX = 0; 
    }
    if((buttonsL.leftStickY < -(joystickDeadZoneRange)) || (buttonsL.leftStickY > (joystickDeadZoneRange))){
      sendTo32u4Data.leftStickY = buttonsL.leftStickY;
      #ifdef debugRemote
        DEBUG_PRINT("Dome LeftStickY: ");
        DEBUG_PRINTLN(sendTo32u4Data.leftStickY);
      #endif
    }else{
      sendTo32u4Data.leftStickY = 0; 
    }  
    if((buttonsR.rightStickX < -(joystickDeadZoneRange)) || (buttonsR.rightStickX > (joystickDeadZoneRange))){
      buttonsR.rightStickX = buttonsR.rightStickX;
      #ifdef debugRemote
        DEBUG_PRINT("Drive RightStickX: ");
        DEBUG_PRINTLN(buttonsR.rightStickX);
      #endif
    }else{
      buttonsR.rightStickX = 0; 
    }
    if((buttonsR.rightStickY < -(joystickDeadZoneRange)) || (buttonsR.rightStickY > (joystickDeadZoneRange))){
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
