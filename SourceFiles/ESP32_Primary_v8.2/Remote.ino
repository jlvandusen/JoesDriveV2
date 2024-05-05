void receiveRemote() {
#ifdef enableESPNOW
  // outgoingESPNOW.psi = sendPSI;
  // outgoingESPNOW.btn = sendHP;
  // outgoingESPNOW.bat = sendBAT;
  // outgoingESPNOW.dis = sendDIS;
#endif
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

    if(buttonsL.l1) { // Check if the Dome Controller L1 Button is held (Flywheel engaged, dome disengaged)
      sendTo32u4Data.domeSpin = 0;
      EnableFlywheel = true;
    } else {
      EnableFlywheel = false;
    }
    if(buttonsR.l1){ // Check if the drive Controller L1 Button is held (Flywheel disengaged, dome engaged)
      flywheel = 0;
      sendTo32u4Data.domeSpin = buttonsR.rightStickX;
      buttonsR.rightStickY = 0;
    } else {
      sendTo32u4Data.domeSpin = 0;
      Output_domeSpin_pwm = 0;
    }
    

// MP3 Trigger Commands being sent to the 32u4 0-9 currently, 
// you can add as many as you wish and even based on multiple button commands
#ifdef enableESPNOW
  // outgoingESPNOW.psi = sendPSI;
  // outgoingESPNOW.btn = sendHP;
  // outgoingESPNOW.bat = sendBAT;
  // outgoingESPNOW.dis = sendDIS;
  // int buttonCircle = 1; // Set to 1 or 0 based on your logic
  // int buttonCross = 0;  // Set to 1 or 0 based on your logic
  // int buttonUp = 1;     // Set to 1 or 0 based on your logic
  // int buttonDown = 0;   // Set to 1 or 0 based on your logic
  // int buttonLeft = 1;   // Set to 1 or 0 based on your logic
  // int buttonRight = 0;  // Set to 1 or 0 based on your logic
  // int buttonPS = 1;     // Set to 1 or 0 based on your logic

  // switch ( buttonsR.up) {   // Example switch...case statement
  //   case 1:
  //       sendTo32u4Data.soundcmd = 1;  // Play surprised!
  //       // outgoingESPNOW.psi = sendPSI;
  //       // outgoingESPNOW.btn = sendHP;
  //       // outgoingESPNOW.bat = sendBAT;
  //       // outgoingESPNOW.dis = sendDIS;
  //       sendPSI = 1;
  //       // Do something when buttonCircle is 1
  //       // For example, turn on an LED or perform an action
  //       break;
  //   case 0:
  //       // Do something when buttonCircle is 0
  //       // For example, turn off an LED or perform a different action
  //       sendPSI = 0;
  //       break;
  //   // Add similar cases for other buttons (cross, up, down, left, right, ps)
  //   // ...
  //   default:
  //       // If none of the cases match, execute default code (optional)
  //       break;
  // }
#endif
  if (buttonsR.up && buttonsL.up) { 
        setOffsetsAndSaveToEEPROM(); // If both Ups are pushed - call Store Configuration in Preferences (EEPROM)
        sendTo32u4Data.soundcmd = 6;  // Play surprised
        sendPSI = 6; // send sound command to ESPNOW DOME
        DEBUG_PRINT(F("Offsets written: ")); DEBUG_PRINTLN(enableDrive); 
        delay(1000);
  } else if (buttonsR.up && !buttonsL.up) {
      sendTo32u4Data.soundcmd = 1;  // Play surprised!
      sendPSI = 1;
      // #ifdef enableESPNOW
      //   // outgoingESPNOW.psi = sendPSI;
      //   // outgoingESPNOW.btn = sendHP;
      //   // outgoingESPNOW.bat = sendBAT;
      //   // outgoingESPNOW.dis = sendDIS;
      //   // sendPSI = 1;
      //   outgoingESPNOW.btn = 1;
      // #endif
  } else if (buttonsR.right) { // check for key press
      sendTo32u4Data.soundcmd = 2;  // Play quick answer!
      sendPSI = 2; // send sound command to ESPNOW DOME
      // #ifdef enableESPNOW
      //   // outgoingESPNOW.psi = sendPSI;
      //   // outgoingESPNOW.btn = sendHP;
      //   // outgoingESPNOW.bat = sendBAT;
      //   // outgoingESPNOW.dis = sendDIS;
      //   // sendHP = 1;
      //   sendPSI =2;
      // #endif
  } else if (buttonsR.down) { // check for key press
    if (buttonsL.down) {
        wipenvram(); // If both Ups are pushed - call Store Configuration in Preferences (EEPROM)
        sendTo32u4Data.soundcmd = 99;  // Play surprised!
      } else {
          sendTo32u4Data.soundcmd = 3;  // Play surprised!
          sendPSI = 3; // send sound command to ESPNOW DOME
          // #ifdef enableESPNOW
          //   // outgoingESPNOW.psi = sendPSI;
          //   // outgoingESPNOW.btn = sendHP;
          //   // outgoingESPNOW.bat = sendBAT;
          //   // outgoingESPNOW.dis = sendDIS;
          //   sendPSI = 3;
          // #endif
      }
  } else if (buttonsR.left) { // check for key press
    sendTo32u4Data.soundcmd = 4;  // Play surprised!
    sendPSI = 4;
  } else if (buttonsL.up) { // check for key press
    if (buttonsR.up) {
        setOffsetsAndSaveToEEPROM(); // If both Ups are pushed - call Store Configuration in Preferences (EEPROM)
        sendTo32u4Data.soundcmd = 99;  // Play surprised!
        sendPSI = 99;
      } else sendTo32u4Data.soundcmd = 1;  // Play surprised!
    sendTo32u4Data.soundcmd = 5;  // Play surprised!
    sendPSI = 5;
  } else if (buttonsL.right) { // check for key press
    sendTo32u4Data.soundcmd = 6;  // Play surprised!
    sendPSI = 6;
  } else if (buttonsL.down) { // check for key press
    if (buttonsR.down) {
    wipenvram(); // If both Ups are pushed - call Store Configuration in Preferences (EEPROM)
    sendTo32u4Data.soundcmd = 99;  // Play surprised!
      } else { 
        sendTo32u4Data.soundcmd = 7;  // Play surprised!
        sendPSI = 7;
      }
  } else if (buttonsL.left) { // check for key press
    sendTo32u4Data.soundcmd = 8;  // Play surprised!
    sendPSI = 8;
  } else if (buttonsR.circle) { // check for key press
    sendTo32u4Data.soundcmd = 9;  // Play surprised!
    sendPSI = 9;
  } else {
    sendTo32u4Data.soundcmd = 0;
    sendPSI = 0;
  }

 // Enable or disable Drives (including Dome Controls) leave sounds working
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

 // Enable or disable Dome motor servo mode
    if(CHECK_BUTTON_PRESSEDL(l3)){
      if (DomeServoMode == false) {
        DomeServoMode = true;
        sendTo32u4Data.moveR3 = true;
        DEBUG_PRINTLN("Dome Servo Mode");
        DEBUG_PRINTLN(DomeServoMode);
      } else {
        DomeServoMode = false; 
        DEBUG_PRINTLN("Dome Servo Disabled");
        DEBUG_PRINTLN(DomeServoMode);
        sendTo32u4Data.moveR3 = false;
      }
    }
     // Enable or disable Dome motor servo mode
    // if(CHECK_BUTTON_PRESSEDR(up)){
    //   if (sendPSI==1) {
    //     sendPSI = 0;
    //   } else {
    //     sendPSI = 1;
    //   }
    // }
    // if (buttonsL.ps) {
    //   DEBUG_PRINTLN("Dome Servo Mode");
    //   if (buttonsL.cross) {
    //     DEBUG_PRINTLN("Dome Nav PS held");
    //     DEBUG_PRINTLN("\r\nDisconnecting the Dome controller.\r\n");
    //     // driveController.disconnect();
    //     domeController.disconnect();
    //     ESP.restart();
    //     enableDrive = false;
    //   }

    // } 
    if (buttonsR.ps) {
        if (buttonsR.cross) {
        DEBUG_PRINTLN("\r\nDisconnecting the Drive controllers.\r\n");
        enableDrive = !enableDrive;
        driveController.disconnect();
        ESP.restart();
        // domeController.disconnect();
      }
    }

 // Reverse all motor controls (reverse direction on a dime with the dome)
    if(CHECK_BUTTON_PRESSEDR(l3)){
      if (reverseDrive == false) {
        reverseDrive = true;
        // #ifdef debugRemote
        DEBUG_PRINTLN("Drive Reversed");
        // #endif
      } else {
        reverseDrive = false; 
        // #ifdef debugRemote
        DEBUG_PRINTLN("Drive Forward");
        // #endif
      }
      sendTo32u4Data.moveL3 = reverseDrive; 
    }

// Send all Left Stick commands to the 32u4 if its the left dome controller
    if((buttonsL.leftStickX < -(joystickDeadZoneRange)) || (buttonsL.leftStickX > (joystickDeadZoneRange))){
      sendTo32u4Data.leftStickX = buttonsL.leftStickX;
    }else{
      sendTo32u4Data.leftStickX = 0; 
    }
    if((buttonsL.leftStickY < -(joystickDeadZoneRange)) || (buttonsL.leftStickY > (joystickDeadZoneRange))){
      sendTo32u4Data.leftStickY = buttonsL.leftStickY;
    }else{
      sendTo32u4Data.leftStickY = 0; 
    }

// Check all Right stick commands against the deadzonerange and allow values outside of 0
    if((buttonsR.rightStickX < -(joystickDeadZoneRange)) || (buttonsR.rightStickX > (joystickDeadZoneRange))){
      buttonsR.rightStickX = buttonsR.rightStickX;
    }else{
      buttonsR.rightStickX = 0; 
    }
    if((buttonsR.rightStickY < -(joystickDeadZoneRange)) || (buttonsR.rightStickY > (joystickDeadZoneRange))){
      buttonsR.rightStickY = buttonsR.rightStickY;
    }else{
      buttonsR.rightStickY = 0; 
    }  
  }  else controllerConnected = false;    
#endif

}
