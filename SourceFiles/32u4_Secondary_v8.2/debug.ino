void debugRoutines(){

  #ifdef debugDOME
    // int hallEffectSensor = digitalRead(hallEffectSensor_Pin);
    // int currentPosition = myEnc.read();
    // int positionDifference = targetPosition - currentPosition;
    long encoderCounts = myEnc.read(); // Read the current counts from the encoder
    float rotationDegrees = (encoderCounts / 1680.0) * 360.0; // Calculate the rotation in degrees
    Serial.print("Rotation: ");
    Serial.print(rotationDegrees);
    Serial.println(" degrees");
    // SerialDebug.print(F("encPos: ")); SerialDebug.print(encPos); SerialDebug.print('\t');
    // SerialDebug.print(F("domeCenterSet: ")); SerialDebug.print(domeCenterSet); SerialDebug.print('\t');
    // SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
    // SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
    // SerialDebug.print(F("positionDifference: ")); SerialDebug.print(positionDifference); SerialDebug.print('\t');
    // SerialDebug.print(F("targetPosition: ")); SerialDebug.print(targetPosition); SerialDebug.print('\t');
    // SerialDebug.print(F("hallEffectSensor: ")); SerialDebug.print(hallEffectSensor); SerialDebug.print('\t');
    // SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); SerialDebug.println('\t');
  #endif
  
  #ifdef debugHALLFull
  
    SerialDebug.print(F("hallEffectSensor_Pin: ")); SerialDebug.print(hallEffectSensor_Pin); SerialDebug.print('\t');
    SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
    SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
    SerialDebug.print(F("leftStickX: ")); SerialDebug.print(receiveFromESP32Data.leftStickX); SerialDebug.print('\t');
    SerialDebug.print(F("R3: ")); SerialDebug.print(receiveFromESP32Data.moveR3); SerialDebug.print('\t');
    SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); SerialDebug.println('\t');
      
  #endif
  
  #ifdef debugHALL
    int domecenter = digitalRead(hallEffectSensor_Pin);
    SerialDebug.print(F("domecenter? (0=true): ")); Serial.print(domecenter); SerialDebug.println('\t');
  #endif

  #ifdef debugENC
    newPosition = myEnc.read();
    // encPos = myEnc.read(); 
    int val = encoder_direction();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      // SerialDebug.print(F("domePrevPWM: "));SerialDebug.print(domePrevPWM); // Print the direction (1 for clockwise, -1 for counterclockwise)
      // SerialDebug.print(" "); // Print the direction (1 for clockwise, -1 for counterclockwise)
      // SerialDebug.print(F("ENC(old): ")); SerialDebug.print(oldPosition); SerialDebug.print('\t');
      // SerialDebug.print(F("ENC(new): ")); SerialDebug.print(newPosition); SerialDebug.print('\t');
      // SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
      // SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
      // SerialDebug.print(F("leftStickX: ")); SerialDebug.print(receiveFromESP32Data.leftStickX); SerialDebug.print('\t');
      // SerialDebug.print(F("R3: ")); SerialDebug.print(receiveFromESP32Data.moveR3); SerialDebug.print('\t');
      // SerialDebug.print(F("domecenter: ")); SerialDebug.print(domecenter); SerialDebug.print('\t');
      // SerialDebug.print(F("domeCenterSet: ")); Serial.print(domeCenterSet); SerialDebug.println('\t');
      // SerialDebug.print(F("DIR: "));SerialDebug.print(val); // Print the direction (1 for clockwise, -1 for counterclockwise)
      // SerialDebug.print(" "); // Print the direction (1 for clockwise, -1 for counterclockwise)
      // SerialDebug.print(F("ENC(old): ")); SerialDebug.print(oldPosition); SerialDebug.print('\t');
      // SerialDebug.print(F("ENC(new): ")); SerialDebug.print(newPosition); SerialDebug.print('\t');
      // SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
      // SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
      // SerialDebug.print(F("leftStickX: ")); SerialDebug.print(receiveFromESP32Data.leftStickX); SerialDebug.print('\t');
      // SerialDebug.print(F("R3: ")); SerialDebug.print(receiveFromESP32Data.moveR3); SerialDebug.print('\t');
      // SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); SerialDebug.println('\t');
    }
    // int domecenter = digitalRead(hallEffectSensor_Pin);
    // SerialDebug.print(F("HallSensor(0/1): ")); SerialDebug.print(domecenter); SerialDebug.print('\t');
    // SerialDebug.print("domeCenterSet: "); SerialDebug.print(domeCenterSet);
    // SerialDebug.print("encPos: "); SerialDebug.print(encPos);
    // SerialDebug.print("domeServoMode: "); SerialDebug.print(domeServoMode);
    // SerialDebug.print("domeServoPWM: "); SerialDebug.print(domeServoPWM);
    // SerialDebug.print("enableDrive: "); Serial.println(enableDrive);
       
  #endif

  #ifdef debugPID
  
    int domecenter = digitalRead(hallEffectSensor_Pin); 
    SerialDebug.print(F("domecenter: ")); SerialDebug.print(domecenter); SerialDebug.print('\t');
    SerialDebug.print(F("encPos: ")); SerialDebug.print(encPos); SerialDebug.print('\t');
    SerialDebug.print(F("domeCenterSet: ")); SerialDebug.print(domeCenterSet); SerialDebug.print('\t');
    SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
    SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
    SerialDebug.print(F("leftStickX: ")); SerialDebug.print(receiveFromESP32Data.leftStickX); SerialDebug.print('\t');
    SerialDebug.print(F("domeServoPWM: ")); SerialDebug.print(domeServoPWM); SerialDebug.print('\t');
    SerialDebug.print(F("Setpoint: ")); SerialDebug.print(Setpoint_domeSpinServoPid); SerialDebug.print('\t');
    SerialDebug.print(F("Input: ")); SerialDebug.print(Input_domeSpinServoPid); SerialDebug.print('\t');
    SerialDebug.print(F("Output: ")); SerialDebug.print(Output_domeSpinServoPid); SerialDebug.print('\t');
    
    SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); SerialDebug.println('\t');
       
  #endif
  
  #ifdef debugEasyTransfer
  
     DEBUG_PRINT(F("driveEnabled: ")); DEBUG_PRINT(receiveFromESP32Data.driveEnabled); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("domeSpin: ")); DEBUG_PRINT(receiveFromESP32Data.domeSpin); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("MoveL3: ")); DEBUG_PRINT(receiveFromESP32Data.moveL3); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("moveR3: ")); DEBUG_PRINT(receiveFromESP32Data.moveR3); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("leftStickX: ")); DEBUG_PRINT(receiveFromESP32Data.leftStickX); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("leftStickY: ")); DEBUG_PRINT(receiveFromESP32Data.leftStickY); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("soundcmd;: ")); DEBUG_PRINT(receiveFromESP32Data.soundcmd); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("sndplaying;: ")); DEBUG_PRINT(sendToESP32Data.sndplaying); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("psiFlash: ")); DEBUG_PRINT(receiveFromESP32Data.psiFlash); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("pitch: ")); DEBUG_PRINT(receiveFromESP32Data.pitch); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("roll: ")); DEBUG_PRINT(receiveFromESP32Data.roll); DEBUG_PRINTLN('\t');
  
  #endif

  #ifdef printRemote
    Serial.print("\t"); Serial.print(receiveFromESP32Data.driveEnabled);
    Serial.print("\t"); Serial.print(leftStickY);
    Serial.print("\t"); Serial.print(leftStickX);
    Serial.print("\t"); Serial.print(soundcmd);
    Serial.print("\t"); Serial.print(sndplaying);
    Serial.println();
  #endif

  #ifdef debugPSI
    Serial.print(" soundcmd: "); Serial.print(soundcmd); 
    Serial.print("    MP3Playing: ");
    if (mp3.isPlaying() == true) {
      Serial.print("True");
      } else  Serial.print("False");
    Serial.print(" sndplaying: ");
    Serial.print(sndplaying);  Serial.print("  "); 
    Serial.println();
  #endif

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
