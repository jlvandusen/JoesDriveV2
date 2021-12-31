void debugRoutines(){

  #ifdef debugDOME
  
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
  
  #ifdef debugHALLFull
  
    SerialDebug.print(F("hallEffectSensor_Pin: ")); SerialDebug.print(hallEffectSensor_Pin); SerialDebug.print('\t');
    SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
    SerialDebug.print(F("domeServoMode: ")); SerialDebug.print(domeServoMode); SerialDebug.print('\t');
    SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
    SerialDebug.print(F("leftStickX: ")); SerialDebug.print(receiveFromESP32Data.leftStickX); SerialDebug.print('\t');
    SerialDebug.print(F("domeSpin: ")); SerialDebug.print(receiveFromESP32Data.domeSpin); SerialDebug.print('\t');
    SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); SerialDebug.println('\t');
      
  #endif
  
  #ifdef debugHALL
    int domecenter = digitalRead(hallEffectSensor_Pin);
    SerialDebug.print(F("domecenter? (0=true): ")); Serial.print(domecenter); SerialDebug.println('\t');
  #endif
  
  #ifdef debugEasyTransfer
  
     SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); Serial.print("\t"); 
     SerialDebug.print(F("domeServoMode: ")); Serial.print(domeServoMode); Serial.print("\t"); 
     SerialDebug.print(F("MoveL3: ")); Serial.print(receiveFromESP32Data.moveL3); Serial.print("\t"); 
     SerialDebug.print(F("MoveR3: ")); Serial.print(receiveFromESP32Data.moveR3); Serial.print("\t"); 
     SerialDebug.print(F("domeSpin: ")); Serial.print(receiveFromESP32Data.domeSpin); Serial.print("\t"); 
     SerialDebug.print(F("leftStickY: ")); Serial.print(receiveFromESP32Data.leftStickY); Serial.print("\t"); 
     SerialDebug.print(F("leftStickX: ")); Serial.print(receiveFromESP32Data.leftStickX); Serial.print("\t"); 
     SerialDebug.print(F("domeSpin: ")); Serial.print(receiveFromESP32Data.domeSpin); Serial.print("\t"); 
     SerialDebug.print(F("domeSpin: ")); Serial.print(receiveFromESP32Data.domeSpin); Serial.print("\t"); 
     SerialDebug.print(F("enableDrive: ")); Serial.print(enableDrive); SerialDebug.println('\t');
  
  #endif

#ifdef printRemote
  Serial.print("\t"); Serial.print(receiveFromESP32Data.driveEnabled);
  Serial.print("\t"); Serial.print(leftStickY);
  Serial.print("\t"); Serial.print(leftStickX);
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


#ifdef debugPSI
  Serial.print("L: "); Serial.print(analogRead(psiSensor_pin_L));
  Serial.println("    R: "); Serial.println(analogRead(psiSensor_pin_R));
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
