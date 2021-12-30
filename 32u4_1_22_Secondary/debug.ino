void debugRoutines(){

#ifdef debugEasyTransfer

  Serial.print("\t"); Serial.print(receiveFromESP32Data.driveEnabled);
  Serial.print("\t"); Serial.print(receiveFromESP32Data.leftStickX);
  Serial.print("\t"); Serial.print(receiveFromESP32Data.leftStickY);
  Serial.print("\t"); Serial.print(receiveFromESP32Data.pitch);
  Serial.print("\t"); Serial.println(receiveFromESP32Data.roll);

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
