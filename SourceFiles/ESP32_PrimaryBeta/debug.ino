  void debugRoutines(){
  
  #ifdef debugNavRemoteRight
    SerialDebug.print(buttonsR.rightStickY); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.rightStickX); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.l1); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.l2); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.up); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.down); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.left); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.right); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.ps); SerialDebug.print('\t');
    SerialDebug.print(buttonsR.l3); SerialDebug.print("\r\n");
  #endif

    #ifdef debugNavRemoteLeft
    
    SerialDebug.print(F("32u4DomeY: ")); SerialDebug.print(sendTo32u4Data.leftStickY); SerialDebug.print('\t');
    SerialDebug.print(F("32u4DomeX: ")); SerialDebug.print(sendTo32u4Data.leftStickX); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.leftStickY); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.leftStickX); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.l1); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.l2); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.l3); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.up); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.down); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.left); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.right); SerialDebug.print('\t');
    SerialDebug.print(buttonsL.ps); SerialDebug.print('\t');
//    SerialDebug.print(sendTo32u4Data.xboxL3); SerialDebug.print('\t');
//    SerialDebug.print(buttonsR.r1); SerialDebug.print('\t');
//    SerialDebug.print(buttonsR.r2); SerialDebug.print('\t');
//    SerialDebug.print(sendTo32u4Data.xboxR3); SerialDebug.print('\t');
//    SerialDebug.print(buttons.a); SerialDebug.print('\t');
//    SerialDebug.print(buttons.b); SerialDebug.print('\t');
//    SerialDebug.print(buttons.x); SerialDebug.print('\t');
//    SerialDebug.print(buttons.y); SerialDebug.print('\t');
    
//    SerialDebug.print(buttonsL.back); SerialDebug.print('\t');
//    SerialDebug.print(enableDrive); SerialDebug.print('\t');
//    SerialDebug.println(buttons.xbox); 
    
  #endif

  #ifdef debugESPNOW
     SerialDebug.print(F("BodyPSI: ")); SerialDebug.print(sendPSI); SerialDebug.print("\t");
     SerialDebug.print(F("BodyHP: ")); SerialDebug.print(sendHP); SerialDebug.print("\t"); 
     SerialDebug.print(F("BodyBAT: ")); SerialDebug.print(sendBAT); SerialDebug.print("\t"); 
     SerialDebug.print(F("BodyDIS: ")); SerialDebug.print(sendDIS); SerialDebug.print("\t");   
     SerialDebug.print(F("DomePSI: ")); SerialDebug.print(incomingPSI); SerialDebug.print("\t"); 
     SerialDebug.print(F("DomeBAT: ")); SerialDebug.print(incomingBAT); SerialDebug.println("\t"); 
     
  #endif

  #ifdef debugPreferences
  
     SerialDebug.print(F("Pitch: ")); SerialDebug.print(receiveIMUData.pitch); SerialDebug.print("\t");
     SerialDebug.print(F("Roll: ")); SerialDebug.print(receiveIMUData.roll); SerialDebug.print("\t"); 
     SerialDebug.print(F("pitchoffset: ")); SerialDebug.print(pitchOffset); SerialDebug.print("\t"); 
     SerialDebug.print(F("rolloffset: ")); SerialDebug.print(rollOffset); SerialDebug.print("\t");   
     SerialDebug.print(F("POT: ")); SerialDebug.print(S2S_pot); SerialDebug.print("\t"); 
     SerialDebug.print(F("POToffset: ")); SerialDebug.print(potOffsetS2S); SerialDebug.println("\t"); 
     
  #endif

  #ifdef debugEasyTransfer
  
     SerialDebug.print(F("driveEnabled: ")); Serial.print(sendTo32u4Data.driveEnabled); Serial.print("\t"); 
     SerialDebug.print(F("domeSpin: ")); Serial.print(sendTo32u4Data.domeSpin); Serial.print("\t"); 
     SerialDebug.print(F("MoveL3: ")); Serial.print(sendTo32u4Data.moveL3); Serial.print("\t"); 
//     SerialDebug.print(F("flywheel: ")); Serial.print(sendTo32u4Data.flywheel); Serial.print("\t"); 
     SerialDebug.print(F("moveR3: ")); Serial.print(sendTo32u4Data.moveR3); Serial.print("\t"); 
     SerialDebug.print(F("leftStickX: ")); Serial.print(sendTo32u4Data.leftStickX); Serial.print("\t"); 
     SerialDebug.print(F("leftStickY: ")); Serial.print(sendTo32u4Data.leftStickY); Serial.print("\t"); 
     SerialDebug.print(F("soundcmd: ")); Serial.print(sendTo32u4Data.soundcmd); Serial.print("\t"); 
     SerialDebug.print(F("psiFlash: ")); Serial.print(sendTo32u4Data.psiFlash); Serial.print("\t"); 
     SerialDebug.print(F("pitch: ")); Serial.print(sendTo32u4Data.pitch); Serial.print("\t"); 
     SerialDebug.print(F("roll: ")); Serial.print(sendTo32u4Data.roll); SerialDebug.println('\t');
  #endif

  #ifdef debugIMU
    SerialDebug.print(IMUconnected); SerialDebug.print('\t');
    SerialDebug.print(F("pitch.: ")); SerialDebug.print(receiveIMUData.pitch); SerialDebug.print('\t');
    SerialDebug.print(F("roll.: ")); SerialDebug.print(receiveIMUData.roll); SerialDebug.print('\t');
    SerialDebug.print(F("Input2/Roll: ")); SerialDebug.print(Input2); SerialDebug.print('\t'); 
  #endif
  
  #ifdef debugPOTS
    double S2SDebug_pot = analogRead(S2SPot_pin);
    SerialDebug.print(F("Setpoint2: ")); SerialDebug.print(Setpoint2); SerialDebug.print('\t');
    SerialDebug.print(F("S2S_pot: ")); SerialDebug.print(S2SDebug_pot); SerialDebug.print('\t');
    SerialDebug.print(F("Setpoint1/Output2: ")); SerialDebug.print(Setpoint1); SerialDebug.print('\t');
    SerialDebug.print(F("diff_S2S: ")); SerialDebug.print(diff_S2S); SerialDebug.print('\t');
    SerialDebug.print(F("Input1: ")); SerialDebug.print(Input1); SerialDebug.print('\t');
    SerialDebug.print(F("Input2: ")); SerialDebug.print(Input2); SerialDebug.println('\t');
  #endif

  #ifdef debugS2S
    double S2SDebug_pot = analogRead(S2SPot_pin);
    SerialDebug.print(F("Target: ")); SerialDebug.print(target_pos_S2S ); SerialDebug.print('\t'); 
    SerialDebug.print(F("JOYX: ")); SerialDebug.print(buttonsR.rightStickX); SerialDebug.print('\t'); 
    SerialDebug.print(F("current_pos_S2S: ")); SerialDebug.print(current_pos_S2S); SerialDebug.print('\t'); 
    SerialDebug.print(F("diff_S2S: ")); SerialDebug.print(diff_S2S); SerialDebug.print('\t');
    SerialDebug.print(F("Setpoint2: ")); SerialDebug.print(Setpoint2); SerialDebug.print('\t');
    SerialDebug.print(F("Input1/Pot: ")); SerialDebug.print(Input1); SerialDebug.print('\t');
    SerialDebug.print(F("Input2/Roll: ")); SerialDebug.print(Input2); SerialDebug.print('\t'); 
    SerialDebug.print(F("Setpoint1/Output2: ")); SerialDebug.print(Setpoint1); SerialDebug.print('\t');
    SerialDebug.print(F("PotRaw.: ")); SerialDebug.print(S2SDebug_pot); SerialDebug.print('\t');
    SerialDebug.print(F("RollRaw.: ")); SerialDebug.print(receiveIMUData.roll); SerialDebug.print('\t');
    SerialDebug.print(F("Output1_S2S_pwm: ")); SerialDebug.print(Output1_S2S_pwm); SerialDebug.print('\t');
    SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
  #endif

#ifdef debugS2SPot
double S2SDebug_pot = analogRead(S2SPot_pin);
 SerialDebug.print("S2SPOT_INPUT1_PID:"); SerialDebug.print(Input1); SerialDebug.print('\t');
 SerialDebug.print("S2SPOT_PIN:"); SerialDebug.print(S2SPot_pin); SerialDebug.print('\t');
 SerialDebug.print("S2SPOT_PWM:"); SerialDebug.print(S2SDebug_pot); SerialDebug.println('\t');
#endif

#ifdef debugFlywheel

    
    SerialDebug.print(F("EnableFly: ")); SerialDebug.print(EnableFlywheel); SerialDebug.print('\t'); 
    SerialDebug.print(F("flywheel: ")); SerialDebug.print(abs(flywheel)); SerialDebug.print('\t'); 
    SerialDebug.print(F("flywheelRot: ")); SerialDebug.print(flywheelRotation); SerialDebug.print('\t'); 
    SerialDebug.print(F("Joy: ")); SerialDebug.print(buttonsR.rightStickX); SerialDebug.print('\t'); 
    SerialDebug.print(F("ControllerConn.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
    SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
      
#endif

#ifdef debugMainDrive

    SerialDebug.print(F("Target: ")); SerialDebug.print(target_pos_drive); SerialDebug.print('\t'); 
    SerialDebug.print(F("JOYX: ")); SerialDebug.print(buttonsR.rightStickY); SerialDebug.print('\t'); 
    SerialDebug.print(F("current_pos: ")); SerialDebug.print(current_pos_drive); SerialDebug.print('\t'); 
    SerialDebug.print(F("Setpoint3: ")); SerialDebug.print(Setpoint3); SerialDebug.print('\t');
    SerialDebug.print(F("Input3/Pitch: ")); SerialDebug.print(Input3); SerialDebug.print('\t');
    SerialDebug.print(F("+Pitchoffset: ")); SerialDebug.print(pitchOffset); SerialDebug.print('\t');
    SerialDebug.print(F("Output3/Roll: ")); SerialDebug.print(Output3); SerialDebug.print('\t'); 
    SerialDebug.print(F("Output_Drive_pwm: ")); SerialDebug.print(Output_Drive_pwm); SerialDebug.print('\t');
    SerialDebug.print(F("Contr.: ")); SerialDebug.print(controllerConnected); SerialDebug.print('\t');
    SerialDebug.print(F("En: ")); SerialDebug.println(enableDrive); 
      
#endif

     
     
  
  }
