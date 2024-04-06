void debugRoutines(){
  #ifdef debugESPNOWSend
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
      DEBUG_PRINT("\r\nLast Packet Send Status:\t");
      DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
      if (status ==0){
        success = "Delivery Success :)";
      } else{
        success = "Delivery Fail :(";
      }
    }
  #endif

  #ifdef debugNavRemoteRight
    DEBUG_PRINT(buttonsR.rightStickY); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.rightStickX); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.l1); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.l2); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.up); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.down); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.left); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.right); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.ps); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsR.l3); DEBUG_PRINTLN("\r\n");
  #endif

  #ifdef debugNavRemoteLeft
    DEBUG_PRINT(F("32u4DomeY: ")); DEBUG_PRINT(sendTo32u4Data.leftStickY); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("32u4DomeX: ")); DEBUG_PRINT(sendTo32u4Data.leftStickX); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.leftStickY); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.leftStickX); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.l1); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.l2); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.l3); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.up); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.down); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.left); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.right); DEBUG_PRINT('\t');
    DEBUG_PRINT(buttonsL.ps); DEBUG_PRINT('\t');
  #endif

  #ifdef debugESPNOW
     DEBUG_PRINT(F("SndPlaying: ")); DEBUG_PRINT(receiveFrom32u4Data.sndplaying); DEBUG_PRINT("\t");
     DEBUG_PRINT(F("BodyPSI: ")); DEBUG_PRINT(sendPSI); DEBUG_PRINT("\t");
     DEBUG_PRINT(F("BodyHP: ")); DEBUG_PRINT(sendHP); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("BodyBAT: ")); DEBUG_PRINT(sendBAT); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("BodyDIS: ")); DEBUG_PRINT(sendDIS); DEBUG_PRINT("\t");   
     DEBUG_PRINT(F("DomePSI: ")); DEBUG_PRINT(incomingPSI); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("DomeBAT: ")); DEBUG_PRINT(incomingBAT); DEBUG_PRINTLN("\t"); 
  #endif

  #ifdef debugPreferences
  
     DEBUG_PRINT(F("Pitch: ")); DEBUG_PRINT(receiveIMUData.pitch); DEBUG_PRINT("\t");
     DEBUG_PRINT(F("Roll: ")); DEBUG_PRINT(receiveIMUData.roll); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("pitchoffset: ")); DEBUG_PRINT(pitchOffset); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("rolloffset: ")); DEBUG_PRINT(rollOffset); DEBUG_PRINT("\t");   
     DEBUG_PRINT(F("POT: ")); DEBUG_PRINT(S2S_pot); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("POToffset: ")); DEBUG_PRINT(potOffsetS2S); DEBUG_PRINTLN("\t"); 
     
  #endif

  #ifdef debugEasyTransfer
  
     DEBUG_PRINT(F("driveEnabled: ")); DEBUG_PRINT(sendTo32u4Data.driveEnabled); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("domeSpin: ")); DEBUG_PRINT(sendTo32u4Data.domeSpin); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("MoveL3: ")); DEBUG_PRINT(sendTo32u4Data.moveL3); DEBUG_PRINT("\t"); 
//     DEBUG_PRINT(F("flywheel: ")); DEBUG_PRINT(sendTo32u4Data.flywheel); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("moveR3: ")); DEBUG_PRINT(sendTo32u4Data.moveR3); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("leftStickX: ")); DEBUG_PRINT(sendTo32u4Data.leftStickX); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("leftStickY: ")); DEBUG_PRINT(sendTo32u4Data.leftStickY); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("soundcmd: ")); DEBUG_PRINT(sendTo32u4Data.soundcmd); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("psiFlash: ")); DEBUG_PRINT(sendTo32u4Data.psiFlash); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("pitch: ")); DEBUG_PRINT(sendTo32u4Data.pitch); DEBUG_PRINT("\t"); 
     DEBUG_PRINT(F("roll: ")); DEBUG_PRINT(sendTo32u4Data.roll); DEBUG_PRINTLN('\t');
  #endif
  #ifdef debugIMU
    DEBUG_PRINT("IMU CONNECTED: ");
    DEBUG_PRINT(IMUconnected);
    DEBUG_PRINT(" Input2/Roll: "); 
    DEBUG_PRINT(Input2);
    DEBUG_PRINT(" IMU ROLL: ");
    DEBUG_PRINT(sendTo32u4Data.roll);
    DEBUG_PRINT(" IMU PITCH: ");
    DEBUG_PRINTLN(sendTo32u4Data.pitch);
  #endif
  
  #ifdef debugPOTS
    double S2SDebug_pot = analogRead(S2SPot_pin);
    DEBUG_PRINT(F("Setpoint2: ")); DEBUG_PRINT(Setpoint2); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("S2S_pot: ")); DEBUG_PRINT(S2SDebug_pot); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Setpoint1/Output2: ")); DEBUG_PRINT(Setpoint1); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("diff_S2S: ")); DEBUG_PRINT(diff_S2S); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Input1: ")); DEBUG_PRINT(Input1); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Input2: ")); DEBUG_PRINT(Input2); DEBUG_PRINTLN('\t');
  #endif

  #ifdef debugS2S
    double S2SDebug_pot = analogRead(S2SPot_pin);
    DEBUG_PRINT(F("Target: ")); DEBUG_PRINT(target_pos_S2S ); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("JOYX: ")); DEBUG_PRINT(buttonsR.rightStickX); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("current_pos_S2S: ")); DEBUG_PRINT(current_pos_S2S); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("diff_S2S: ")); DEBUG_PRINT(diff_S2S); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Setpoint2: ")); DEBUG_PRINT(Setpoint2); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Input1/Pot: ")); DEBUG_PRINT(Input1); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Input2/Roll: ")); DEBUG_PRINT(Input2); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("Setpoint1/Output2: ")); DEBUG_PRINT(Setpoint1); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("PotRaw.: ")); DEBUG_PRINT(S2SDebug_pot); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("RollRaw.: ")); DEBUG_PRINT(receiveIMUData.roll); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Output1_S2S_pwm: ")); DEBUG_PRINT(Output1_S2S_pwm); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("En: ")); DEBUG_PRINTLN(enableDrive); 
  #endif

#ifdef debugS2SPot
double S2SDebug_pot = analogRead(S2SPot_pin);
 DEBUG_PRINT("S2SPOT_INPUT1_PID:"); DEBUG_PRINT(Input1); DEBUG_PRINT('\t');
 DEBUG_PRINT("S2SPOT_PWM:"); DEBUG_PRINT(S2SDebug_pot); DEBUG_PRINTLN('\t');
#endif

#ifdef debugFlywheel

    
    DEBUG_PRINT(F("EnableFly: ")); DEBUG_PRINT(EnableFlywheel); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("flywheel: ")); DEBUG_PRINT(abs(flywheel)); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("flywheelRot: ")); DEBUG_PRINT(flywheelRotation); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("Joy: ")); DEBUG_PRINT(buttonsR.rightStickX); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("ControllerConn.: ")); DEBUG_PRINT(controllerConnected); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("En: ")); DEBUG_PRINTLN(enableDrive); 
      
#endif

#ifdef debugMainDrive
    DEBUG_PRINT(F("Target: ")); DEBUG_PRINT(Setpoint_Drive_In); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("JoyY: ")); DEBUG_PRINT(buttonsR.rightStickY); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("current_pos: ")); DEBUG_PRINT(current_pos_drive); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("Setpoint3: ")); DEBUG_PRINT(Setpoint3); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Input3/Pitch: ")); DEBUG_PRINT(Input3); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("+Pitchoffset: ")); DEBUG_PRINT(pitchOffset); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Output3/Drive: ")); DEBUG_PRINT(Output3); DEBUG_PRINT('\t'); 
    DEBUG_PRINT(F("Output_Drive_pwm: ")); DEBUG_PRINT(Output_Drive_pwm); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("Contr.: ")); DEBUG_PRINT(controllerConnected); DEBUG_PRINT('\t');
    DEBUG_PRINT(F("En: ")); DEBUG_PRINTLN(enableDrive); 
#endif 
}
