void BatteryLevel(){
  Serial.print("Nav1 Controller battery level is: ");
  switch (driveController.state.status.battery){
    case PSController::kHigh:
    Serial.println("High");
    break;
    case PSController::kFull:
    Serial.println("High");
    break;
    case PSController::kCharging:
    Serial.println("Charging");
    break;
    case PSController::kLow:
    Serial.println("Low");
    break;
    case PSController::kDying:
    Serial.println("Critical");
    break;
    case PSController::kShutdown:
    Serial.println("Shutting Down");
    break;
  }
}

   //------------------ Battery -------------------------
void batterycheck() {
  if (driveController.isConnected()) {
    if (driveController.state.status.battery == PSController::kCharging ) Serial.println("The drive controller battery charging");
    else if (driveController.state.status.battery == PSController::kFull ) Serial.println("The drive controller battery charge is FULL");
    else if (driveController.state.status.battery == PSController::kHigh ) Serial.println("The drive controller battery charge is HIGH");
    else if (driveController.state.status.battery == PSController::kLow ) Serial.println("The drive controller battery charge is LOW");
    else if (driveController.state.status.battery == PSController::kDying ) Serial.println("The drive controller battery charge is DYING");
    else if (driveController.state.status.battery == PSController::kShutdown ) Serial.println("The drive controller battery charge is SHUTDOWN");
    else {
      Serial.println("Checking drive controller battery charge");
      batterycheck();
    }
  }
    if (domeController.isConnected()) {
    if (domeController.state.status.battery == PSController::kCharging ) Serial.println("The dome controller battery charging");
    else if (domeController.state.status.battery == PSController::kFull ) Serial.println("The dome controller battery charge is FULL");
    else if (domeController.state.status.battery == PSController::kHigh ) Serial.println("The dome controller battery charge is HIGH");
    else if (domeController.state.status.battery == PSController::kLow ) Serial.println("The dome controller battery charge is LOW");
    else if (domeController.state.status.battery == PSController::kDying ) Serial.println("The dome controller battery charge is DYING");
    else if (domeController.state.status.battery == PSController::kShutdown ) Serial.println("The dome controller battery charge is SHUTDOWN");
    else {
      Serial.println("Checking dome controller battery charge");
      batterycheck();
    }
  }
}

void setOffsetsONLY() {
  pitchOffset = 0 - receiveIMUData.pitch;
  rollOffset = 0 - receiveIMUData.roll;
  #ifndef revS2S
  potOffsetS2S = 0 - (map(analogRead(S2SPot_pin), 0, 4095, 255,-255));
  #else
  potOffsetS2S = 0 - (map(analogRead(S2SPot_pin), 0, 4095, -255,255));
  #endif
  delay(1000);
}

void setOffsetsAndSaveToEEPROM() {
  // pitchOffset = receiveIMUData.pitch * -1;
  // rollOffset = receiveIMUData.roll * -1;
  // if (Input3 > Drive_IMUDeadzone) {
    if (receiveIMUData.roll > Drive_IMUDeadzone) {
    // pitchOffset = Input3;
    pitchOffset = receiveIMUData.pitch * -1;
  } else if (receiveIMUData.roll < Drive_IMUDeadzone) {
    // pitchOffset = Input3 * -1 ;
    pitchOffset = receiveIMUData.pitch ; 
  } else pitchOffset = 0;

  if (receiveIMUData.roll > S2S_potDeadzone) {
    rollOffset = receiveIMUData.roll;
  } else if (receiveIMUData.roll <-S2S_potDeadzone) {
    rollOffset = receiveIMUData.roll * -1 ;
  } else rollOffset = 0;
  
  if(pitchOffset != preferences.getFloat("pitchOffset", 0)){
    preferences.putFloat("pitchOffset", pitchOffset);
  }
  if(rollOffset != preferences.getFloat("rollOffset", 0)){
    preferences.putFloat("rollOffset", rollOffset);
  }
  #ifndef revS2S
  potOffsetS2S = 0 - (map(analogRead(S2SPot_pin), 0, 4095, 255,-255));
  #else
  potOffsetS2S = 0 - (map(analogRead(S2SPot_pin), 0, 4095, -255,255));
  #endif
  if(potOffsetS2S != preferences.getInt("potOffsetS2S", 0)) {
    preferences.putInt("potOffsetS2S", potOffsetS2S);
  }
  DEBUG_PRINT(F("Offsets written: ")); DEBUG_PRINTLN(enableDrive); 
  delay(1000);
}


void wipenvram() {
  preferences.clear();
  rollOffset = 0;
  pitchOffset = 0;
  potOffsetS2S = 0;
  // nvs_flash_erase(); // erase the NVS partition and...
  // nvs_flash_init(); // initialize the NVS partition.
  // while(true);
}