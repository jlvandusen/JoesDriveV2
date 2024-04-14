
void Timechecks() {
  currentMillis = millis();
  receiveMillis = currentMillis;
  if (currentMillis - lastPrintMillis >= printMillis) {  // Check for last debug print, if past, print debugs if enabled.
    lastPrintMillis = currentMillis;
    debugRoutines();
  }
  if (currentMillis - receiveMillis >= 1000 && enableDrive) {
  enableDrive = !enableDrive;
  }
}

void SendRecieveData() {
  if (recESP32.receiveData()) {
    receiveMillis = currentMillis;
    enableDrive = receiveFromESP32Data.driveEnabled;
    reverseDrive = receiveFromESP32Data.moveR3;
    domeServoMode = receiveFromESP32Data.moveL3;
    domeServoPWM = map(receiveFromESP32Data.domeSpin,-127,127,-255,255);
    soundcmd = receiveFromESP32Data.soundcmd;
    sndplaying = sendToESP32Data.sndplaying;
      
    #ifdef EnableFilters
      enableDriveFiltered = adcFilterdriveEnabled.filter(enableDrive);
      domeServoModeFiltered = adcFiltermoveL3.filter(domeServoMode);
      reverseDriveFiltered = adcFiltermoveR3.filter(reverseDrive);
      domeServoPWMFiltered = adcFilterdomeSpin.filter(domeServoPWM);
      soundcmd = adcFiltersoundcmd.filter(soundcmd);
      sndplaying = sendToESP32Data.sndplaying;
      
      enableDrive = enableDriveFiltered;
      domeServoMode = domeServoModeFiltered;
      reverseDrive = reverseDriveFiltered;
      domeServoPWM = domeServoPWMFiltered;
    #endif

    sendESP32.sendData();
  }
}



    
//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}
