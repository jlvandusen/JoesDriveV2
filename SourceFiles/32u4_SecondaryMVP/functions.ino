
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
    domeServoMode = receiveFromESP32Data.moveR3;
    domeServoPWM = map(receiveFromESP32Data.domeSpin,-127,127,-255,255);
    reverseDrive = receiveFromESP32Data.moveL3;
    sendESP32.sendData();

  }
}

boolean encoder_direction() {
  boolean encoderA = digitalRead(motorEncoder_pin_A);
  if ((aLastState == HIGH) && (encoderA == LOW)) {
    if (digitalRead(motorEncoder_pin_B)) {
      encoderPos--;  // Counterclockwise rotation
    } else {
      encoderPos++;  // Clockwise rotation
    }
    return encoderPos;
  }
  aLastState = encoderA;
}

    
//void psiTime(){
//  if (receiveFromESP32Data.psiFlash) {
//    psiValue = map(((analogRead(psiSensor_pin_L) + analogRead(psiSensor_pin_R)) / 2),0,1000,0,100);
//  } else {
//    psiValue - 0; 
//  }
//}
