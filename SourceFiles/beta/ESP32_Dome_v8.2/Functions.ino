
/*  
 *  ESPNOW Callback when data is received
 *  See walkthrough: https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
*/
 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  #ifdef debugRecieveESPNOW
  Serial.print("Bytes received: ");
  Serial.println(len);
  #endif
  incomingPSI = incomingReadings.psi;
  incomingBTN = incomingReadings.btn;
  incomingBAT = incomingReadings.bat;
  // Check if the incomingPSI value has changed to a non-zero value
  if (incomingPSI != 0 && !shouldFlicker) {
    // Start the flickering process
    shouldFlicker = true;
    flickerEndTime = millis() + random(1000, 3000);
  }

}

/* 
 *  ESPNOW Callback when data is sent
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef debugSendESPNOW
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  #endif 
}

/*  
 *  LED Controls for light types
 *  
*/
void displayPSI () {
//  if (incomingPSI != 0 && !flicker) {
//    // Start flickering when PSI is not 0
//    flicker = true;
//    startTime = millis();
//  } else if (incomingPSI == 0 && flicker) {
//    // Stop flickering when PSI is 0
//    flicker = false;
//    PSI.clear();
//    PSI.show();
//  }
//  if (flicker) {
//    // Continue flickering for the timeframe even when the incomingPSI is 0
//    unsigned long currentTime = millis();
//    if (currentTime - startTime >= random(1000, 3000)) {
//      startTime = currentTime;
//      flickerDisplay();
//    }
//  }
  // Check if we should start flickering
  if (incomingPSI != 0 && !shouldFlicker) {
    shouldFlicker = true;
    flickerEndTime = millis() + random(1000, 3000); // Set the end time for flickering
  }

  // Handle the flickering
  if (shouldFlicker) {
    if (millis() <= flickerEndTime) {
      // Flicker the display
      flickerDisplay();
    } else {
      // Stop flickering and clear the display
      shouldFlicker = false;
      PSI.clear();
      PSI.show();
    }
  }
}

void displayEYE () {
  // Control the EYE: Stay red and randomly dim and go bright
  if (millis() - EYELastChange > EYEChangeInterval) {
    int eyeBrightness = random(0, 256); // Random brightness between 0 and 255
    EYE.setPixelColor(0, EYE.Color(eyeBrightness, 0, 0)); // Red color with random brightness
    EYE.show();
    EYELastChange = millis();
    EYEChangeInterval = random(3000, 10000);
  }
}

void displayHP () {
    // Control the HP: Stay white at 50% brightness
  HP.setPixelColor(0, HP.Color(128, 128, 128)); // White color at 50% brightness
  HP.show();
}

void displaysLOGIC() {
  
  // Control the sLOGIC: Change various colors randomly
//  int sLogicColor = random(0, 16777216); // Random 24-bit color
//  lLOGIC.setPixelColor(0, sLogicColor);
//  lLOGIC.show();
  if (millis() - sLogicLastChange > sLogicChangeInterval) {
    int sLogicColor = random(0, 16777216); // Random 24-bit color
    sLOGIC.setPixelColor(0, sLogicColor);
    sLOGIC.show();
    sLogicLastChange = millis();
    sLogicChangeInterval = random(3000, 10000);
  }
}

void displaylLOGIC() {
    // Control the lLOGIC: Change various colors randomly
  if (millis() - lLogicLastChange > lLogicChangeInterval) {
    int lLogicColor = random(0, 16777216); // Random 24-bit color
    lLOGIC.setPixelColor(0, lLogicColor);
    lLOGIC.show();
    lLogicLastChange = millis();
    lLogicChangeInterval = random(3000, 10000);
  }
}



void flickerDisplay() {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    // Generate random white level (adjust as needed)
    int whiteLevel = random(0, 256);
    PSI.setPixelColor(pixel, PSI.Color(whiteLevel, whiteLevel, whiteLevel));
  }
  PSI.show(); // Apply the changes
}


//void sendAndReceive(){
//  if(millis() - lastSendRecMillis >= recDelay){
//    // PSILED(); 
//    lastBodyReceive = millis();
//  }
//  lastSendRecMillis = millis(); 
//  // battLevel();
//}
