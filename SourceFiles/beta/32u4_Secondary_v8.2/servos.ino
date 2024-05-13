
void Servos() {
/*
* Controls the tilt and triangular positioning of the dome mast
* Based on James Bruton's excellent examples on triangulation and positioning using 2 BB sized Servos.
*/
  int domeTurnPercent; 
  int y_Axis, x_Axis; 

  #ifdef checksumValidation
  // Assuming you have received the data in receiveFromESP32Data
  // uint8_t receivedChecksum = receiveFromESP32Data.leftStickX;  // Extract the received checksum
  // Extract the received checksum (assuming it's the last byte received)
  uint8_t receivedChecksum = Serial1.read();  // Read the checksum byte
  // Calculate the local checksum
  uint8_t localChecksum = 0;
  localChecksum += receiveFromESP32Data.leftStickX;
  localChecksum += receiveFromESP32Data.leftStickY;
  // Add other relevant fields to the local checksum if needed
  if (receivedChecksum == localChecksum) {
    if(enableDrive) {
        x_Axis = receiveFromESP32Data.leftStickX;
        y_Axis = receiveFromESP32Data.leftStickY;
      // }
    
      if(reverseDrive){
        y_Axis *= -1;
        x_Axis *= -1;
      }
      
      if(y_Axis < 0) { //Scales the Y values from -100/100 to 40deg toward the 'back' and 35deg toward the 'front'
        // leftStickY = map(y_Axis,-100,0,-domeTiltYAxis_MaxAngle,0);
        leftStickY = map(y_Axis,-127,127,-domeTiltYAxis_MaxAngle,0);
      } else if(y_Axis > 0) {
        // leftStickY = map(y_Axis,0,100,0,domeTiltYAxis_MaxAngle);
        leftStickY = map(y_Axis,-127,127,0,domeTiltYAxis_MaxAngle);
      } else {
        leftStickY = 0;
      }


      if(x_Axis < 0){ //Scales the X values from -100/100 to 29def left/right
        // leftStickX  = map(x_Axis,-100,0,-domeTiltXAxis_MaxAngle,0);
        leftStickX  = map(x_Axis,-127,127,-domeTiltXAxis_MaxAngle,0);
      }else if(x_Axis > 0){
        // leftStickX  = map(x_Axis,0,100,0,domeTiltXAxis_MaxAngle);
        leftStickX  = map(x_Axis,-127,127,0,domeTiltXAxis_MaxAngle);
      }else{
        leftStickX  = 0;
      }


      if(currentMillis - lastServoUpdateMillis >= 10) { //Eases Y values by adding/subtracting the ease values every 10 millis
        lastServoUpdateMillis = currentMillis; 
        if(leftStickY < domeTiltAngle_Y_Axis - servoEase) {
          domeTiltAngle_Y_Axis -= servoEase; 
        } else if(leftStickY > domeTiltAngle_Y_Axis + servoEase) {
          domeTiltAngle_Y_Axis += servoEase; 
        } else {
        domeTiltAngle_Y_Axis = leftStickY;
        }

        
        if(leftStickX < domeTiltAngle_X_Axis - servoEase) { //Eases X values by adding/subtracting the ease values every 10 millis
          domeTiltAngle_X_Axis -= servoEase; 
        } else if(leftStickX > domeTiltAngle_X_Axis + servoEase) {
          domeTiltAngle_X_Axis += servoEase; 
        } else {
        domeTiltAngle_X_Axis = leftStickX;
        }
      }
        
      if(domeTiltAngle_Y_Axis < 0) { //Turns the scaled angles from the y axis to servo positions
        leftServoPosition = leftServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch), -40,0,55,0);
        rightServoPosition = rightServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch), -40,0,-55,0);
        
      } else if(domeTiltAngle_Y_Axis > 0) {
        leftServoPosition = leftServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch, 0,35,0,-50);
        rightServoPosition = rightServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch, 0,35,0,50);
      } else {
        if(receiveFromESP32Data.pitch <= 0) {
          leftServoPosition = leftServo_0_Position - map(receiveFromESP32Data.pitch,-40,0,55,0);
          rightServoPosition = rightServo_0_Position + map(receiveFromESP32Data.pitch,-40,0,55,0);
        } else {
          leftServoPosition = leftServo_0_Position - map(receiveFromESP32Data.pitch,0,35,0,-50);
          rightServoPosition = rightServo_0_Position + map(receiveFromESP32Data.pitch,0,35,0,-50);
        } 
      }

        
      if(domeTiltAngle_X_Axis < 0) { //Turns the scaled angles from the x axis to servo positions and adds them to what we have from Y
        leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,30,0);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,50,0 );
        
      } else if(domeTiltAngle_X_Axis > 0) {
        leftServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-50)+map(receiveFromESP32Data.roll,0,29,0,-30));
        rightServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-30)+map(receiveFromESP32Data.roll,0,29,0,-50));
        
      } else {
        if(receiveFromESP32Data.roll <= 0) {
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,30,0);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,50,0 );
        } else {
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,0,29,0,-30);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,0,29,0,-50);
        } 
      }


      leftDifference = abs(leftOldPosition - leftServoPosition); 
      rightDifference = abs(rightOldPosition - rightServoPosition); 
      
      if(leftDifference > rightDifference){
        if(leftOldPosition < leftServoPosition){ 
          leftOldPosition ++; 
        }else if(leftOldPosition > leftServoPosition){
          leftOldPosition --; 
        }

        if(rightOldPosition < rightServoPosition){
          rightOldPosition += rightDifference / leftDifference;
        }else{
          rightOldPosition -= rightDifference / leftDifference;
        }
        
      }else {
        if(rightOldPosition < rightServoPosition) { 
          rightOldPosition ++; 
        } else if (rightOldPosition > rightServoPosition) {
          rightOldPosition --; 
        }

        if(leftOldPosition < leftServoPosition) {
          leftOldPosition += leftDifference / rightDifference;
        } else if (leftOldPosition > leftServoPosition) {
          leftOldPosition -= leftDifference / rightDifference;
        }  
      }
          
      myservo2.write(constrain(leftOldPosition,leftServo_0_Position-45,leftServo_0_Position+55), servoSpeed);      //Sends positions to the servos
      myservo1.write(constrain(rightOldPosition,rightServo_0_Position-55,rightServo_0_Position+45), servoSpeed);   //Sends positions to the servos
    }
  } else {
    // Data is corrupted; handle accordingly (e.g., request retransmission)
    // ...
    SerialDebug.print(F("InvalidChecksum: ")); SerialDebug.print(receivedChecksum); SerialDebug.print('\t');
    SerialDebug.print(F("receivedChecksum: ")); SerialDebug.print(receivedChecksum); SerialDebug.print('\t');
    SerialDebug.print(F("localChecksum: ")); SerialDebug.print(localChecksum); SerialDebug.println('\t');
  }
  #else
  if(enableDrive) {
    x_Axis = receiveFromESP32Data.leftStickX;
    y_Axis = receiveFromESP32Data.leftStickY;
    if(reverseDrive){
      y_Axis *= -1;
      x_Axis *= -1;
    }
      
      if(y_Axis < 0) { //Scales the Y values from -100/100 to 40deg toward the 'back' and 35deg toward the 'front'
        // leftStickY = map(y_Axis,-100,0,-domeTiltYAxis_MaxAngle,0);
        leftStickY = map(y_Axis,-127,127,-domeTiltYAxis_MaxAngle,0);
      } else if(y_Axis > 0) {
        // leftStickY = map(y_Axis,0,100,0,domeTiltYAxis_MaxAngle);
        leftStickY = map(y_Axis,-127,127,0,domeTiltYAxis_MaxAngle);
      } else {
        leftStickY = 0;
      }


      if(x_Axis < 0){ //Scales the X values from -100/100 to 29def left/right
        // leftStickX  = map(x_Axis,-100,0,-domeTiltXAxis_MaxAngle,0);
        leftStickX  = map(x_Axis,-127,127,-domeTiltXAxis_MaxAngle,0);
      }else if(x_Axis > 0){
        // leftStickX  = map(x_Axis,0,100,0,domeTiltXAxis_MaxAngle);
        leftStickX  = map(x_Axis,-127,127,0,domeTiltXAxis_MaxAngle);
      }else{
        leftStickX  = 0;
      }


      if(currentMillis - lastServoUpdateMillis >= 10) { //Eases Y values by adding/subtracting the ease values every 10 millis
        lastServoUpdateMillis = currentMillis; 
        if(leftStickY < domeTiltAngle_Y_Axis - servoEase) {
          domeTiltAngle_Y_Axis -= servoEase; 
        } else if(leftStickY > domeTiltAngle_Y_Axis + servoEase) {
          domeTiltAngle_Y_Axis += servoEase; 
        } else {
        domeTiltAngle_Y_Axis = leftStickY;
        }

        
        if(leftStickX < domeTiltAngle_X_Axis - servoEase) { //Eases X values by adding/subtracting the ease values every 10 millis
          domeTiltAngle_X_Axis -= servoEase; 
        } else if(leftStickX > domeTiltAngle_X_Axis + servoEase) {
          domeTiltAngle_X_Axis += servoEase; 
        } else {
        domeTiltAngle_X_Axis = leftStickX;
        }
      }
        
      if(domeTiltAngle_Y_Axis < 0) { //Turns the scaled angles from the y axis to servo positions
        leftServoPosition = leftServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch), -40,0,55,0);
        rightServoPosition = rightServo_0_Position + map((domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch), -40,0,-55,0);
        
      } else if(domeTiltAngle_Y_Axis > 0) {
        leftServoPosition = leftServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch, 0,35,0,-50);
        rightServoPosition = rightServo_0_Position + map(domeTiltAngle_Y_Axis-receiveFromESP32Data.pitch, 0,35,0,50);
      } else {
        if(receiveFromESP32Data.pitch <= 0) {
          leftServoPosition = leftServo_0_Position - map(receiveFromESP32Data.pitch,-40,0,55,0);
          rightServoPosition = rightServo_0_Position + map(receiveFromESP32Data.pitch,-40,0,55,0);
        } else {
          leftServoPosition = leftServo_0_Position - map(receiveFromESP32Data.pitch,0,35,0,-50);
          rightServoPosition = rightServo_0_Position + map(receiveFromESP32Data.pitch,0,35,0,-50);
        } 
      }

        
      if(domeTiltAngle_X_Axis < 0) { //Turns the scaled angles from the x axis to servo positions and adds them to what we have from Y
        leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,30,0);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,50,0 );
        
      } else if(domeTiltAngle_X_Axis > 0) {
        leftServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-50)+map(receiveFromESP32Data.roll,0,29,0,-30));
        rightServoPosition += (map(domeTiltAngle_X_Axis,0,29,0,-30)+map(receiveFromESP32Data.roll,0,29,0,-50));
        
      } else {
        if(receiveFromESP32Data.roll <= 0) {
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,30,0);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,-29,0,50,0 );
        } else {
          leftServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,0,29,0,-30);
        rightServoPosition += map(domeTiltAngle_X_Axis+receiveFromESP32Data.roll,0,29,0,-50);
        } 
      }


      leftDifference = abs(leftOldPosition - leftServoPosition); 
      rightDifference = abs(rightOldPosition - rightServoPosition); 
      
      if(leftDifference > rightDifference){
        if(leftOldPosition < leftServoPosition){ 
          leftOldPosition ++; 
        }else if(leftOldPosition > leftServoPosition){
          leftOldPosition --; 
        }

        if(rightOldPosition < rightServoPosition){
          rightOldPosition += rightDifference / leftDifference;
        }else{
          rightOldPosition -= rightDifference / leftDifference;
        }
        
      }else {
        if(rightOldPosition < rightServoPosition) { 
          rightOldPosition ++; 
        } else if (rightOldPosition > rightServoPosition) {
          rightOldPosition --; 
        }

        if(leftOldPosition < leftServoPosition) {
          leftOldPosition += leftDifference / rightDifference;
        } else if (leftOldPosition > leftServoPosition) {
          leftOldPosition -= leftDifference / rightDifference;
        }  
      }
        
    myservo2.write(constrain(leftOldPosition,leftServo_0_Position-45,leftServo_0_Position+55), servoSpeed);      //Sends positions to the servos
    myservo1.write(constrain(rightOldPosition,rightServo_0_Position-55,rightServo_0_Position+45), servoSpeed);   //Sends positions to the servos
  }
  #endif
}
