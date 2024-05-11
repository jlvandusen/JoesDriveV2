


//void encoder() {
//  encPos = myEnc.read(); 
//
//  if (domeServoMode && !moveR3Was) {
//    moveR3Was = true;
//    myEnc.write(encPos - 1497/2); //myEnc.write(encPos - 1497/2); 1680
//    
//  } else if (!domeServoMode && moveR3Was) {
//    moveR3Was = false;
//    myEnc.write(encPos + 1497/2); // 1680
//  }
//  if (encPos > 1497) {
//    encPos -= 1497; 
//    myEnc.write(encPos);
//  } else if (encPos < 0) {
//    encPos += 1497; 
//    myEnc.write(encPos);
//  }
//
//  if (reverseDrive && r3Flag == false) {
//    r3Flag = true; 
//    domeServoMode = !domeServoMode; 
//  } else if (!reverseDrive && r3Flag == true) {
//    r3Flag = false; 
//  }
//  
//}

//void readEncoder() {
//  aState = digitalRead(motorEncoder_pin_A);
//  bState = digitalRead(motorEncoder_pin_B);
//
//  if (aState != bState) {
//    encoderPosition++;
//  } else {
//    encoderPosition--;
//  }
//}
//  
///*
// *   Turn the dome spin clockwise till it finds the hallsensor then set to domeCenterSet true
// */
//void setDomeCenter() {
//  if (domeServoMode) {
//    #ifdef UseHallMonitor
//      if(digitalRead(hallEffectSensor_Pin) == 0){
//        domeCenterSet = true; 
//        myEnc.write(740); //740
//      } else domeCenterSet = false; 
//    #else
//    encoder_direction();
//      if (newPosition != 740) {
//        domeCenterSet = false; 
//      } else domeCenterSet = true; 
//    #endif
//    if ((newPosition >= oldPosition) && !domeCenterSet && (domeServoPWM != 0)) {
//      digitalWrite(domeMotor_pin_B,LOW);
//      digitalWrite(domeMotor_pin_A,HIGH);
//      analogWrite(domeMotor_pwm,100);
//    } else if ((newPosition > oldPosition) && !domeCenterSet && (domeServoPWM != 0)) {
//        digitalWrite(domeMotor_pin_B,HIGH);
//        digitalWrite(domeMotor_pin_A,LOW);
//        analogWrite(domeMotor_pwm,100);
//    }
//  }
//}

void spinStuff() {
  if(domeServoMode) {
    servoDome();
  } else {
    spinDome();
  }
}

void servoDome () {
   if (domeServoMode) {
    int joystickInput = receiveFromESP32Data.domeSpin; // Use domeSpin as the joystick input
    int hallEffectSensorState = digitalRead(hallEffectSensor_Pin);
    int motorSpeed = 0; // Variable for motor speed
    
    if (hallEffectSensorState == HIGH) {
      // The spindle is in the forward position
      targetPosition = myEnc.read();
    } else {
      // Read joystick input and calculate the target position
      int deltaDegrees = map(joystickInput, -127, 127, -maxDegrees, maxDegrees);
      targetPosition = constrain(targetPosition + (deltaDegrees * degreesPerCount), -maxDegrees * degreesPerCount, maxDegrees * degreesPerCount);
    }
    
    // Calculate motor speed and direction
    int currentPosition = myEnc.read();
    int positionDifference = targetPosition - currentPosition;
    motorSpeed = map(abs(positionDifference), 0, maxDegrees * degreesPerCount, 0, 255); // Scale speed based on position difference
    motorSpeed = constrain(motorSpeed, 0, 255); // Constrain speed to valid PWM range
    
    // Set motor direction and speed
    if (positionDifference > 0) {
      digitalWrite(domeMotor_pin_A, HIGH);
      digitalWrite(domeMotor_pin_B, LOW);
      analogWrite(domeMotor_pwm, motorSpeed);
    } else if (positionDifference < 0) {
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, HIGH);
      analogWrite(domeMotor_pwm, motorSpeed);
    } else {
      // Stop the motor if the target position is reached
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, LOW);
      analogWrite(domeMotor_pwm, 0);
    }

    // If no input is given, return to the forward position
    if (joystickInput == 0) {
      targetPosition = 0;
    }

    // Debugging output
    Serial.print("Joystick Input: ");
    Serial.print(joystickInput);
    Serial.print(" Current Position: ");
    Serial.print(currentPosition);
    Serial.print(" Target Position: ");
    Serial.println(targetPosition);
  }

}

void spinDome() {
  if (domeServoPWM > 3 && enableDrive) {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    analogWrite(domeMotor_pwm,map(domeServoPWM,3,127,0,255));

  } else if (domeServoPWM < -3 && enableDrive) {
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,map(domeServoPWM,-127,-3,0,255));

  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
  
}
      

void domeServoMovement() {
  
  if (enableDrive) {
      Setpoint_domeSpinServoPid = domeServoPWM;
      // Setpoint_domeSpinServoPid = map(domeServoPWM,-127,127,-75,75);
    } else {
      Setpoint_domeSpinServoPid = 0; 
    }
  
    Input_domeSpinServoPid = map(encPos,0,1497,180,-180); 

    myPID_domeSpinServoPid.Compute(); 
      
    if (Output_domeSpinServoPid >2 && enableDrive) {
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, HIGH); // Motor Forward
      analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
    } else if (Output_domeSpinServoPid <-2 && enableDrive) {
      digitalWrite(domeMotor_pin_A, HIGH);
      digitalWrite(domeMotor_pin_B, LOW); // Motor Backward
      analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
    } else {
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, LOW); // Dome Motor Stopped
    }
}
