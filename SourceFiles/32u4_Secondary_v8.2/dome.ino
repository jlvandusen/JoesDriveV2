
void spinStuff() {
  if(domeServoMode) {
    spinDomeServo();
  } else {
    spinDome();
  }
}

void spinDomeServo () {
   if (domeServoMode) {
    int joystickInput = constrain(receiveFromESP32Data.domeSpin, -127, 127); // Ensure valid input range
    #ifdef EnableFilters
    joystickInput = adcFilterdomeSpin.filter(joystickInput);
    #endif
    if(reverseDrive){
      joystickInput *= -1;
    }
    int hallEffectSensorState = digitalRead(hallEffectSensor_Pin);
    int motorSpeed = 0; // Variable for motor speed

    // if (hallEffectSensorState == 1 || hallEffectSensorState == 0) {
    if (hallEffectSensorState == 0) {
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
    #ifdef debugDomeMotor
      Serial.print("Joystick Input: ");
      Serial.print(joystickInput);
      Serial.print(" Current Position: ");
      Serial.print(currentPosition);
      Serial.print(" Target Position: ");
      Serial.println(targetPosition);
    #endif
  }

}

void spinDome() {
  int joystickInput = constrain(receiveFromESP32Data.domeSpin, -127, 127); // Ensure valid input range
  #ifdef EnableFilters
    joystickInput = adcFilterdomeSpin.filter(joystickInput);
  #endif
  if(reverseDrive){
    joystickInput *= -1;
  }
  int motorSpeed = 0; // Variable for motor speed
  // motorSpeed = map(abs(joystickInput), -127, 127, 0, 255);
  if (joystickInput > domeMotorDeadzone && enableDrive) {
    motorSpeed = map(joystickInput, 0, 127, 0, 255);
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    analogWrite(domeMotor_pwm,motorSpeed);

  } else if (joystickInput < -domeMotorDeadzone && enableDrive) {
    motorSpeed = map(joystickInput, -127, 0, 255, 0);
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,motorSpeed);

  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
  
}
      

// void domeServoMovement() {
  
//   if (enableDrive) {
//       Setpoint_domeSpinServoPid = domeServoPWM;
//       // Setpoint_domeSpinServoPid = map(domeServoPWM,-127,127,-75,75);
//     } else {
//       Setpoint_domeSpinServoPid = 0; 
//     }
  
//     Input_domeSpinServoPid = map(encPos,0,1497,180,-180); 

//     myPID_domeSpinServoPid.Compute(); 
      
//     if (Output_domeSpinServoPid >2 && enableDrive) {
//       digitalWrite(domeMotor_pin_A, LOW);
//       digitalWrite(domeMotor_pin_B, HIGH); // Motor Forward
//       analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
//     } else if (Output_domeSpinServoPid <-2 && enableDrive) {
//       digitalWrite(domeMotor_pin_A, HIGH);
//       digitalWrite(domeMotor_pin_B, LOW); // Motor Backward
//       analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
//     } else {
//       digitalWrite(domeMotor_pin_A, LOW);
//       digitalWrite(domeMotor_pin_B, LOW); // Dome Motor Stopped
//     }
// }
