
void spinStuff() {
  if(domeServoMode) {
    spinDomeServo();
  } else {
    spinDome();
  }
}
void spinDomeServo() {
  long encoderCounts = myEnc.read(); // Read the current counts from the encoder
  float rotationDegrees = (encoderCounts / encoderCountsPerRevolution) * 360.0; // Calculate the rotation in degrees
  int hallSensorValue = digitalRead(hallEffectSensor_Pin); // Read the Hall sensor value
  int joystickInput = constrain(receiveFromESP32Data.domeSpin, -127, 127); // Ensure valid input range
  int motorSpeed = 0;
  #ifdef EnableFilters
    joystickInput = adcFilterdomeSpin.filter(joystickInput);
  #endif
  if(reverseDrive){
    joystickInput *= -1;
  }
  if (hallSensorValue == 0) {
    myEnc.write(0);
  }
  motorSpeed = map(joystickInput, 0, 127, 0, 255);
  if (joystickInput > domeMotorDeadzone && enableDrive) {
    if (rotationDegrees <= maxDegrees && rotationDegrees >= 0){
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
      analogWrite(domeMotor_pwm, motorSpeed);
    }
  } else if (joystickInput < -domeMotorDeadzone && enableDrive) {
    if (rotationDegrees >= -maxDegrees && rotationDegrees < 0){
      digitalWrite(domeMotor_pin_A, HIGH);
      digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
      analogWrite(domeMotor_pwm, motorSpeed);
    }
  }
  if (joystickInput == 0 && enableDrive) {
    // If there's no joystick input, return the motor to center
    if (rotationDegrees > 0) {
      // If the rotation is positive, move the motor in the negative direction
      digitalWrite(domeMotor_pin_A, HIGH);
      digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
      analogWrite(domeMotor_pwm, 150);
    } else if (rotationDegrees < 0) {
      // If the rotation is negative, move the motor in the positive direction
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
      analogWrite(domeMotor_pwm, 150);
    } else {
      // If the rotation is at center, stop the motor
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
    }
  }

}


void spinDome() {
  long encoderCounts = myEnc.read(); // Read the current counts from the encoder
  float rotationDegrees = (encoderCounts / 1680.0) * 360.0; // Calculate the rotation in degrees
  int joystickInput = constrain(receiveFromESP32Data.domeSpin, -127, 127); // Ensure valid input range
  int hallSensorValue = digitalRead(hallEffectSensor_Pin); // Read the Hall sensor value
  #ifdef EnableFilters
    joystickInput = adcFilterdomeSpin.filter(joystickInput);
  #endif
  if(reverseDrive){
    joystickInput *= -1;
  }
  int motorSpeed = 0; // Variable for motor speed
  if (hallSensorValue == 0) {
    myEnc.write(0);
  }
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
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 and 2 Stopped
  }
  
}
void setDomeCenter() {
  long encoderCounts = myEnc.read(); // Read the current counts from the encoder
  int hallSensorValue = digitalRead(hallEffectSensor_Pin); // Read the Hall sensor value
  while (hallSensorValue != 0) {
    hallSensorValue = digitalRead(hallEffectSensor_Pin); // Read the Hall sensor value
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,75);
  }
  digitalWrite(domeMotor_pin_A, LOW); // Motor 2 STOP
  digitalWrite(domeMotor_pin_B, LOW); // Motor 1 STOP
  myEnc.write(0);
}
