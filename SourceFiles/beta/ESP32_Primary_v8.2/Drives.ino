void S2S_Movement(){
//  Using the DFRobot Motor Driver 0601 we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop
//  Input1 = Potentiometer, Input2 = IMU
  if (reverseDrive) {
    target_pos_S2S = map(buttonsR.rightStickX, -127,127,maxS2STilt,-maxS2STilt);  //Read in the Move Controller X Axis of Right Stick and force it from 127 extremes to 40
  } else {
      target_pos_S2S = map(buttonsR.rightStickX, -127,127,-maxS2STilt,maxS2STilt);  //Read in the Move Controller X Axis of Right Stick and force it from 127 extremes to 40
  }
  
  
  easing_S2S = 250;  //modify these values for sensitivity (500)
  easing_S2S /= 250;
  
  diff_S2S = target_pos_S2S - current_pos_S2S; // Work out the required travel.
  
  if( diff_S2S != 0.00 ) {
    current_pos_S2S += diff_S2S * easing_S2S; // Avoid any strange zero condition
  }
  
  Setpoint2 = current_pos_S2S;
  
//  Input2 = (receiveIMUData.roll*-1)- IMUDeadzone;   // ****add a bit to the IMU to get the real middle point
  if (reverseDrive) {
    Input2 = ((receiveIMUData.roll * -1) + rollOffset);   // ****Add Offsets to the IMU readings (setting Zero)
  } else {
      Input2 = ((receiveIMUData.roll) + rollOffset);   // ****Add Offsets to the IMU readings (setting Zero)
  }

  
  Setpoint2 = constrain(Setpoint2, -maxS2STilt,maxS2STilt);  // Allow the S2S to only move 45 each direction
  
  PID2_S2S.Compute(); // Apply PID values to Joystick values

  S2S_pot = analogRead(S2SPot_pin);   // read S2S pot
  // Setpoint1 = Output2;
  Setpoint1 = map(constrain(Output2, -maxS2STilt,maxS2STilt), -maxS2STilt,maxS2STilt, maxS2STilt,-maxS2STilt);

  #ifndef revS2S
  // NOTE: Depending upon resolution of POT, it may be 1024 or 4095
  S2S_pot = map(S2S_pot, 0, 4095, 135,-135); // 255
  #else
  S2S_pot = map(S2S_pot, 0, 4095, -135,135); // 255
  #endif

  Input1  = S2S_pot + potOffsetS2S;  // Take in the value from Potentiometer with offests (setting Zero)
  // Input1 = constrain(Input1,-45,45);  // force Input1 to be only -45 to 45 (90degrees)
  // Setpoint1 = constrain(Setpoint1, -45,45); // take in joystick values defined by the output from the Potentiometer
  // Setpoint1 = map(Setpoint1,45,-45,-45,45);
  
  PID1_S2S.Compute();
  // if ((abs(Input1) < S2S_potDeadzone ) || (abs(Input1) > S2S_potDeadzone)) {
    // if (Output1 < 0) // decide which way to turn the wheels based on deadSpot variable
  if ((Output1 <= -1) && (Input1 > -maxS2STilt)) // decide which way to turn the wheels based on deadSpot variable
  {
    Output1_S2S_pwm = abs(Output1);
    #ifndef revS2S
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, HIGH); // S2S Left
    #else
    digitalWrite(S2S_pin_1, HIGH);
    digitalWrite(S2S_pin_2, LOW); // S2S Right
    #endif
  }
  // else if (Output1 >= 0) // decide which way to turn the wheels based on deadSpot variable
  else if ((Output1 >= 1) && (Input1 < maxS2STilt)) // decide which way to turn the wheels based on deadSpot variable
  { 
    Output1_S2S_pwm = abs(Output1);
    #ifndef revS2S
    digitalWrite(S2S_pin_1, HIGH);
    digitalWrite(S2S_pin_2, LOW); // S2S Right
    #else
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, HIGH); // S2S Left
    #endif
  } 
  else
  {
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, LOW); // S2S Stopped (brake)
  }
  if (controllerConnected && enableDrive && !buttonsL.l1 && !buttonsR.l1) // Check to ensure that L1 is not pressed on either Dome or Drive Controllers
  { 
    analogWrite(S2S_pwm, Output1_S2S_pwm);
  }
  else {
    Output1_S2S_pwm = 0;
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 stopped
  }
}

void drive_Movement() {
//  Using the DFRobot Motor Driver 0601 we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop
//    digitalWrite(Drive_pin_1, HIGH or LOW);  
//    digitalWrite(Drive_pin_2, HIGH or LOW);
//    analogWrite(Drive_pwm, (0-255));
  Input3 = receiveIMUData.pitch - pitchOffset; 
  if(reverseDrive == false){
    buttonsR.rightStickY *= -1;
  }
    
  if(Setpoint_Drive_In > buttonsR.rightStickY) {
    Setpoint_Drive_In-=driveDelay; 
  } else if(Setpoint_Drive_In < buttonsR.rightStickY) {
    Setpoint_Drive_In+=driveDelay; 
  }

  Setpoint3 = map(Setpoint_Drive_In,-127,128,-40,40); 
  
  PID_Drive.Compute();

  if(Output3 > 10 && enableDrive) {
	  digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, HIGH);
	  analogWrite(Drive_pwm, map(abs(Output3),0,255,0,255));
    // analogWrite(Drive_pwm, Output3);
  } else if(Output3 < -10 && enableDrive) {
	  digitalWrite(Drive_pin_1, HIGH);
    digitalWrite(Drive_pin_2, LOW);
	  // analogWrite(Drive_pwm, Output3);
    analogWrite(Drive_pwm, map(abs(Output3),0,255,0,255));
  } else {
	  digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, LOW);
  }

}


void spinFlywheel() {
  if (EnableFlywheel && enableDrive) {
    if (reverseDrive) {
      flywheel = map(buttonsR.rightStickX, -128,128,255,-255);
      constrain(flywheel, 255, -255);
    } else {
        flywheel = map(buttonsR.rightStickX, -128,128,-255,255);
        constrain(flywheel, -255, 255);
    }
    if (flywheel < -10) {
          digitalWrite(flyWheelMotor_pin_A, LOW);
          digitalWrite(flyWheelMotor_pin_B, HIGH); // Motor 1 Forward
          analogWrite(flyWheelMotor_pwm,abs(flywheel));
        } else if (flywheel > 10) {
          digitalWrite(flyWheelMotor_pin_A, HIGH);
          digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Forward
          analogWrite(flyWheelMotor_pwm,abs(flywheel));
        } else {
          digitalWrite(flyWheelMotor_pin_A, LOW);
          digitalWrite(flyWheelMotor_pin_A, LOW); // Motor 1 Forward
        }
    analogWrite(flyWheelMotor_pwm,abs(flywheel));
  }
}

void autoDisableMotors(){
 // buttonsR.rightStickY = Main Drive Forward and Backward
 // buttonsR.rightStickX = S2S Steering using tilt
 if((buttonsR.rightStickY > -joystickDeadZoneRange && buttonsR.rightStickY < joystickDeadZoneRange) && (buttonsR.rightStickX > -joystickDeadZoneRange && buttonsR.rightStickX < joystickDeadZoneRange) && (buttonsL.leftStickX > -joystickDeadZoneRange && buttonsL.leftStickX < joystickDeadZoneRange) && (buttonsL.leftStickY < joystickDeadZoneRange && buttonsL.leftStickY > -joystickDeadZoneRange) && (autoDisableState == 0)) {
   autoDisableMotorsMillis = millis();
   autoDisableState = 1;
 } else if (buttonsR.rightStickY < -joystickDeadZoneRange || buttonsR.rightStickY > joystickDeadZoneRange || buttonsR.rightStickY < -joystickDeadZoneRange || buttonsR.rightStickY > joystickDeadZoneRange || buttonsL.leftStickX < -joystickDeadZoneRange || buttonsL.leftStickX > joystickDeadZoneRange || buttonsL.leftStickY > joystickDeadZoneRange || buttonsL.leftStickY < -joystickDeadZoneRange) {
   autoDisableState = 0;     
   autoDisableDoubleCheck = 0; 
   autoDisable = 0;  
 }
         
 if(autoDisableState == 1 && (millis() - autoDisableMotorsMillis >= 3000) && Input3 < 25 && Input1 < 8){
   digitalWrite(Drive_pin_1, LOW);
   digitalWrite(Drive_pin_2, LOW);
   digitalWrite(S2S_pin_1, LOW);
   digitalWrite(S2S_pin_2, LOW);
   digitalWrite(flyWheelMotor_pin_A, LOW);
   digitalWrite(flyWheelMotor_pin_B, LOW);
   autoDisable = 1;
     
 }else if(Input3 > 50 || Input1 > 20){
   autoDisableState = 0;
   autoDisableDoubleCheck = 0;  
   autoDisable = 0;    
 }else if((Input3 > 25 || Input1 > 8) && autoDisableDoubleCheck == 0){
   autoDisableDoubleCheckMillis = millis();
   autoDisableDoubleCheck = 1;
    
 } else if((autoDisableDoubleCheck == 1) && (millis() - autoDisableDoubleCheckMillis >= 100)){
   if(Input3 > 30 || Input1 > 8){ 
     autoDisableState = 0;
     autoDisableDoubleCheck = 0;
     autoDisable = 0;
   }else{
     autoDisableDoubleCheck = 0;
   }
 } 
     
}
