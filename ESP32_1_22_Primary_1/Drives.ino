void S2S_Movement(){
//  Using the DFRobot Motor Driver 0601 we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop

  target_pos_S2S = map(buttonsR.rightStickX, -127,127,-40,40);  //Read in the Move Controller X Axis of Right Stick and force it from 127 extremes to 40
  
  easing_S2S = 500;  //modify these values for sensitivity
  easing_S2S /= 500;
  
  diff_S2S = target_pos_S2S - current_pos_S2S; // Work out the required travel.
  
  if( diff_S2S != 0.00 ) {
    current_pos_S2S += diff_S2S * easing_S2S; // Avoid any strange zero condition
  }
  
  Setpoint2 = current_pos_S2S;
  
  S2S_pot = analogRead(S2SPot_pin);   // read S2S pot
  
  Input2 = (receiveIMUData.roll*-1)- bodge;   // ****add a bit to the IMU to get the real middle point
  
  Setpoint2 = constrain(Setpoint2, -45,45);  // Allow the S2S to only move 45 each direction
  
  PID2_S2S.Compute();
  
  Setpoint1 = Output2;
  #ifndef revS2S
  S2S_pot = map(S2S_pot, 0, 4095, 255,-255);
  #else
  S2S_pot = map(S2S_pot, 0, 4095, -255,255);
  #endif
  
  S2S_pot = S2S_pot-2;
  
  Input1  = S2S_pot;
  Input1 = constrain(Input1,-45,45);
  Setpoint1 = constrain(Setpoint1, -45,45);
  Setpoint1 = map(Setpoint1,45,-45,-45,45);
  
  PID1_S2S.Compute();
  
  if (Output1 < 0) // decide which way to turn the wheels based on deadSpot variable
  {
    Output1_S2S_pwm = abs(Output1);
    #ifndef revS2S
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, HIGH); // Motor 1 Forward
    #else
    digitalWrite(S2S_pin_1, HIGH);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 Forward
    #endif
  }
  else if (Output1 >= 0) // decide which way to turn the wheels based on deadSpot variable
  { 
    Output1_S2S_pwm = abs(Output1);
    #ifndef revS2S
    digitalWrite(S2S_pin_1, HIGH);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 Backwards
    #else
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, HIGH); // Motor 1 Forward
    #endif
  } 
  else
  {
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 stopped
  }
  if (controllerConnected && enableDrive && !buttonsL.l1 && !buttonsR.l1) { // Check to ensure that L1 is not pressed on either Dome or Drive Controllers
    analogWrite(S2S_pwm, Output1_S2S_pwm);
  }
  else {
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 stopped
  }
 
}


void drive_Movement(){
//  Using the DFRobot Motor Driver 0601 we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop
//    digitalWrite(Drive_pin_1, HIGH);  
//    digitalWrite(Drive_pin_2, LOW);
//    analogWrite(Drive_pwm, 255);
  target_pos_drive = map(buttonsR.rightStickY, -127,127,-65,65);
  
  easing_drive = 1000;          //modify this value for sensitivity
  easing_drive /= 1000;
  

  diff_drive = target_pos_drive - current_pos_drive;  // Work out the required travel.
  
  if( diff_drive != 0.00 ) {
    current_pos_drive += diff_drive * easing_drive;   // Avoid any strange zero condition
  }
  
  Setpoint3 = current_pos_drive;
  
  Input3 = receiveIMUData.pitch+3;
  
  PID_Drive.Compute();
  
  if (Output3 <= 1) {                              // decide which way to turn the wheels based on deadSpot variable
    Output_Drive_pwm = abs(Output3);
    #ifndef revDrive
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, HIGH);
    #else
    digitalWrite(Drive_pin_1, HIGH);
    digitalWrite(Drive_pin_2, LOW);
    #endif
  }
  else if (Output3 > 1) {                         // decide which way to turn the wheels based on deadSpot variable
    Output_Drive_pwm = abs(Output3);
    #ifndef revDrive
    digitalWrite(Drive_pin_1, HIGH);  
    digitalWrite(Drive_pin_2, LOW);
    #else
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, HIGH);
    #endif
  }
  else {  // Motor 1 stopped
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, LOW);
  }
  if (controllerConnected && enableDrive && !buttonsL.l1 && !buttonsR.l1) { // Check to ensure that L1 is not pressed on either Dome or Drive Controllers
    analogWrite(Drive_pwm, Output_Drive_pwm);
  }
  else {
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, LOW);
  }

}

void spinFlywheel() {
  if (sendTo32u4Data.flywheel >= 10) {
    #ifndef revGyro
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, HIGH); // Motor 1 Forward
    #else
    digitalWrite(flyWheelMotor_pin_A, HIGH);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Forward
    #endif
    
  } else if (sendTo32u4Data.flywheel < -10) {
    #ifndef revGyro
    digitalWrite(flyWheelMotor_pin_A, HIGH);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Backward
    #else
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, HIGH); // Motor 1 Forward
    #endif

  } else {
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Stopped
  }
  if (controllerConnected && enableDrive && buttonsL.l1){
    analogWrite(flyWheelMotor_pwm,abs(Output_flywheel_pwm));
  }
  else {
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Stopped
  }
  
}
