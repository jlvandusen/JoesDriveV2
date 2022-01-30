void S2S_Movement(){
//  Using the DFRobot Motor Driver 0601 we need 3 pins
//  VCC and GND to power the driver logic
//  First pin is PWM for speed control 0 - 255
//  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop
//  Input1 = Potentiometer, Input2 = IMU

  target_pos_S2S = map(buttonsR.rightStickX, -127,127,-40,40);  //Read in the Move Controller X Axis of Right Stick and force it from 127 extremes to 40
  
  easing_S2S = 500;  //modify these values for sensitivity
  easing_S2S /= 500;
  
  diff_S2S = target_pos_S2S - current_pos_S2S; // Work out the required travel.
  
  if( diff_S2S != 0.00 ) {
    current_pos_S2S += diff_S2S * easing_S2S; // Avoid any strange zero condition
  }
  
  Setpoint2 = current_pos_S2S;
  
//  Input2 = (receiveIMUData.roll*-1)- IMUDeadzone;   // ****add a bit to the IMU to get the real middle point
  Input2 = (receiveIMUData.roll + rollOffset);   // ****Add Offsets to the IMU readings (setting Zero)
  
  Setpoint2 = constrain(Setpoint2, -45,45);  // Allow the S2S to only move 45 each direction
  
  PID2_S2S.Compute(); // Apply PID values to Joystick values

  S2S_pot = analogRead(S2SPot_pin);   // read S2S pot
  Setpoint1 = Output2;
  #ifndef revS2S
  S2S_pot = map(S2S_pot, 0, 4095, 255,-255);
  #else
  S2S_pot = map(S2S_pot, 0, 4095, -255,255);
  #endif
  
//  S2S_pot = S2S_pot-2;
  
//  Input1  = S2S_pot;  // Take in the value from Potentiometer with map angle of -45 to 45
  Input1  = S2S_pot + potOffsetS2S;  // Take in the value from Potentiometer with offests (setting Zero)
  Input1 = constrain(Input1,-45,45);
  Setpoint1 = constrain(Setpoint1, -45,45); // take in joystick values defined by the output from the Potentiometer
  Setpoint1 = map(Setpoint1,45,-45,-45,45);
  
  PID1_S2S.Compute();
  if ((abs(Input1) < S2S_potDeadzone ) || (abs(Input1) > S2S_potDeadzone)) {
    if (Output1 < 0) // decide which way to turn the wheels based on deadSpot variable
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
    else if (Output1 >= 0) // decide which way to turn the wheels based on deadSpot variable
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
  }
  if (controllerConnected && enableDrive && !buttonsL.l1 && !buttonsR.l1) { // Check to ensure that L1 is not pressed on either Dome or Drive Controllers
    analogWrite(S2S_pwm, Output1_S2S_pwm);
  }
  else {
    Output1_S2S_pwm = 0;
    digitalWrite(S2S_pin_1, LOW);
    digitalWrite(S2S_pin_2, LOW); // Motor 1 stopped
  }
 
}

//void S2S_Movement(){
////  Using the DFRobot Motor Driver we need 3 pins
////  VCC and GND to power the driver logic and must match the CPU voltage (3.3v)
////  First pin is PWM for speed control 0 - 255
////  Second and Third Pins are logic, LOW, HIGH = forward, HIGH, LOW = Backwards, LOW, LOW = stop (for ESP32 use 1 and 0)
//
//  if (IMUconnected){
//  Input_S2S_Servo = receiveIMUData.roll *-1;
//  } else {
//    Input_S2S_Servo = Input_S2S_Servo;
//  }
////  if(sendTo32u4Data.moveL3 == false){
////    buttonsR.rightStickX *= -1; 
////  }
//  Setpoint_S2S_Servo = map(constrain(buttonsR.rightStickX , 0 , 512), 0,512,maxS2STilt,-maxS2STilt); //- is  left, + is  right
////  Setpoint_S2S_Servo = buttonsR.rightStickX; 
//  myPID_S2S_Servo.Compute();
//
//  int S2SPot = map(analogRead(S2SPot_pin), 0, 1024, -135,135);
////  int S2SPot = analogRead(S2SPot_pin);
//
////  Input_S2S_Stabilization = (map(analogRead(S2SPot_pin),0,1023,0,270)+S2S_offset)*-1; // Currently the offset is set to -120
//  Input_S2S_Stabilization = S2SPot + S2S_offset; // -120
////  Setpoint_S2S_Stabilization = Output_S2S_Servo;
//  Setpoint_S2S_Stabilization = map(constrain(Output_S2S_Stabilization, -maxS2STilt,maxS2STilt), -maxS2STilt,maxS2STilt, maxS2STilt,-maxS2STilt);
//  myPID_S2S_Stabilization.Compute(); 
//  
//  if(Output_S2S_Stabilization > 5 && controllerConnected && enableDrive) {
//    digitalWrite(S2S_pin_1, 0);
//    digitalWrite(S2S_pin_2, 1); // Motor 1 Forward
//    analogWrite(S2S_pwm, map(abs(Output_S2S_Stabilization),255,-255,200,0));
//  } else if(Output_S2S_Stabilization < -5 && controllerConnected && enableDrive) {
//    digitalWrite(S2S_pin_1, 1);
//    digitalWrite(S2S_pin_2, 0); // Motor 1 Backwards
//    analogWrite(S2S_pwm, map(Output_S2S_Stabilization,-255,255,200,0));
//    
//  } else {
//    digitalWrite(S2S_pin_1, 0);
//    digitalWrite(S2S_pin_2, 0); // Motor 1 stopped
//  }
//
//#ifdef debugS2SPot
//  SerialDebug.print("S2SPOT_IN_PID:"); SerialDebug.print('\t');
//  SerialDebug.println(Input_S2S_Stabilization);
//  SerialDebug.print("S2SPOT_PIN:"); SerialDebug.print('\t');
//  SerialDebug.println(S2SPot_pin);
//  SerialDebug.print("S2SPOT_PWM:"); SerialDebug.print('\t');
//  SerialDebug.println(S2SPot);
//  SerialDebug.print("[2J"); // clear screen command
//  SerialDebug.write(27);    // ESC command
//#endif
//
//}

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
  
//  Input3 = receiveIMUData.pitch+3;
  Input3 = receiveIMUData.pitch + pitchOffset;
  
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
    Output_Drive_pwm = 0;
    digitalWrite(Drive_pin_1, LOW);
    digitalWrite(Drive_pin_2, LOW);
  }

}

void spinFlywheel() {
  if (flywheel >= 10) {
    #ifndef revGyro
    digitalWrite(flyWheelMotor_pin_A, LOW);
    digitalWrite(flyWheelMotor_pin_B, HIGH); // Motor 1 Forward
    #else
    digitalWrite(flyWheelMotor_pin_A, HIGH);
    digitalWrite(flyWheelMotor_pin_B, LOW); // Motor 1 Forward
    #endif
    
  } else if (flywheel < -10) {
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

//void autoDisableMotors(){
//  // buttonsR.rightStickY = Main Drive Forward and Backward
//  // buttonsR.rightStickX = S2S Steering using tilt
//  if((buttonsR.rightStickY > joystickDeadZoneRange && buttonsR.rightStickY < joystickDeadZoneRange) && (buttonsR.rightStickX > joystickDeadZoneRange && buttonsR.rightStickX < joystickDeadZoneRange) && (buttonsL.leftStickX > joystickDeadZoneRange && buttonsL.leftStickX < joystickDeadZoneRange) && (buttonsL.leftStickY < joystickDeadZoneRange && buttonsL.leftStickY > joystickDeadZoneRange) && (autoDisableState == 0)) {
//    autoDisableMotorsMillis = millis();
//    autoDisableState = 1;
//  } else if(buttonsR.rightStickY < joystickDeadZoneRange || buttonsR.rightStickY > joystickDeadZoneRange || buttonsR.rightStickY < joystickDeadZoneRange || buttonsR.rightStickY > joystickDeadZoneRange || buttonsL.leftStickX < joystickDeadZoneRange || buttonsL.leftStickX > joystickDeadZoneRange || buttonsL.leftStickY > joystickDeadZoneRange || buttonsL.leftStickY < joystickDeadZoneRange) {
//    autoDisableState = 0;     
//    autoDisableDoubleCheck = 0; 
//    autoDisable = 0;  
//  }
//          
//  if(autoDisableState == 1 && (millis() - autoDisableMotorsMillis >= 3000) && Output_S2S_Stabilization < 25 && Output_Drive_Stabilization < 8){
//    digitalWrite(Drive_pin_1, 0);
//    digitalWrite(Drive_pin_2, 0);
//    digitalWrite(S2S_pin_1, 0);
//    digitalWrite(S2S_pin_2, 0);
//    digitalWrite(flyWheelMotor_pin_A, 0);
//    digitalWrite(flyWheelMotor_pin_B, 0);
//    autoDisable = 1;
//      
//  }else if(Output_S2S_Stabilization > 50 || Output_Drive_Stabilization > 20){
//    autoDisableState = 0;
//    autoDisableDoubleCheck = 0;  
//    autoDisable = 0;    
//  }else if((Output_S2S_Stabilization > 25 || Output_Drive_Stabilization > 8) && autoDisableDoubleCheck == 0){
//    autoDisableDoubleCheckMillis = millis();
//    autoDisableDoubleCheck = 1;
//     
//  } else if((autoDisableDoubleCheck == 1) && (millis() - autoDisableDoubleCheckMillis >= 100)){
//    if(Output_S2S_Stabilization > 30 || Output_Drive_Stabilization > 8){ 
//      autoDisableState = 0;
//      autoDisableDoubleCheck = 0;
//      autoDisable = 0;
//    }else{
//      autoDisableDoubleCheck = 0;
//    }
//  } 
//      
//}
