void encoder() {
  encPos = myEnc.read(); 

  if (domeServoMode == 0 && moveR3Was == false) {
    moveR3Was = true;
    myEnc.write(encPos - 1680/2); //myEnc.write(encPos - 1497/2); 1497
    
  } else if (domeServoMode == 1 && moveR3Was == true) {
    moveR3Was = false;
    myEnc.write(encPos + 1680/2);
  }
  if (encPos > 1680) {
    encPos -= 1680; 
    myEnc.write(encPos);
  } else if (encPos < 0) {
    encPos += 1680; 
    myEnc.write(encPos);
  }

  if (receiveFromESP32Data.moveL3 == 1 && r3Flag == false) {
    r3Flag = true; 
    domeServoMode = !domeServoMode; 
  } else if (receiveFromESP32Data.moveL3 == 0 && r3Flag == true) {
    r3Flag = false; 
  }
  
}
  
/*
 *   Turn the dome spin clockwise till it finds the hallsensor then set to domeCenterSet true
 */
void setDomeCenter() {
  digitalWrite(domeMotor_pin_B,HIGH);
  digitalWrite(domeMotor_pin_A,LOW);
  analogWrite(domeMotor_pwm,50);
  if(digitalRead(hallEffectSensor_Pin) == 0){
    domeCenterSet = 1; 
    myEnc.write(840); //740
  }
}

void spinStuff() {
  if(domeServoMode == 1) {
    domeServoMovement();
  } else {
    spinDome();
  }
}

void spinDome() {
  
// #ifndef debugENC
  if (domeServoPWM > 3 && enableDrive) {
// #else
//   if (domeServoPWM > 3) {
// #endif
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    analogWrite(domeMotor_pwm,map(domeServoPWM,3,100,0,255));
// #ifndef debugENC
  } else if (domeServoPWM < -3 && enableDrive) {
// #else
//   } else if (domeServoPWM < -3) {
// #endif
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,map(domeServoPWM,100,3,255,0));

  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
  
}
      

void domeServoMovement() {
  
  if (enableDrive) {
      Setpoint_domeSpinServoPid = map(receiveFromESP32Data.domeSpin,-127,127,-75,75);
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
