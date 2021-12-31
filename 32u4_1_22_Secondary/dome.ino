void encoder() {
  encPos = myEnc.read(); 
  if (receiveFromESP32Data.moveR3 && moveR3Was == false) {
    moveR3Was = true;
    myEnc.write(encPos - 1497/2);
    
  } else if (receiveFromESP32Data.moveR3 == false && moveR3Was == true) {
    moveR3Was = false;
    myEnc.write(encPos + 1497/2);
  }
  if (encPos > 1497) {
    encPos -= 1497; 
    myEnc.write(encPos);
  } else if (encPos < 0) {
    encPos += 1497; 
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
 *   Turn the dome spin clockwise till it finds the hallsensor then set to domeCenterSet 1
 */
void setDomeCenter() {
  int domecenter = digitalRead(hallEffectSensor_Pin);
  if (domeCenterSet == false) {
    digitalWrite(domeMotor_pin_B,HIGH);
    digitalWrite(domeMotor_pin_A,LOW);
    
    if (domecenter == 0) {
      domeCenterSet = true; 
      myEnc.write(749);
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, LOW); // Motor stop we are forward
    }
  analogWrite(domeMotor_pwm,255);  // Turn the head slowly till it finds center (hall = 0)
  }

}

void spinDome() {
  domeServoPWM = map(receiveFromESP32Data.domeSpin,-127,127,-255,255);
  if (domeServoPWM > 3 && enableDrive && !domeServoMode) {
//    domeCenterSet = false;
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    
  } else if (domeServoPWM < -3 && enableDrive && !domeServoMode) {
//    domeCenterSet = false;
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward

  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
  analogWrite(domeMotor_pwm,abs(domeServoPWM));
}
      

void domeServoMovement() {
  if (domeServoMode && enableDrive) {
      Setpoint_domeSpinServoPid = map(receiveFromESP32Data.domeSpin, -100, 100, 75, -75); 
    } else {
      Setpoint_domeSpinServoPid = 0; 
    }
  
  Input_domeSpinServoPid = map(encPos,0,1497,180, -180); 

  myPID_domeSpinServoPid.Compute(); 
    
  if (Output_domeSpinServoPid >2 && receiveFromESP32Data.driveEnabled) {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, HIGH); // Motor 1 Forward
    analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
  } else if (Output_domeSpinServoPid <-2 && receiveFromESP32Data.driveEnabled) {
    digitalWrite(domeMotor_pin_A, HIGH);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Backward
    analogWrite(domeMotor_pwm,abs(Output_domeSpinServoPid));
  } else {
    digitalWrite(domeMotor_pin_A, LOW);
    digitalWrite(domeMotor_pin_B, LOW); // Motor 1 Stopped
  }
}
