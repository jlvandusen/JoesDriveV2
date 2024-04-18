void encoder() {
  encPos = myEnc.read(); 

  if (domeServoMode && !moveR3Was) {
    moveR3Was = true;
    myEnc.write(encPos - 1497/2); //myEnc.write(encPos - 1497/2); 1680
    
  } else if (!domeServoMode && moveR3Was) {
    moveR3Was = false;
    myEnc.write(encPos + 1497/2); // 1680
  }
  if (encPos > 1497) {
    encPos -= 1497; 
    myEnc.write(encPos);
  } else if (encPos < 0) {
    encPos += 1497; 
    myEnc.write(encPos);
  }

  if (reverseDrive && r3Flag == false) {
    r3Flag = true; 
    domeServoMode = !domeServoMode; 
  } else if (!reverseDrive && r3Flag == true) {
    r3Flag = false; 
  }
  
}
  
/*
 *   Turn the dome spin clockwise till it finds the hallsensor then set to domeCenterSet true
 */
void setDomeCenter() {
  if (domeServoMode) {
    #ifdef UseHallMonitor
      if(digitalRead(hallEffectSensor_Pin) == 0){
        domeCenterSet = true; 
        myEnc.write(740); //740
      } else domeCenterSet = false; 
    #else
    encoder_direction();
      if (newPosition != 740) {
        domeCenterSet = false; 
      } else domeCenterSet = true; 
    #endif
    if ((newPosition >= oldPosition) && !domeCenterSet && (domeServoPWM != 0)) {
      digitalWrite(domeMotor_pin_B,LOW);
      digitalWrite(domeMotor_pin_A,HIGH);
      analogWrite(domeMotor_pwm,100);
    } else if ((newPosition > oldPosition) && !domeCenterSet && (domeServoPWM != 0)) {
        digitalWrite(domeMotor_pin_B,HIGH);
        digitalWrite(domeMotor_pin_A,LOW);
        analogWrite(domeMotor_pwm,100);
    }
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
