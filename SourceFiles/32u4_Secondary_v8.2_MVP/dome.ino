void encoder() {
  encPos = myEnc.read(); 
  if (reverseDrive && !moveR3Was) {
    moveR3Was = true;
    myEnc.write(encPos - 1497/2); //myEnc.write(encPos - 1497/2); 1680
    
  } else if (!reverseDrive && moveR3Was) {
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

  // if (domeServoMode && r3Flag == false) {
  //   r3Flag = true; 
  //   domeServoMode = !domeServoMode; 
  // } else if (!domeServoMode && r3Flag == true) {
  //   r3Flag = false; 
  // }
  
}
  

void encoder_direction() {
    newPosition = myEnc.read();
    // if (newPosition != oldPosition ) {
    //   oldPosition = newPosition;
    // }
}


/*
 *   Turn the dome in the opposite direction based on which way it was turned till it finds the hallsensor then stop
 */

void DomeRecenter() {
  #ifdef UseHallMonitor
    if(digitalRead(hallEffectSensor_Pin) == 0) {
      domeCenter = true;
      myEnc.write(749); //740
      newPosition = 749;
      oldPosition = 749;
      digitalWrite(domeMotor_pin_B,LOW);
      digitalWrite(domeMotor_pin_A,LOW);
    } else {
      domeCenter = false;
      encoder_direction();
    }
    if (!domeCenter) {
      if (newPosition < oldPosition){
        digitalWrite(domeMotor_pin_B,LOW);
        digitalWrite(domeMotor_pin_A,HIGH);
        analogWrite(domeMotor_pwm,100);
      }
      if (newPosition > oldPosition){
        digitalWrite(domeMotor_pin_B,HIGH);
        digitalWrite(domeMotor_pin_A,LOW);
        analogWrite(domeMotor_pwm,100);
      }
    }
    #endif  
}

/*
 *   Turn the dome spin clockwise till it finds the hallsensor then set to domeCenterSet true
 */

void setDomeCenter() {
  #ifdef UseHallMonitor
    if(digitalRead(hallEffectSensor_Pin) == 0) {
      domeCenterSet = 1; 
      myEnc.write(749); //740
      newPosition = 0;
      oldPosition = newPosition;
      digitalWrite(domeMotor_pin_B,LOW);
      digitalWrite(domeMotor_pin_A,LOW);
    } else {
      digitalWrite(domeMotor_pin_B,LOW);
      digitalWrite(domeMotor_pin_A,HIGH);
      analogWrite(domeMotor_pwm,100);
    }
    #endif  
}

void spinStuff() {
  if(domeServoMode) {
    domeServoMovement();
  } else {
    spinDome();
  }
}

void spinDome() {
  if (enableDrive) {
    int domeSpin = map(receiveFromESP32Data.domeSpin, -128,128,255,-255);
    constrain(domeSpin, -255,255);
    if (domeSpin < 5) {
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, HIGH); // Motor Forward
      analogWrite(domeMotor_pwm,abs(domeSpin));
    } else if (domeSpin > -5) {
      digitalWrite(domeMotor_pin_A, HIGH);
      digitalWrite(domeMotor_pin_B, LOW); // Motor Backward
      analogWrite(domeMotor_pwm,abs(domeSpin));
    } else {
      // digitalWrite(domeMotor_pin_A, LOW);
      // digitalWrite(domeMotor_pin_B, LOW); // Dome Motor Stopped
    }
  }
}

void domeServoMovement() {
  if(enableDrive) {
    DomeRecenter();
    int domeSpinPWM = map(receiveFromESP32Data.domeSpin, -128,128,255,-255);
    constrain(domeSpinPWM, -255,255);
    // encoder_direction();
    if (domeSpinPWM < 5) {
      digitalWrite(domeMotor_pin_A, LOW);
      digitalWrite(domeMotor_pin_B, HIGH); // Motor Forward
      analogWrite(domeMotor_pwm,abs(domeSpinPWM));
    } else if (domeSpinPWM > -5) {
      digitalWrite(domeMotor_pin_A, HIGH);
      digitalWrite(domeMotor_pin_B, LOW); // Motor Backward
      analogWrite(domeMotor_pwm,abs(domeSpinPWM));
    } 
  }

}
