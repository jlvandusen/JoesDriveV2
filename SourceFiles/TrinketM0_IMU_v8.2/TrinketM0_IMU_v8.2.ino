/*
 * Joe's Drive  - V8.2 5/2024
 * Trinket IMU Dedicated board
 * Written by James VanDusen - https://www.facebook.com/groups/799682090827096
 * You will need libraries: 
 * EasyTransfer Library - https://github.com/madsci1016/Arduino-EasyTransfer
 * Adafruit MPU6050 - https://github.com/adafruit/Adafruit_MPU6050
 * Adafruit Common Sensor - https://github.com/adafruit/Adafruit_Sensor
*/

#define debugMPU

#include <Arduino.h>
#include <EasyTransfer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

//create object
EasyTransfer sendIMU; 

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float pitch;
  float roll;
};

//give a name to the group of data
//SEND_DATA_STRUCTURE mydata;
SEND_DATA_STRUCTURE sendIMUData;

void setup(){
  delay(10000); // Pause for the operation system to boot
  Serial.begin(115200);
  Serial1.begin(115200);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  Serial.println("Adafruit MPU6050 VALIDATION..."); // Try to initialize the MPU
  sendIMU.begin(details(sendIMUData), &Serial1);  
  if (!mpu.begin()) {   
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  setupmpu(); // Calls and displays various readings for the frequencies
  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();M

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();
  Serial.println("MPU6050 Found OVER I2C (QWIIC)!");
}

void loop(){
  //this is how you access the variables. [name of the group].[variable name]
//  Serial.println("This should be working...");
  readmpu();
  sendIMU.sendData();  //send the data
}

void setupmpu() {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }
  Serial.println("");
  delay(100);
}

void readmpu() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  sendIMUData.pitch = accel.acceleration.x;
  sendIMUData.roll = accel.acceleration.y;
  #ifdef debugMPU
  Serial.print("roll: ");
  Serial.print(sendIMUData.roll);
  Serial.print(" pitch: ");
  Serial.println(sendIMUData.pitch);
  #endif
//  Z = accel.acceleration.z;
//  Roll = atan2(Y, Z) * 180/PI;
//  Pitch = atan2(X, sqrt(Y*Y + Z*Z)) * 180/PI;
//  Roll = a.acceleration.x;
//  Pitch = a.acceleration.y;
}
