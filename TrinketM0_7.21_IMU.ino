/* 
Utilizes the following libraries
https://github.com/adafruit/Adafruit_MPU6050 for i2c and mpu6050 suppport (under arduino)
*/

//#define debugMPU   //Prints pitch and roll from MPU6050
//#define debugSerial //Prints out what is being sent over Serial1

#include <Arduino.h>
#include <EasyTransfer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

const int numReadings = 4;

struct SEND_DATA_STRUCTURE_IMU{
  float pitch;
  float roll;
};

SEND_DATA_STRUCTURE_IMU sendIMUData;

EasyTransfer SendIMU;

//#define OUTPUT_READABLE_YAWPITCHROLL

bool blinkState = false;
bool firstLoop = true; 

//roll = atan2(Y, Z) * 180/PI;
//pitch = atan2(X, sqrt(Y*Y + Z*Z)) * 180/PI;

int loopTime=15;   
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float avgPitch[numReadings];
float avgRoll[numReadings]; 
int thisReadPitch;
int thisReadRoll; 

unsigned long printTime, lastLoop;

void setup(void) {
  Serial.begin(115200);
  Serial1.begin(115200);
  SendIMU.begin(details(sendIMUData), &Serial1); // pins 4 and 3 RX/TX
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 VALIDATION...");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found OVER I2C (QWIIC)!");
  setupmpu(); // Calls and displays various readings for the frequencies
  mpu_temp = mpu.getTemperatureSensor();
  mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();


}

void loop() {
  readmpu();
  if(millis() - lastLoop >= loopTime){
    SendIMU.sendData();
    #ifdef debugMPU
      debugmpu();
    #endif
    #ifdef debugSerial
      debugserialsend();
    #endif
  }

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
//  Z = accel.acceleration.z;
//  Roll = atan2(Y, Z) * 180/PI;
//  Pitch = atan2(X, sqrt(Y*Y + Z*Z)) * 180/PI;
//  Roll = a.acceleration.x;
//  Pitch = a.acceleration.y;
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

void debugserialsend(){
  Serial.print("Serial1:  ");
  Serial.print("Roll: ");
  Serial.print(sendIMUData.roll);
  Serial.print("  Pitch: ");
  Serial.println(sendIMUData.pitch);
}


void debugmpu(){
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sensors_event_t accel;
  sensors_event_t gyro;
//  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

//  /* Display the results (acceleration is measured in m/s^2) */
//  Serial.print("\t\tAccel X: ");
//  Serial.print(accel.acceleration.x);
//  Serial.print(" \tY: ");
//  Serial.print(accel.acceleration.y);
//  Serial.print(" \tZ: ");
//  Serial.print(accel.acceleration.z);
//  Serial.println(" m/s^2 ");
//
//  /* Display the results (rotation is measured in rad/s) */
//  Serial.print("\t\tGyro X: ");
//  Serial.print(gyro.gyro.x);
//  Serial.print(" \tY: ");
//  Serial.print(gyro.gyro.y);
//  Serial.print(" \tZ: ");
//  Serial.print(gyro.gyro.z);
//  Serial.println(" radians/s ");
//  Serial.println();

  delay(500);

}
