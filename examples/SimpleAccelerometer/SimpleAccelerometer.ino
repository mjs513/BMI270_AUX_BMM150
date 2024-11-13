/*
  BMI270_AUX_BMM150 - Simple Accelerometer

  This example reads the acceleration values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

History:
 Riccardo Rizzo - created 10 Jul 2019
 Yahya Tawil - ported from Arduino BMI270 library to BMI270_AUX_BMM150  13 May 2023 

  This example code is in the public domain.
*/

#include "BMI270_AUX_BMM150.h"

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  IMU.setAcellConfig(BMI2_ACC_ODR_100HZ, BMI2_ACC_RANGE_2G, BMI2_ACC_NORMAL_AVG4);


  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    Serial.print(millis());Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
}
