#include <SparkFunLSM6DS3.h>
#include <DFRobot_QMC5883.h>

#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B
DFRobot_QMC5883 myCompass(&Wire, QMC5883_ADDRESS);

void setup() {
  Serial.begin(115200);
  delay(1000);

  myIMU.settings.gyroRange = 245; // max deg/sec
  myIMU.settings.accelRange = 2;  // max g

  //Call .begin() to configure the IMU
  while (myIMU.begin() != IMU_SUCCESS) {
    Serial.println("Failed to initialize LSM6DS3!");
    delay(500);
  }

  while (!myCompass.begin()) {
    Serial.println("Failed to initialize QMC5883!");
    delay(500);
  }

  // Initialize QMC5883
  myCompass.setRange(QMC5883_RANGE_2GA);
  myCompass.setMeasurementMode(QMC5883_CONTINOUS);
  myCompass.setDataRate(QMC5883_DATARATE_50HZ);
  myCompass.setSamples(QMC5883_SAMPLES_8);
}

void loop()
{
  // Print out the current time, followed by 3 accelerometer, gyroscope and magnetometer values
  unsigned long time = millis();
  Serial.print(time);
  Serial.print(" ");
  Serial.print(myIMU.readRawAccelX());
  Serial.print(" ");
  Serial.print(myIMU.readRawAccelY());
  Serial.print(" ");
  Serial.print(myIMU.readRawAccelZ());
  Serial.print(" ");
  Serial.print(myIMU.readRawGyroX());
  Serial.print(" ");
  Serial.print(myIMU.readRawGyroY());
  Serial.print(" ");
  Serial.print(myIMU.readRawGyroZ());
  Serial.print(" ");
  sVector_t mag = myCompass.readRaw();
  Serial.print(mag.XAxis);
  Serial.print(" ");
  Serial.print(mag.YAxis);
  Serial.print(" ");
  Serial.println(mag.ZAxis);

  delay(100);
}