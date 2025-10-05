// https://www.esp8266learning.com/esp8266-lsm6ds3-accelerometer-gyroscope-example.php
// https://www.esp8266learning.com/wemos-hmc5883l-example.php

#include <SparkFunLSM6DS3.h>
#include <DFRobot_QMC5883.h>

#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B
DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");

  //Call .begin() to configure the IMU
  myIMU.begin();

  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  if(compass.isHMC())
  {
    Serial.println("Initialize HMC5883");

    //Set/get the compass signal gain range, default to be 1.3 Ga
    // compass.setRange(HMC5883L_RANGE_1_3GA);
    // Serial.print("compass range is:");
    // Serial.println(compass.getRange());

    //Set/get measurement mode
    // compass.setMeasurementMode(HMC5883L_CONTINOUS);
    // Serial.print("compass measurement mode is:");
    // Serial.println(compass.getMeasurementMode());

    //Set/get the data collection frequency of the sensor
    // compass.setDataRate(HMC5883L_DATARATE_15HZ);
    // Serial.print("compass data rate is:");
    // Serial.println(compass.getDataRate());

    //Get/get sensor status
    // compass.setSamples(HMC5883L_SAMPLES_8);
    // Serial.print("compass samples is:");
    // Serial.println(compass.getSamples());
  }
  else if(compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    Serial.print("compass range is:");
    Serial.println(compass.getRange());

    compass.setMeasurementMode(QMC5883_CONTINOUS);
    Serial.print("compass measurement mode is:");
    Serial.println(compass.getMeasurementMode());

    compass.setDataRate(QMC5883_DATARATE_50HZ);
    Serial.print("compass data rate is:");
    Serial.println(compass.getDataRate());

    compass.setSamples(QMC5883_SAMPLES_8);
    Serial.print("compass samples is:");
    Serial.println(compass.getSamples());
  }
  else if(compass.isVCM())
  {
    Serial.println("Initialize VCM5883L");
    // compass.setMeasurementMode(VCM5883L_CONTINOUS);
    // Serial.print("compass measurement mode is:");
    // Serial.println(compass.getMeasurementMode());

    // compass.setDataRate(VCM5883L_DATARATE_200HZ);
    // Serial.print("compass data rate is:");
    // Serial.println(compass.getDataRate());
  }
}

void loop()
{
  //Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.print(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.print(" ");
  Serial.print(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.print(" ");
  Serial.println(myIMU.readFloatAccelZ(), 4);
  Serial.print("\nGyroscope:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 4);

  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(myIMU.readTempC(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(myIMU.readTempF(), 4);

  Serial.print("\nMagnetometer:\n");
  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on: http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degrees = ");
  Serial.println(mag.HeadingDegress);

  delay(100);
}