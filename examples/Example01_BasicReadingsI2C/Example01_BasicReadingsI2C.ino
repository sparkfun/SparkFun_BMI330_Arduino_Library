/*
  Example 01 - Basic Readings (I2C)

  Reads acceleration, angular rate and die temperature from a SparkFun Qwiic BMI330 breakout
  and prints them once every 100 ms.

  Two things to know about a fresh BMI330:

    1. Both sensors power up DISABLED. begin() leaves them that way on purpose, because the
       right power mode is an application decision. Nothing is read until enableAccel() and
       enableGyro() have run.

    2. The default ranges are +/-2 g and +/-2000 dps. Picking the smallest range your
       application can survive is free resolution - the output is always 16 bits.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC

  The board answers at 0x68 as shipped. Close the ADR jumper for 0x69.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 1 - Basic Readings (I2C)");

    Wire.begin();

    // Use SFE_BMI330_ALT_ADDRESS here if the ADR jumper is closed.
    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring and the ADR jumper!");
        delay(1000);
    }

    uint8_t chipId = 0, revId = 0;
    myIMU.getChipID(chipId, revId);

    Serial.print("Connected to a BMI330 at 0x");
    Serial.print(myIMU.getAddress(), HEX);
    Serial.print(" (chip ID 0x");
    Serial.print(chipId, HEX);
    Serial.print(", rev ");
    Serial.print(revId);
    Serial.println(")");

    // Turn both sensors on at 100 Hz. +/-4 g and +/-500 dps suit general handheld motion: fast
    // enough to see a shake, fine enough to resolve a slow tilt.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_4G);
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_500DPS);

    // An ODR the selected power mode cannot support does not fail the write - it shows up here.
    sfe_bmi330_error_status_t errors;
    myIMU.getErrorStatus(errors);

    if (errors.accelConfError || errors.gyroConfError)
        Serial.println("WARNING: the device rejected a sensor configuration.");

    Serial.println();
    Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tGyrX(dps)\tGyrY(dps)\tGyrZ(dps)\tTemp(C)");
}

void loop()
{
    sfe_bmi330_sensor_data_t data;

    // One burst read, so every field below describes the same instant.
    if (myIMU.getSensorData(data) != ksfTkErrOk)
    {
        Serial.println("Read failed!");
        delay(1000);
        return;
    }

    Serial.print(data.accel.x, 3);
    Serial.print("\t");
    Serial.print(data.accel.y, 3);
    Serial.print("\t");
    Serial.print(data.accel.z, 3);
    Serial.print("\t");
    Serial.print(data.gyro.x, 2);
    Serial.print("\t\t");
    Serial.print(data.gyro.y, 2);
    Serial.print("\t\t");
    Serial.print(data.gyro.z, 2);
    Serial.print("\t\t");
    Serial.println(data.temperature, 1);

    delay(100);
}
