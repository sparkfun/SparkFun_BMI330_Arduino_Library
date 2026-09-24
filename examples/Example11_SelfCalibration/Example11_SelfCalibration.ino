/*
  Example 11 - Gyroscope Self-Calibration

  Every MEMS gyroscope has a zero-rate offset: it reports a small non-zero rate while
  perfectly still. That offset is what makes an integrated heading drift, and it is the single
  biggest thing between a raw gyroscope and a usable one. The BMI330 can measure and correct
  its own.

  This sketch prints the resting gyroscope output before and after calibration, so you can see
  the offset collapse.

  THE PRECONDITIONS ARE NOT OPTIONAL:

    1. The device must be MOTIONLESS. This is the important one. The routine assumes any rate
       it sees is offset, so a calibration run on a moving device reports success and leaves an
       offset WORSE than the one it started with. There is no error flag for "you moved it".

    2. The accelerometer must be in high-performance mode at 100 Hz and +/-8 g. The library
       sets that up and restores your configuration afterwards.

    3. It takes up to 430 ms.

  WHAT GETS CORRECTED, AND WHERE IT LIVES: pass BMI330_SELF_CALIB_OFFSET, _SENSITIVITY or
  _BOTH. With applyCorrection true, the device applies the result to its own output
  immediately. The correction lives in volatile registers - it does NOT survive a power cycle
  or a soft reset, so either re-run this at boot or read the offsets out with getGyroOffset()
  and store them yourself.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC

  Put the board on a solid surface and do not touch it while this runs.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

// Average the gyroscope over a second of standing still. Whatever this reads is offset.
void measureRestingOffset(const char *label)
{
    const uint16_t kSamples = 100;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (uint16_t i = 0; i < kSamples; i++)
    {
        sfe_bmi330_data_t gyro;

        if (myIMU.getGyroData(gyro) == ksfTkErrOk)
        {
            sumX += gyro.x;
            sumY += gyro.y;
            sumZ += gyro.z;
        }

        delay(10);
    }

    Serial.print(label);
    Serial.print(sumX / kSamples, 3);
    Serial.print(", ");
    Serial.print(sumY / kSamples, 3);
    Serial.print(", ");
    Serial.print(sumZ / kSamples, 3);
    Serial.println(" dps");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 11 - Gyroscope Self-Calibration");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_8G);

    // The narrowest gyroscope range gives the finest resolution, which makes a small offset
    // easy to see.
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_125DPS);

    Serial.println();
    Serial.println("Place the board on a solid surface. Do not touch it.");
    Serial.println("Starting in 3 seconds...");
    delay(3000);

    Serial.println();
    measureRestingOffset("Resting rate BEFORE calibration: ");

    int16_t ox = 0, oy = 0, oz = 0;
    myIMU.getGyroOffset(ox, oy, oz);
    Serial.print("Stored offsets before: ");
    Serial.print(ox);
    Serial.print(", ");
    Serial.print(oy);
    Serial.print(", ");
    Serial.println(oz);

    Serial.println();
    Serial.println("Calibrating - keep still...");

    sfe_bmi330_self_calib_result_t result;
    sfTkError_t rc = myIMU.performGyroSelfCalibration(BMI330_SELF_CALIB_BOTH, true, result);

    if (rc != ksfTkErrOk || !result.completed)
    {
        Serial.print("Calibration did not complete. Feature engine error status: 0x");
        Serial.println(result.errorStatus, HEX);
        Serial.println("0x0C means a precondition failed - check the accelerometer is enabled.");
        return;
    }

    Serial.print("Calibration finished: ");
    Serial.println(result.passed ? "PASS" : "FAIL");
    Serial.print("Feature engine error status: 0x");
    Serial.print(result.errorStatus, HEX);
    Serial.println(" (0x05 = no error)");

    // Restore the range the sketch wants - the routine reconfigured the accelerometer, and
    // this puts the gyroscope back on the setting we are measuring with.
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_125DPS);
    delay(100);

    Serial.println();
    measureRestingOffset("Resting rate AFTER calibration:  ");

    myIMU.getGyroOffset(ox, oy, oz);
    Serial.print("Stored offsets after:  ");
    Serial.print(ox);
    Serial.print(", ");
    Serial.print(oy);
    Serial.print(", ");
    Serial.println(oz);

    Serial.println();
    Serial.println("These offsets are volatile - they are lost on reset or power-down.");
    Serial.println("Save them and restore with setGyroOffset(), or re-run this at every boot.");
    Serial.println();
    Serial.println("GyrX(dps)\tGyrY(dps)\tGyrZ(dps)");
}

void loop()
{
    sfe_bmi330_data_t gyro;

    if (myIMU.getGyroData(gyro) == ksfTkErrOk)
    {
        Serial.print(gyro.x, 3);
        Serial.print("\t\t");
        Serial.print(gyro.y, 3);
        Serial.print("\t\t");
        Serial.println(gyro.z, 3);
    }

    delay(500);
}
