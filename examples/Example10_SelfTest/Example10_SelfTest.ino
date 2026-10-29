/*
  Example 10 - Self-Test

  The BMI330 can check its own sensors. Self-test drives a known electrostatic stimulus into
  the accelerometer and gyroscope and compares the response against factory limits, reporting a
  pass or fail per axis. It is the right thing to run at power-up on anything where a silently
  dead axis would matter.

  What it tells you, and what it does not: a pass means the MEMS structures move as they
  should and the signal chain is intact. It says nothing about calibration or mounting - a
  perfectly healthy sensor glued on crooked passes every time. For offset and sensitivity, see
  Example 11.

  THREE THINGS TO KNOW BEFORE RUNNING IT:

    1. Keep the board STILL. The test measures a small deliberate deflection; external motion
       lands on top of it and produces failures on a good part.

    2. It is disruptive. The routine forces the accelerometer into high-performance mode at
       100 Hz and takes up to 350 ms. This library saves your accelerometer configuration and
       restores it afterwards, but any sampling in flight is interrupted regardless.

    3. The BMI330 needs its own gyroscope filter coefficients seeded into feature memory first,
       and they are NOT the same as the BMI323's. The library handles this inside
       performSelfTest(); a driver ported from the BMI323 without that step reports failures on
       healthy parts.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC

  Leave the board on a table and do not touch it while the test runs.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

void printResult(const char *label, bool ok)
{
    Serial.print("  ");
    Serial.print(label);
    Serial.println(ok ? "PASS" : "FAIL");
}

void runSelfTest(const char *what, sfe_bmi330_self_test_t selection)
{
    Serial.print("Running self-test: ");
    Serial.println(what);

    sfe_bmi330_self_test_result_t result;
    sfTkError_t rc = myIMU.performSelfTest(selection, result);

    if (rc != ksfTkErrOk || !result.completed)
    {
        // "Did not finish" is a different problem from "finished and failed" - most often a
        // precondition was not met, which shows up as error status 0x0C.
        Serial.print("  The test did not complete. Feature engine error status: 0x");
        Serial.println(result.errorStatus, HEX);
        Serial.println("  0x0C means a precondition failed. 0x05 means no error.");
        Serial.println();
        return;
    }

    Serial.print("  Overall: ");
    Serial.println(result.passed ? "PASS" : "FAIL");

    if (selection != BMI330_SELF_TEST_GYRO)
    {
        printResult("Accel X:     ", result.accelXOk);
        printResult("Accel Y:     ", result.accelYOk);
        printResult("Accel Z:     ", result.accelZOk);
    }

    if (selection != BMI330_SELF_TEST_ACCEL)
    {
        printResult("Gyro X:      ", result.gyroXOk);
        printResult("Gyro Y:      ", result.gyroYOk);
        printResult("Gyro Z:      ", result.gyroZOk);
        printResult("Gyro drive:  ", result.gyroDriveOk);
    }

    Serial.print("  Feature engine error status: 0x");
    Serial.print(result.errorStatus, HEX);
    Serial.println(" (0x05 = no error)");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 10 - Self-Test");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // The self-test sets its own preconditions, but the sensors have to be running first.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_8G);
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_2000DPS);

    Serial.println();
    Serial.println("Keep the board completely still. Starting in 3 seconds...");
    delay(3000);
    Serial.println();

    // BMI330_SELF_TEST_ACCEL or BMI330_SELF_TEST_GYRO test one sensor on its own.
    runSelfTest("accelerometer and gyroscope", BMI330_SELF_TEST_BOTH);

    Serial.println("Self-test complete. Resuming normal operation.");
    Serial.println();

    // performSelfTest() restores the accelerometer configuration, but set it explicitly here so
    // the sketch does not depend on that.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_4G);

    Serial.println("AccX(g)\tAccY(g)\tAccZ(g)");
}

void loop()
{
    sfe_bmi330_data_t accel;

    if (myIMU.getAccelData(accel) == ksfTkErrOk)
    {
        Serial.print(accel.x, 3);
        Serial.print("\t");
        Serial.print(accel.y, 3);
        Serial.print("\t");
        Serial.println(accel.z, 3);
    }

    delay(500);
}
