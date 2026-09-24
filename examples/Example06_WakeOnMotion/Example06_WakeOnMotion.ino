/*
  Example 06 - Wake on Motion (Any-Motion, No-Motion and Low Power)

  The first of the feature-engine examples, and the pattern most battery-powered designs end up
  wanting: watch for motion in hardware at a few tens of microamps, run at full rate only while
  something is happening, and drop back to low power when it stops.

    Any-motion  fires when the board starts moving.  Routed to INT1 - it wakes the design.
    No-motion   fires when it has been still a while. Routed to INT2 - it puts it back to sleep.

  Separate pins let an ISR tell the two apart without a bus transaction.

  THE UNITS ARE THE HARDWARE'S, NOT SI:

    slopeThreshold  1.953 mg per LSB (1/512 g). A threshold of 100 is about 195 mg.
    hysteresis      Same 1.953 mg per LSB.
    duration        20 ms per LSB. A duration of 10 is 200 ms.
    waitTime        20 ms per LSB, 0 to 7.

  These detectors watch the SLOPE of acceleration, not its magnitude. A device sitting still at
  any angle reads 1 g of gravity but zero slope - which is why no-motion works in any
  orientation. Both are enabled per axis: pass BMI330_FEAT_ANY_MOTION_Z alone, say, to ignore
  motion you do not care about.

  THE POWER MODES, ROUGHLY:

    Low-power accel   Accelerometer duty-cycled, gyroscope off. About 45 uA, and enough to run
                      the feature engine.
    High-perf IMU     Both sensors continuous. Around 790 uA.

  GYRO SUSPEND VERSUS DISABLE. DISABLE (used here) turns the drive circuit off; waking costs the
  gyroscope's full start-up time, tens of milliseconds. SUSPEND keeps the drive running and only
  the ADC off, so waking is quick at the cost of standing current.

  Low-power mode is limited to 400 Hz and below. A rate the mode cannot support does not fail the
  write - it sets a flag in ERR_REG (see Example 03).

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC
  D2    --> INT1  (any-motion)
  D3    --> INT2  (no-motion)

  Open the Serial Monitor at 115200 baud. Pick the board up to wake it, put it down to let it
  sleep again.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

const uint8_t kAnyMotionPin = 2;
const uint8_t kNoMotionPin = 3;

volatile bool anyMotionFired = false;
volatile bool noMotionFired = false;

bool awake = false;
uint32_t lastPrint = 0;

void anyMotionISR()
{
    anyMotionFired = true;
}

void noMotionISR()
{
    noMotionFired = true;
}

// Low-power watch: accelerometer duty-cycled at 50 Hz, gyroscope fully off. The feature engine
// keeps running, so both detectors stay armed.
void enterLowPower()
{
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_LOW_POWER, BMI330_ODR_50_HZ, BMI330_ACCEL_RANGE_4G,
                         BMI330_BW_ODR_HALF, BMI330_AVG_4);
    myIMU.setGyroMode(BMI330_GYRO_MODE_DISABLE);
    awake = false;
    Serial.println("[INT2] Still - dropping to low power (accel duty-cycled, gyro off)");
}

// Full rate: both sensors in high-performance mode.
void enterActive()
{
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_200_HZ, BMI330_ACCEL_RANGE_4G);
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_200_HZ, BMI330_GYRO_RANGE_500DPS);

    // The gyroscope needs a moment after its drive circuit starts before the data means anything.
    // This is the cost that BMI330_GYRO_MODE_SUSPEND buys out.
    delay(50);
    awake = true;
    Serial.println("[INT1] Motion - waking to full rate");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 6 - Wake on Motion");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // Any-motion: about 195 mg of slope, sustained for 200 ms. Low enough to catch a deliberate
    // pick-up, high enough to ignore a desk being leaned on.
    sfe_bmi330_motion_config_t anyMotion;
    anyMotion.slopeThreshold = 100; // 100 x 1.953 mg = 195 mg
    anyMotion.duration = 10;        // 10 x 20 ms = 200 ms
    myIMU.setAnyMotionConfig(anyMotion);

    // No-motion: under about 39 mg of slope for two full seconds before calling it still. The long
    // duration is deliberate - a short one fires between footsteps.
    sfe_bmi330_motion_config_t noMotion;
    noMotion.slopeThreshold = 20; // 20 x 1.953 mg = 39 mg
    noMotion.duration = 100;      // 100 x 20 ms = 2000 ms
    myIMU.setNoMotionConfig(noMotion);

    sfe_bmi330_int_pin_config_t pinConfig; // Push-pull, active high
    myIMU.setInterruptPinConfig(BMI330_INT1, pinConfig);
    myIMU.setInterruptPinConfig(BMI330_INT2, pinConfig);

    myIMU.mapInterrupt(BMI330_INT_SRC_ANY_MOTION, BMI330_INT1);
    myIMU.mapInterrupt(BMI330_INT_SRC_NO_MOTION, BMI330_INT2);

    // All three axes for both detectors.
    myIMU.enableFeature(BMI330_FEAT_ANY_MOTION | BMI330_FEAT_NO_MOTION, true);

    pinMode(kAnyMotionPin, INPUT);
    pinMode(kNoMotionPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kAnyMotionPin), anyMotionISR, RISING);
    attachInterrupt(digitalPinToInterrupt(kNoMotionPin), noMotionISR, RISING);

    enterLowPower();
}

void loop()
{
    if (anyMotionFired)
    {
        anyMotionFired = false;

        uint16_t status = 0;
        myIMU.getInterruptStatus(BMI330_INT1, status);

        if ((status & BMI330_INT_STATUS_ANY_MOTION) && !awake)
            enterActive();
    }

    if (noMotionFired)
    {
        noMotionFired = false;

        uint16_t status = 0;
        myIMU.getInterruptStatus(BMI330_INT2, status);

        if ((status & BMI330_INT_STATUS_NO_MOTION) && awake)
            enterLowPower();
    }

    // While awake, do the real work. A real design would also sleep the host while !awake and let
    // INT1 wake it.
    if (awake && millis() - lastPrint >= 250)
    {
        lastPrint = millis();

        sfe_bmi330_sensor_data_t data;
        if (myIMU.getSensorData(data) == ksfTkErrOk)
        {
            Serial.print("  Accel (g): ");
            Serial.print(data.accel.x, 2);
            Serial.print(", ");
            Serial.print(data.accel.y, 2);
            Serial.print(", ");
            Serial.print(data.accel.z, 2);
            Serial.print("   Gyro (dps): ");
            Serial.print(data.gyro.x, 1);
            Serial.print(", ");
            Serial.print(data.gyro.y, 1);
            Serial.print(", ");
            Serial.println(data.gyro.z, 1);
        }
    }
}
