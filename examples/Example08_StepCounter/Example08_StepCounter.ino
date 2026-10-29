/*
  Example 08 - Step Counter and Step Detector

  The BMI330 counts steps by itself. The host can poll the count, or be interrupted every N
  steps, or be interrupted on every single step - three different power and latency trade-offs
  for the same feature.

    Step counter   Maintains a 32-bit count in the device. Interrupts at a watermark, in units
                   of 20 steps. Watermark 0 means "count, but never interrupt" - the right
                   setting when the host polls at its own pace.

    Step detector  Interrupts on every step. Costs an interrupt per step, so it is for
                   something that must react immediately, not for counting.

  This sketch enables both: the detector so you can see each step land, and the counter with a
  20-step watermark so you can see the milestone interrupt arrive.

  THE COUNT IS 32 BITS ACROSS TWO REGISTERS. getStepCount() reads them in one burst, so the
  halves cannot disagree. Reading them separately could straddle a carry and report a count
  that never existed - unlikely, but it is a real class of bug and it costs nothing to avoid.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC
  D2    --> INT1  (step detector - every step)
  D3    --> INT2  (step counter  - every 20 steps)

  Open the Serial Monitor at 115200 baud and carry the board around while you walk.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

const uint8_t kStepDetectorPin = 2;
const uint8_t kStepCounterPin = 3;

volatile bool stepDetected = false;
volatile bool watermarkHit = false;

uint32_t lastPoll = 0;

void stepDetectorISR()
{
    stepDetected = true;
}

void stepCounterISR()
{
    watermarkHit = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 8 - Step Counter");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // The step algorithm is tuned for a 50 Hz accelerometer. Normal mode keeps the current
    // down, which is the whole point of counting steps in hardware.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_NORMAL, BMI330_ODR_50_HZ, BMI330_ACCEL_RANGE_8G);

    // Watermark is in units of 20 steps: 1 means interrupt every 20 steps. Set 0 to count
    // without ever interrupting.
    myIMU.setStepCounterWatermark(1);

    sfe_bmi330_int_pin_config_t pinConfig;
    myIMU.setInterruptPinConfig(BMI330_INT1, pinConfig);
    myIMU.setInterruptPinConfig(BMI330_INT2, pinConfig);

    myIMU.mapInterrupt(BMI330_INT_SRC_STEP_DETECTOR, BMI330_INT1);
    myIMU.mapInterrupt(BMI330_INT_SRC_STEP_COUNTER, BMI330_INT2);

    myIMU.enableFeature(BMI330_FEAT_STEP_DETECTOR | BMI330_FEAT_STEP_COUNTER, true);

    // Start from zero so the numbers below mean something.
    myIMU.resetStepCount();

    pinMode(kStepDetectorPin, INPUT);
    pinMode(kStepCounterPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kStepDetectorPin), stepDetectorISR, RISING);
    attachInterrupt(digitalPinToInterrupt(kStepCounterPin), stepCounterISR, RISING);

    Serial.println();
    Serial.println("Ready. Carry the board and walk.");
    Serial.println("The algorithm wants a few consistent steps before it starts counting -");
    Serial.println("a couple of paces of warm-up is normal, not a fault.");
    Serial.println();

    lastPoll = millis();
}

void loop()
{
    if (stepDetected)
    {
        stepDetected = false;

        uint16_t status = 0;
        myIMU.getInterruptStatus(BMI330_INT1, status);

        if (status & BMI330_INT_STATUS_STEP_DETECTOR)
            Serial.println("  step");
    }

    if (watermarkHit)
    {
        watermarkHit = false;

        uint16_t status = 0;
        myIMU.getInterruptStatus(BMI330_INT2, status);

        if (status & BMI330_INT_STATUS_STEP_COUNTER)
        {
            uint32_t steps = 0;
            myIMU.getStepCount(steps);

            Serial.print(">>> Watermark: ");
            Serial.print(steps);
            Serial.println(" steps");
        }
    }

    // Polling the count directly - no interrupt needed. This is all a pedometer really has to
    // do, and with watermark 0 it costs one register read whenever the host feels like it.
    if (millis() - lastPoll >= 5000)
    {
        uint32_t steps = 0;
        myIMU.getStepCount(steps);

        Serial.print("[poll] total steps: ");
        Serial.println(steps);

        lastPoll = millis();
    }
}
