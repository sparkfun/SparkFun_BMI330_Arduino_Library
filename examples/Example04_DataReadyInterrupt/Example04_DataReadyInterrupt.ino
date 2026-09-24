/*
  Example 04 - Data Ready Interrupt

  Instead of polling, let the BMI330 tell the host when a sample is ready. The breakout brings
  out both interrupt pins, and this sketch uses INT1.

  Polling a 100 Hz sensor from loop() either reads the same sample twice or misses one, because
  the host's loop rate and the sensor's sample rate are unrelated. An interrupt ties them
  together: exactly one read per sample, no duplicates, no gaps.

  Three settings matter here:

    Pin behavior  Push-pull, active high, and enabled. Open-drain is for sharing one host line
                  between several devices - supply a pull-up if you use it, because the board
                  does not fit one.

    Latch mode    Pulsed (the default) asserts the pin briefly per event. Latched holds it until
                  the status register is read. Use latched if your host might miss a short
                  pulse; the cost is that the status read becomes mandatory.

    Routing       Every source is routed independently, so data-ready can sit on INT1 while
                  motion events go to INT2 - see Example 06.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC
  D2    --> INT1

  Pick a pin that supports attachInterrupt() on your board.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

const uint8_t kInterruptPin = 2;

// Set in the ISR, cleared in loop(). volatile because the compiler cannot see the ISR writing it.
volatile bool interruptFired = false;

uint32_t samplesRead = 0;
uint32_t lastReport = 0;

void bmi330ISR()
{
    // Keep the ISR to a flag. Reading the sensor here would mean I2C traffic from interrupt
    // context, which is a good way to hang a Wire library that was not designed for it.
    interruptFired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 4 - Data Ready Interrupt");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // 100 Hz on both sensors.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_4G);
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_500DPS);

    // Push-pull, active high, enabled.
    sfe_bmi330_int_pin_config_t pinConfig;
    pinConfig.enabled = true;
    pinConfig.activeHigh = true;
    pinConfig.openDrain = false;
    myIMU.setInterruptPinConfig(BMI330_INT1, pinConfig);

    // Pulsed. The sketch reads the status register every time anyway, so latching would work
    // equally well here - try it by passing true.
    myIMU.setInterruptLatch(false);

    // Route accelerometer data-ready to INT1. The gyroscope runs at the same rate, so one
    // source is enough to pace the reads; routing both would just double the interrupts.
    myIMU.mapInterrupt(BMI330_INT_SRC_ACCEL_DRDY, BMI330_INT1);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), bmi330ISR, RISING);

    Serial.println("Waiting for interrupts on INT1...");
    Serial.println();

    lastReport = millis();
}

void loop()
{
    if (interruptFired)
    {
        interruptFired = false;

        // Clear-on-read: this one read consumes every pending flag for INT1, so test the whole
        // word rather than reading again per bit.
        uint16_t status = 0;
        myIMU.getInterruptStatus(BMI330_INT1, status);

        if (status & BMI330_INT_STATUS_ACCEL_DRDY)
        {
            sfe_bmi330_sensor_data_t data;
            myIMU.getSensorData(data);
            samplesRead++;

            // Print sparingly - at 100 Hz, printing every sample would make the serial port the
            // bottleneck and cost you interrupts.
            if (samplesRead % 100 == 0)
            {
                Serial.print("Accel: ");
                Serial.print(data.accel.x, 3);
                Serial.print(", ");
                Serial.print(data.accel.y, 3);
                Serial.print(", ");
                Serial.print(data.accel.z, 3);
                Serial.print(" g   Sensor time: ");
                Serial.println(data.sensorTime);
            }
        }
    }

    // Once a second, report the measured rate. It should sit very close to the configured ODR;
    // if it is low, the host is missing interrupts.
    if (millis() - lastReport >= 1000)
    {
        Serial.print(">>> ");
        Serial.print(samplesRead);
        Serial.println(" samples in the last second (expect ~100)");

        samplesRead = 0;
        lastReport = millis();
    }
}
