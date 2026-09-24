/*
  Example 05 - FIFO Buffer

  The BMI330 has a 2 kB FIFO. Instead of one interrupt per sample, it can collect samples and
  interrupt once when it has enough of them - which is how you sample at 400 Hz without the
  host servicing 400 interrupts a second, and how the host can sleep between bursts.

  This sketch configures the FIFO for accelerometer, gyroscope and sensor time, sets a
  watermark, routes the watermark interrupt to INT2, and decodes frames when it fires.

  THE FIFO IS HEADERLESS. Frames carry no tag saying what is in them. The order is fixed by
  hardware - accelerometer (3 words), gyroscope (3 words), temperature (1 word), sensor time
  (1 word) - with disabled sources simply absent. That means the parser has to be told the same
  enable set that was programmed, which is why readFifoData() takes it as an argument. Pass the
  wrong one and the data decodes into plausible nonsense rather than failing.

  WATERMARK IS IN WORDS, NOT FRAMES. A watermark of 80 words with accelerometer plus gyroscope
  plus time enabled (7 words per frame) is about 11 frames. frameLengthWords() does the
  arithmetic so you do not have to.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC
  D3    --> INT2

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

const uint8_t kInterruptPin = 3;

// What the FIFO records. Temperature is left out here - it changes far too slowly to be worth a
// word in every frame.
const uint16_t kFifoSources = BMI330_FIFO_ACCEL | BMI330_FIFO_GYRO | BMI330_FIFO_TIME;

// Collect roughly this many frames before interrupting.
const uint8_t kFramesPerBurst = 20;

volatile bool interruptFired = false;

// Decoded frames land here. Sized to the burst, with headroom in case the FIFO ran slightly
// ahead of the host.
sfe_bmi330_fifo_frame_t frames[32];

void bmi330ISR()
{
    interruptFired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 5 - FIFO Buffer");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // 100 Hz keeps the printing readable. The FIFO earns its keep at much higher rates.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_4G);
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_500DPS);

    // Overwrite the oldest data when full rather than stopping. For streaming, losing the
    // stale end of the buffer is better than losing the live end.
    myIMU.setFifoConfig(kFifoSources, false);

    uint8_t wordsPerFrame = sfDevBMI330::frameLengthWords(kFifoSources);
    uint16_t watermarkWords = (uint16_t)wordsPerFrame * kFramesPerBurst;
    myIMU.setFifoWatermark(watermarkWords);

    Serial.print("Frame length: ");
    Serial.print(wordsPerFrame);
    Serial.print(" words. Watermark: ");
    Serial.print(watermarkWords);
    Serial.print(" words (");
    Serial.print(kFramesPerBurst);
    Serial.println(" frames).");

    sfe_bmi330_int_pin_config_t pinConfig;
    myIMU.setInterruptPinConfig(BMI330_INT2, pinConfig);
    myIMU.mapInterrupt(BMI330_INT_SRC_FIFO_WATERMARK, BMI330_INT2);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), bmi330ISR, RISING);

    // Start from an empty buffer - anything captured during configuration is not worth keeping.
    myIMU.flushFifo();

    Serial.println();
    Serial.println("Waiting for the FIFO to fill...");
}

void loop()
{
    if (!interruptFired)
        return;

    interruptFired = false;

    uint16_t status = 0;
    myIMU.getInterruptStatus(BMI330_INT2, status);

    if (!(status & BMI330_INT_STATUS_FIFO_WATERMARK))
        return;

    uint16_t fillLevel = 0;
    myIMU.getFifoFillLevel(fillLevel);

    uint8_t numFrames = 0;

    // Only whole frames come out. A partial frame at the end of the buffer is left in place and
    // arrives complete on the next read.
    if (myIMU.readFifoData(frames, sizeof(frames) / sizeof(frames[0]), numFrames, kFifoSources) !=
        ksfTkErrOk)
    {
        Serial.println("FIFO read failed!");
        return;
    }

    Serial.print("--- Watermark hit. Fill level ");
    Serial.print(fillLevel);
    Serial.print(" words, decoded ");
    Serial.print(numFrames);
    Serial.println(" frames ---");

    // Print the first and last frame of the burst. Printing all twenty at 100 Hz would put the
    // serial port permanently behind the sensor.
    for (uint8_t i = 0; i < numFrames; i++)
    {
        if (i != 0 && i != numFrames - 1)
            continue;

        Serial.print("  frame ");
        Serial.print(i);
        Serial.print(": ");

        if (frames[i].hasAccel)
        {
            Serial.print("accel ");
            Serial.print(myIMU.accelRawToG(frames[i].accel.x), 3);
            Serial.print(", ");
            Serial.print(myIMU.accelRawToG(frames[i].accel.y), 3);
            Serial.print(", ");
            Serial.print(myIMU.accelRawToG(frames[i].accel.z), 3);
            Serial.print(" g  ");
        }
        else
        {
            // The accelerometer was idle when this frame was captured, so the device wrote a
            // sentinel rather than a sample.
            Serial.print("accel (no data)  ");
        }

        if (frames[i].hasGyro)
        {
            Serial.print("gyro ");
            Serial.print(myIMU.gyroRawToDps(frames[i].gyro.x), 1);
            Serial.print(", ");
            Serial.print(myIMU.gyroRawToDps(frames[i].gyro.y), 1);
            Serial.print(", ");
            Serial.print(myIMU.gyroRawToDps(frames[i].gyro.z), 1);
            Serial.print(" dps  ");
        }

        if (frames[i].hasTime)
        {
            Serial.print("t=");
            Serial.print(frames[i].sensorTime);
        }

        Serial.println();
    }
}
