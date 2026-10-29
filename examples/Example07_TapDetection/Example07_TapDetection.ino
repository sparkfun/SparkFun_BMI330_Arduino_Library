/*
  Example 07 - Tap Detection

  Single, double and triple tap, detected entirely on the BMI330. The host is told a gesture
  happened; it does not have to watch the accelerometer to find one.

  ONE INTERRUPT, THREE GESTURES. The device raises a single tap interrupt and reports which
  gesture it was in a separate register. The sequence is always: see the flag, then call
  getTapEvent().

  TWO SETTINGS DECIDE WHETHER TAP WORKS AT ALL:

    THE AXIS. tap.axis is the axis the detector watches, and it watches ONLY that one. Z is the
    board's face - right if you tap the top of a board lying flat. If the board is mounted on
    edge, or you tap the enclosure from the side, the impulse lands on X or Y and a Z-axis
    detector never sees it. If nothing registers, change kTapAxis first.

    THE THRESHOLD. tap.peakThreshold too low and footsteps near the bench register; too high and
    nothing does. The right value depends on what the board is mounted in - a tap through 3 mm
    of acrylic is a very different signal from a tap on a bare PCB.

  The values below are Bosch's reference configuration for this part. The duration fields are in
  20 ms ticks. waitForTimeout = true waits out the gesture window and reports the CONFIRMED
  gesture; false reports as soon as something is seen, so a triple tap arrives as a single first.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC
  D2    --> INT1

  Open the Serial Monitor at 115200 baud, then tap the board once, twice or three times.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

const uint8_t kInterruptPin = 2;

// Which axis to watch. Change this first if nothing registers.
const sfe_bmi330_tap_axis_t kTapAxis = BMI330_TAP_AXIS_Z;

volatile bool interruptFired = false;

void bmi330ISR()
{
    interruptFired = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 7 - Tap Detection");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // Tap needs a fast accelerometer - the impulse is short, and at 50 Hz it falls between
    // samples. +/-8 g keeps a firm tap inside the range; at +/-2 g the peak would clip.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_400_HZ, BMI330_ACCEL_RANGE_8G);

    // Bosch's reference tap configuration for this part.
    sfe_bmi330_tap_config_t tap;
    tap.axis = kTapAxis;
    tap.mode = 1;                  // 0 sensitive, 1 normal, 2 robust.
    tap.peakThreshold = 0x2C;      // The knob to tune. Raise it if stray taps appear.
    tap.maxPeaksForTap = 5;
    tap.maxGestureDuration = 0x11; // 340 ms for the whole gesture.
    tap.maxDurationBetweenPeaks = 5;
    tap.shockSettlingDuration = 5;
    tap.minQuietBetweenTaps = 7;   // 140 ms of quiet required between taps.
    tap.quietTimeAfterGesture = 5;
    tap.waitForTimeout = true;     // Report the confirmed gesture, not the first thing seen.
    myIMU.setTapConfig(tap);

    sfe_bmi330_int_pin_config_t pinConfig; // Push-pull, active high
    myIMU.setInterruptPinConfig(BMI330_INT1, pinConfig);
    myIMU.mapInterrupt(BMI330_INT_SRC_TAP, BMI330_INT1);

    // Enabling only single tap makes detection quicker, because the device stops waiting to see
    // whether a second tap follows.
    myIMU.enableFeature(BMI330_FEAT_SINGLE_TAP | BMI330_FEAT_DOUBLE_TAP | BMI330_FEAT_TRIPLE_TAP, true);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), bmi330ISR, RISING);

    Serial.println("Tap the board. Nothing registering? Change kTapAxis, or lower tap.peakThreshold.");
}

void loop()
{
    if (!interruptFired)
        return;

    interruptFired = false;

    // Clear-on-read: one read consumes every pending INT1 flag.
    uint16_t status = 0;
    myIMU.getInterruptStatus(BMI330_INT1, status);

    if (!(status & BMI330_INT_STATUS_TAP))
        return;

    sfe_bmi330_tap_event_t event;
    myIMU.getTapEvent(event);

    if (event.tripleTap)
        Serial.println("Triple tap!");
    else if (event.doubleTap)
        Serial.println("Double tap!");
    else if (event.singleTap)
        Serial.println("Single tap!");
}
