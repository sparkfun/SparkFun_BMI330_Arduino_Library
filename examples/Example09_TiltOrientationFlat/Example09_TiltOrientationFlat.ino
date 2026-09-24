/*
  Example 09 - Tilt, Orientation and Flat

  Three posture features that all answer "which way is the board pointing?", from different
  angles:

    Tilt          Fires when the gravity vector has moved more than a set angle away from a
                  slowly-updated reference. It detects the CHANGE, not the absolute pose, which
                  is what you want for "has this been knocked over?"

    Orientation   Classifies the board as portrait or landscape - and optionally face up or
                  face down - and interrupts on a change. This is the phone-screen-rotation
                  feature. It reports an absolute state you can read at any time.

    Flat          Fires when the board has been lying within a set angle of horizontal for a
                  while. Useful for "is it on the table?"

  All three run off the accelerometer alone, which is why they cost so little: the gyroscope
  can stay off entirely.

  BLOCKING is worth understanding. Each of these has a blocking setting that suppresses
  detection while the board is moving, because gravity cannot be measured reliably during
  acceleration. Higher blocking means fewer false reports and slower response. The defaults are
  a reasonable middle.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC
  D2    --> INT1  (all three features)

  Open the Serial Monitor at 115200 baud, then rotate the board, stand it on edge, and lay it
  flat again.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

const uint8_t kInterruptPin = 2;

volatile bool interruptFired = false;

void bmi330ISR()
{
    interruptFired = true;
}

const char *orientationName(sfe_bmi330_orientation_t orientation)
{
    switch (orientation)
    {
    case BMI330_ORIENT_PORTRAIT_UPRIGHT:
        return "portrait, upright";
    case BMI330_ORIENT_LANDSCAPE_LEFT:
        return "landscape, left";
    case BMI330_ORIENT_PORTRAIT_UPSIDE_DOWN:
        return "portrait, upside down";
    case BMI330_ORIENT_LANDSCAPE_RIGHT:
        return "landscape, right";
    default:
        return "unknown";
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 9 - Tilt, Orientation and Flat");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // Posture features only need the accelerometer, and only slowly. Leaving the gyroscope off
    // saves the large majority of the part's current.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_NORMAL, BMI330_ODR_50_HZ, BMI330_ACCEL_RANGE_4G);

    // Tilt: report a departure of roughly 35 degrees from the reference vector. minTiltAngle is
    // stored as 256 x cos(angle), so 210 is about 35 degrees and a LARGER number is a SMALLER
    // angle - easy to get backwards.
    sfe_bmi330_tilt_config_t tilt;
    tilt.segmentSize = 100;   // 2 s of averaging into the reference vector.
    tilt.minTiltAngle = 210;  // ~35 degrees.
    myIMU.setTiltConfig(tilt);

    // Orientation, with face up / face down classification turned on.
    sfe_bmi330_orientation_config_t orientation;
    orientation.faceUpDownEnabled = true;
    orientation.mode = 0;       // Symmetrical.
    orientation.blocking = 3;   // Most aggressive - ignore everything while moving.
    orientation.theta = 40;
    orientation.holdTime = 5;   // 100 ms of stability before reporting.
    myIMU.setOrientationConfig(orientation);

    // Flat: within about 8 units of horizontal, held for 640 ms.
    sfe_bmi330_flat_config_t flat;
    flat.theta = 8;
    flat.blocking = 2;
    flat.holdTime = 32;
    myIMU.setFlatConfig(flat);

    sfe_bmi330_int_pin_config_t pinConfig;
    myIMU.setInterruptPinConfig(BMI330_INT1, pinConfig);

    // All three share INT1. The status word says which one fired, so one pin is enough.
    myIMU.mapInterrupt(BMI330_INT_SRC_TILT, BMI330_INT1);
    myIMU.mapInterrupt(BMI330_INT_SRC_ORIENTATION, BMI330_INT1);
    myIMU.mapInterrupt(BMI330_INT_SRC_FLAT, BMI330_INT1);

    myIMU.enableFeature(BMI330_FEAT_TILT | BMI330_FEAT_ORIENTATION | BMI330_FEAT_FLAT, true);

    pinMode(kInterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(kInterruptPin), bmi330ISR, RISING);

    Serial.println();
    Serial.println("Ready. Rotate the board, stand it on edge, lay it flat.");
    Serial.println();
}

void loop()
{
    if (!interruptFired)
        return;

    interruptFired = false;

    // One read, several possible causes - clear-on-read means testing the returned word rather
    // than reading again per feature.
    uint16_t status = 0;
    myIMU.getInterruptStatus(BMI330_INT1, status);

    if (status & BMI330_INT_STATUS_TILT)
        Serial.println("Tilt detected - the board has moved away from its reference attitude.");

    if (status & BMI330_INT_STATUS_ORIENTATION)
    {
        sfe_bmi330_orientation_t orientation;
        bool faceDown = false;
        myIMU.getOrientation(orientation, faceDown);

        Serial.print("Orientation changed: ");
        Serial.print(orientationName(orientation));
        Serial.print(", ");
        Serial.println(faceDown ? "face down" : "face up");
    }

    if (status & BMI330_INT_STATUS_FLAT)
        Serial.println("Flat - the board is lying level.");
}
