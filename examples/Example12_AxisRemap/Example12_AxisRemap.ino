/*
  Example 12 - Axis Remapping

  A sensor is rarely mounted the way the datasheet drew it. Axis remapping tells the BMI330 how
  the board actually sits, and it applies the correction INSIDE the device - so the
  accelerometer, the gyroscope, the FIFO and every feature-engine feature all agree.

  That last part is why this is worth doing in the device rather than in your sketch. Fixing
  the axes in software corrects the numbers you print and nothing else: tap detection still
  watches the physical Z axis, tilt still references the physical gravity direction, and the
  step counter still expects up to be up. One setAxisRemap() call fixes all of them at once.

  A remap is a permutation plus three sign flips. Read the permutation as "reported X, Y, Z
  come from physical ...": BMI330_AXIS_MAP_YXZ means reported X is physical Y, reported Y is
  physical X, and Z is unchanged.

  KEEP THE FRAME RIGHT-HANDED. A physical rotation always swaps an even number of signs and axes
  together. Swapping two axes on their own, or flipping a single sign, describes a MIRROR image:
  the accelerometer still looks plausible, but every gyroscope rate comes out with the wrong
  handedness. A 90 degree turn in the board's plane is a swap of X and Y PLUS one inversion.

  TWO THINGS THAT CATCH PEOPLE OUT:

    1. THE COMMIT SUSPENDS THE ACCELEROMETER. The device only accepts an axis-map update while
       the accelerometer is suspended, so setAxisRemap() saves the current power mode, disables
       the sensor, commits, and restores it. Sampling stops for a few tens of milliseconds. Do
       not call this from inside a data-ready interrupt handler, and do not expect a continuous
       sample stream across it.

    2. A remap does not survive a soft reset. softReset() returns everything to defaults, so
       re-apply it afterwards.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC

  Lay the board flat, face up, on a table. Z should read about +1 g. The sketch then flips the
  board's frame upside down in software and Z should read about -1 g without you touching it.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

void printAccel(const char *label)
{
    // Average a few samples so the comparison is not dominated by noise.
    const uint8_t kSamples = 20;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

    for (uint8_t i = 0; i < kSamples; i++)
    {
        sfe_bmi330_data_t accel;

        if (myIMU.getAccelData(accel) == ksfTkErrOk)
        {
            sx += accel.x;
            sy += accel.y;
            sz += accel.z;
        }

        delay(20);
    }

    Serial.print(label);
    Serial.print("X ");
    Serial.print(sx / kSamples, 3);
    Serial.print(" g, Y ");
    Serial.print(sy / kSamples, 3);
    Serial.print(" g, Z ");
    Serial.print(sz / kSamples, 3);
    Serial.println(" g");
}

void applyRemap(const char *description, sfe_bmi330_axis_map_t map, bool invX, bool invY, bool invZ)
{
    sfe_bmi330_axis_remap_t remap;
    remap.map = map;
    remap.invertX = invX;
    remap.invertY = invY;
    remap.invertZ = invZ;

    Serial.println();
    Serial.print("Applying: ");
    Serial.println(description);

    if (myIMU.setAxisRemap(remap) != ksfTkErrOk)
    {
        Serial.println("  Remap FAILED - the feature engine did not acknowledge the update.");
        Serial.println("  Check getFeatureEngineStatus(); the engine must be running.");
        return;
    }

    // Read it back - the device is the authority on what was actually stored.
    sfe_bmi330_axis_remap_t readBack;
    myIMU.getAxisRemap(readBack);

    Serial.print("  Stored: map ");
    Serial.print(readBack.map);
    Serial.print(", invert X/Y/Z = ");
    Serial.print(readBack.invertX);
    Serial.print("/");
    Serial.print(readBack.invertY);
    Serial.print("/");
    Serial.println(readBack.invertZ);

    delay(100);
    printAccel("  Reading:  ");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 12 - Axis Remapping");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    // Turn the accelerometer on so there is something to read. setAxisRemap() will suspend and
    // restore it around each commit.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_4G);
    delay(100);

    Serial.println();
    Serial.println("Lay the board flat and face up. Z should read about +1 g.");
    Serial.println("Starting in 3 seconds - do not move the board during the test.");
    delay(3000);

    Serial.println();
    printAccel("Default (no remap): ");

    // Board rotated 90 degrees in its own plane: reported X is physical Y, reported Y is
    // physical -X. The swap needs the inversion to stay a rotation rather than a mirror.
    applyRemap("rotated 90 degrees (X = Y, Y = -X)", BMI330_AXIS_MAP_YXZ, false, true, false);

    // Invert Y and Z: the board is mounted upside down. Z should now read about -1 g even
    // though nothing physically moved.
    applyRemap("mounted upside down (invert Y and Z)", BMI330_AXIS_MAP_XYZ, false, true, true);

    // Back to the factory frame.
    applyRemap("back to default", BMI330_AXIS_MAP_XYZ, false, false, false);

    Serial.println();
    Serial.println("Done. Remember: a remap is lost on soft reset, so re-apply it afterwards.");
    Serial.println();
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
