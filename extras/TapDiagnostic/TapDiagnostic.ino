/*
  Tap Diagnostic (bench utility, not a library example)

  A bench tool for when tap detection is not firing. Tap is the fussiest feature on the BMI330,
  and when it fails it fails silently - there is no error code for "your taps are landing on an
  axis I am not watching" or "your threshold is above your knuckles". This sketch answers those
  questions directly.

  It does three things:

    1. DUMPS THE DEVICE STATE. Feature engine status, the feature-engine error nibble, the
       enable register, the interrupt map, and the accelerometer configuration read back from
       the hardware rather than from what the sketch thinks it wrote. If the engine is not
       running or the enable did not stick, everything else is moot and you will see it here.

    2. MEASURES YOUR TAPS. It samples the accelerometer flat out and reports the peak excursion
       on each axis, so you can see how hard your taps actually are and - more importantly -
       WHICH AXIS they land on. The tap detector watches exactly one axis. A board standing in
       a carrier, or tapped on the enclosure side, puts the impulse on X or Y, and a Z-axis
       detector will never see it no matter how low the threshold goes.

    3. SWEEPS THE AXES. It arms the detector on X, then Y, then Z in turn, each with a
       deliberately low threshold, and counts gestures on each. Whichever axis counts, that is
       the axis to put in your sketch.

  RUN IT, THEN TAP STEADILY throughout - about one tap per second, in the way you intend the
  finished product to be tapped. The whole sweep takes about half a minute.

  WHAT TO DO WITH THE RESULT:

    One axis counts gestures     -> use that axis in Example07, raise the threshold until the
                                    false positives stop.
    No axis counts, but the peak -> the threshold is too high for your mounting. Lower
    excursions are large             kThresholdLow further, or tap harder.
    No axis counts, peaks tiny   -> the impulse is not reaching the sensor. Usually a board
                                    floating on a cable rather than fixed to what you are
                                    tapping. Mount it down and try again.
    Feature engine not running   -> nothing to do with tap. Check power and the bus.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC

  No interrupt wiring needed. This sketch polls, which is exactly the point - it takes the
  interrupt pin out of the equation so a wiring fault cannot be mistaken for a detector fault.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

// Deliberately low, so the sweep errs toward false positives rather than missing a real tap.
// Tune upward in your own sketch once you know the axis.
const uint16_t kThresholdLow = 0x10;

// Seconds to listen on each axis.
const uint8_t kSecondsPerAxis = 8;

// Resting acceleration, measured at startup and subtracted so gravity does not count as a peak.
float baselineX = 0.0f, baselineY = 0.0f, baselineZ = 0.0f;

// Peak excursion seen on each axis since the last report.
float peakX = 0.0f, peakY = 0.0f, peakZ = 0.0f;

void dumpState(void)
{
    Serial.println(F("--- Device state -------------------------------------------"));

    uint8_t chipId = 0, revId = 0;
    myIMU.getChipID(chipId, revId);
    Serial.print(F("Chip ID            : 0x"));
    Serial.print(chipId, HEX);
    Serial.print(F("   (expect 0x48)   revision "));
    Serial.println(revId);

    Serial.print(F("Feature engine     : "));
    Serial.println(myIMU.getFeatureEngineStatus() ? F("running") : F("*** NOT RUNNING ***"));

    uint16_t featureIO1 = 0;
    myIMU.getFeatureEngineError(featureIO1);
    Serial.print(F("FEATURE_IO1        : 0x"));
    Serial.print(featureIO1, HEX);
    Serial.print(F("   error nibble 0x"));
    Serial.print(featureIO1 & 0x000F, HEX);
    Serial.println(F("   (0x5 = no error)"));

    // Bit 7 is the feature engine's complaint that the accelerometer is running too slowly for
    // the features that are enabled. If this is set, fix the ODR before touching anything else.
    if (featureIO1 & 0x0080)
        Serial.println(F("  *** sample rate error: the accelerometer ODR is too low for an enabled feature"));

    uint16_t enabled = 0;
    myIMU.getEnabledFeatures(enabled);
    Serial.print(F("FEATURE_IO0        : 0x"));
    Serial.print(enabled, HEX);
    Serial.print(F("   tap bits "));
    Serial.print((enabled & 0x1000) ? F("S") : F("-"));
    Serial.print((enabled & 0x2000) ? F("D") : F("-"));
    Serial.println((enabled & 0x4000) ? F("T") : F("-"));

    // Read the accelerometer configuration back from the hardware. Worth doing on any sketch
    // that is behaving oddly: a reading that looks scaled wrong usually means the range in the
    // device and the range the conversion is using have diverged.
    sfe_bmi330_accel_mode_t mode;
    sfe_bmi330_odr_t odr;
    sfe_bmi330_accel_range_t range;
    sfe_bmi330_bandwidth_t bandwidth;
    sfe_bmi330_averaging_t averaging;
    myIMU.getAccelConfig(mode, odr, range, bandwidth, averaging);

    const char *rangeName = "?";
    float fullScale = 0.0f;
    switch (range)
    {
    case BMI330_ACCEL_RANGE_2G:
        rangeName = "+/-2 g";
        fullScale = 2.0f;
        break;
    case BMI330_ACCEL_RANGE_4G:
        rangeName = "+/-4 g";
        fullScale = 4.0f;
        break;
    case BMI330_ACCEL_RANGE_8G:
        rangeName = "+/-8 g";
        fullScale = 8.0f;
        break;
    case BMI330_ACCEL_RANGE_16G:
        rangeName = "+/-16 g";
        fullScale = 16.0f;
        break;
    }

    Serial.print(F("Accel range        : "));
    Serial.print(rangeName);
    Serial.print(F("   (readings above "));
    Serial.print(fullScale, 1);
    Serial.println(F(" g are impossible and mean something is wrong)"));

    Serial.print(F("Accel mode / ODR   : 0x"));
    Serial.print(mode, HEX);
    Serial.print(F(" / 0x"));
    Serial.println(odr, HEX);

    uint16_t intMap2 = 0;
    myIMU.readRegister(0x3B, intMap2);
    Serial.print(F("INT_MAP2           : 0x"));
    Serial.print(intMap2, HEX);
    Serial.print(F("   tap routed to "));
    uint8_t tapRoute = intMap2 & 0x0003;
    Serial.println(tapRoute == 0 ? F("NOWHERE - tap will not appear in any status register")
                                 : (tapRoute == 1 ? F("INT1") : F("INT2")));

    // The raw tap configuration words, straight out of feature memory.
    uint16_t tapWords[3] = {0};
    myIMU.readFeatureWords(BMI330_FEAT_ADDR_TAP, tapWords, 3);
    Serial.print(F("Tap config words   : 0x"));
    Serial.print(tapWords[0], HEX);
    Serial.print(F(" 0x"));
    Serial.print(tapWords[1], HEX);
    Serial.print(F(" 0x"));
    Serial.println(tapWords[2], HEX);

    Serial.println(F("------------------------------------------------------------"));
    Serial.println();
}

void measureBaseline(void)
{
    const uint8_t kSamples = 50;
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

        delay(10);
    }

    baselineX = sx / kSamples;
    baselineY = sy / kSamples;
    baselineZ = sz / kSamples;

    Serial.print(F("Resting orientation: X "));
    Serial.print(baselineX, 3);
    Serial.print(F(" g, Y "));
    Serial.print(baselineY, 3);
    Serial.print(F(" g, Z "));
    Serial.print(baselineZ, 3);
    Serial.println(F(" g"));
    Serial.println();
}

// Arm the tap detector on one axis and listen. Returns the number of gestures counted, and
// leaves the per-axis peak excursions in peakX / peakY / peakZ.
uint16_t listenOnAxis(sfe_bmi330_tap_axis_t axis, const char *axisName)
{
    sfe_bmi330_tap_config_t tap;
    tap.axis = axis;
    tap.mode = 0;                  // Most sensitive mode for the sweep.
    tap.peakThreshold = kThresholdLow;
    tap.maxPeaksForTap = 6;
    tap.maxGestureDuration = 0x11;
    tap.maxDurationBetweenPeaks = 5;
    tap.shockSettlingDuration = 5;
    tap.minQuietBetweenTaps = 7;
    tap.quietTimeAfterGesture = 5;
    tap.waitForTimeout = false;    // Report as soon as anything is seen - we are counting, not classifying.
    myIMU.setTapConfig(tap);

    // Re-arm. Disabling and re-enabling makes the engine pick up the new axis cleanly.
    myIMU.enableFeature(BMI330_FEAT_SINGLE_TAP | BMI330_FEAT_DOUBLE_TAP | BMI330_FEAT_TRIPLE_TAP, false);
    myIMU.enableFeature(BMI330_FEAT_SINGLE_TAP | BMI330_FEAT_DOUBLE_TAP | BMI330_FEAT_TRIPLE_TAP, true);

    // Throw away anything pending from the previous axis.
    uint16_t discard = 0;
    myIMU.getInterruptStatus(BMI330_INT1, discard);

    peakX = 0.0f;
    peakY = 0.0f;
    peakZ = 0.0f;

    Serial.print(F("Listening on "));
    Serial.print(axisName);
    Serial.print(F(" for "));
    Serial.print(kSecondsPerAxis);
    Serial.println(F(" seconds - tap now, about once a second..."));

    uint16_t gestures = 0;
    uint32_t samples = 0;
    uint32_t started = millis();

    while (millis() - started < (uint32_t)kSecondsPerAxis * 1000UL)
    {
        sfe_bmi330_data_t accel;

        if (myIMU.getAccelData(accel) == ksfTkErrOk)
        {
            samples++;

            float dx = fabs(accel.x - baselineX);
            float dy = fabs(accel.y - baselineY);
            float dz = fabs(accel.z - baselineZ);

            if (dx > peakX)
                peakX = dx;
            if (dy > peakY)
                peakY = dy;
            if (dz > peakZ)
                peakZ = dz;
        }

        uint16_t status = 0;
        myIMU.getInterruptStatus(BMI330_INT1, status);

        if (status & BMI330_INT_STATUS_TAP)
        {
            gestures++;
            Serial.print(F("  tap! ("));
            Serial.print(gestures);
            Serial.println(F(")"));
        }
    }

    Serial.print(F("  "));
    Serial.print(axisName);
    Serial.print(F(": "));
    Serial.print(gestures);
    Serial.print(F(" gesture(s) from "));
    Serial.print(samples);
    Serial.print(F(" samples. Peak excursion X "));
    Serial.print(peakX, 2);
    Serial.print(F(" g, Y "));
    Serial.print(peakY, 2);
    Serial.print(F(" g, Z "));
    Serial.print(peakZ, 2);
    Serial.println(F(" g"));
    Serial.println();

    return gestures;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println(F("SparkFun BMI330 Tap Diagnostic"));
    Serial.println();

    Wire.begin();
    Wire.setClock(400000);

    while (myIMU.begin() == false)
    {
        Serial.println(F("BMI330 not found - check your wiring!"));
        delay(1000);
    }

    // Fast and wide: 400 Hz so the impulse is not missed between samples, +/-8 g so a firm tap
    // does not clip. A clipped peak loses the shape the detector is looking for.
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_400_HZ, BMI330_ACCEL_RANGE_8G);

    // Route tap into INT_STATUS_INT1. This is required even when polling - an unmapped source
    // does not appear in either status register.
    myIMU.mapInterrupt(BMI330_INT_SRC_TAP, BMI330_INT1);

    myIMU.enableFeature(BMI330_FEAT_SINGLE_TAP | BMI330_FEAT_DOUBLE_TAP | BMI330_FEAT_TRIPLE_TAP, true);

    delay(100);
    dumpState();

    Serial.println(F("Hold still for a moment - measuring the resting orientation."));
    measureBaseline();

    Serial.println(F("Now tap steadily, about once a second, the way you intend to use it."));
    Serial.println();
    delay(1000);

    uint16_t onX = listenOnAxis(BMI330_TAP_AXIS_X, "X");
    uint16_t onY = listenOnAxis(BMI330_TAP_AXIS_Y, "Y");
    uint16_t onZ = listenOnAxis(BMI330_TAP_AXIS_Z, "Z");

    Serial.println(F("=== Result ================================================="));
    Serial.print(F("Gestures counted: X "));
    Serial.print(onX);
    Serial.print(F(", Y "));
    Serial.print(onY);
    Serial.print(F(", Z "));
    Serial.println(onZ);

    uint16_t best = onX;
    const char *bestName = "X";
    sfe_bmi330_tap_axis_t bestAxis = BMI330_TAP_AXIS_X;

    if (onY > best)
    {
        best = onY;
        bestName = "Y";
        bestAxis = BMI330_TAP_AXIS_Y;
    }

    if (onZ > best)
    {
        best = onZ;
        bestName = "Z";
        bestAxis = BMI330_TAP_AXIS_Z;
    }

    if (best == 0)
    {
        Serial.println(F("No gestures on any axis."));
        Serial.println(F("If the peak excursions above were well under 1 g, the impulse is not"));
        Serial.println(F("reaching the sensor - fix the board to whatever you are tapping and retry."));
        Serial.println(F("If they were large, lower kThresholdLow and run again."));
    }
    else
    {
        Serial.print(F("Use BMI330_TAP_AXIS_"));
        Serial.print(bestName);
        Serial.println(F(" in Example07, then raise peakThreshold until false taps stop."));

        // Leave the detector armed on the winning axis so loop() is immediately useful.
        sfe_bmi330_tap_config_t tap;
        tap.axis = bestAxis;
        tap.peakThreshold = kThresholdLow;
        myIMU.setTapConfig(tap);
    }

    Serial.println(F("============================================================"));
    Serial.println();
    Serial.println(F("Continuing to listen on the best axis. Ctrl-R to run the sweep again."));
}

void loop()
{
    uint16_t status = 0;
    myIMU.getInterruptStatus(BMI330_INT1, status);

    if (status & BMI330_INT_STATUS_TAP)
    {
        sfe_bmi330_tap_event_t event;
        myIMU.getTapEvent(event);

        Serial.print(F("tap: "));
        Serial.print(event.singleTap ? F("single ") : F(""));
        Serial.print(event.doubleTap ? F("double ") : F(""));
        Serial.print(event.tripleTap ? F("triple ") : F(""));
        Serial.println();
    }

    delay(5);
}
