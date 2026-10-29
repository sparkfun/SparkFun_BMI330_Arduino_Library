/*
  Example 03 - Configuration and Filtering

  Walks through the BMI330's configuration knobs and shows what each one costs and buys. The
  sketch cycles through several settings, printing the noise it measures at each, so you can
  watch the trade-offs on a bench rather than take them on faith.

  The knobs, and what they do:

    Range       16-bit output always spans the selected range, so sensitivity halves each time
                the range doubles: 16384 LSB/g at +/-2 g, 2048 LSB/g at +/-16 g. Narrower is
                better resolution - until something clips, at which point it is much worse.

    ODR         Output data rate. Higher rates cost current and admit more noise per sample.

    Bandwidth   Low-pass cutoff, expressed as ODR/2 or ODR/4. ODR/4 is quieter and adds group
                delay, which matters if you are closing a control loop around the output.

    Averaging   Averages N samples in hardware. Noise falls as sqrt(N) - 64x averaging is 8x
                quieter - and the output data rate does NOT change, so it costs the host
                nothing. It does raise the sensor's internal sample rate, and with it current.

  An illegal ODR-and-power-mode pairing (say 6400 Hz in low-power mode) does not make the
  register write fail. It sets acc_conf_err or gyr_conf_err in ERR_REG, which this sketch
  checks after every change. That check is worth keeping in your own code.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  QWIIC --> QWIIC

  Leave the board STILL on a table while this runs - it is measuring noise, and a nudge will
  swamp the numbers it is trying to show you.

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

SfeBMI330ArdI2C myIMU;

// Measure the standard deviation of the accelerometer Z axis, in milli-g, over a batch of
// samples. This is the number that averaging and bandwidth actually move.
float measureNoise_mg(uint16_t samples)
{
    float sum = 0.0f;
    float sumSq = 0.0f;

    for (uint16_t i = 0; i < samples; i++)
    {
        sfe_bmi330_data_t accel;

        if (myIMU.getAccelData(accel) != ksfTkErrOk)
            return -1.0f;

        sum += accel.z;
        sumSq += accel.z * accel.z;
        delay(5);
    }

    float mean = sum / samples;
    float variance = (sumSq / samples) - (mean * mean);

    if (variance < 0.0f)
        variance = 0.0f;

    return sqrt(variance) * 1000.0f;
}

void reportConfig(const char *label, sfe_bmi330_accel_range_t range, sfe_bmi330_odr_t odr,
                  sfe_bmi330_bandwidth_t bandwidth, sfe_bmi330_averaging_t averaging)
{
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, odr, range, bandwidth, averaging);

    // Let the filter settle before measuring - the first samples after a configuration change
    // still carry the old settings' history.
    delay(200);

    sfe_bmi330_error_status_t errors;
    myIMU.getErrorStatus(errors);

    Serial.print(label);
    Serial.print("\t");

    if (errors.accelConfError)
    {
        Serial.println("REJECTED by the device (illegal ODR / power mode pairing)");
        return;
    }

    Serial.print(measureNoise_mg(100), 2);
    Serial.println(" mg RMS");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 3 - Configuration and Filtering");

    Wire.begin();

    while (myIMU.begin() == false)
    {
        Serial.println("BMI330 not found - check your wiring!");
        delay(1000);
    }

    Serial.println();
    Serial.println("Measuring accelerometer Z-axis noise. Keep the board still.");
    Serial.println();
    Serial.println("Configuration\t\t\tNoise");
    Serial.println("-------------------------------------------");

    // Range: wider range, coarser LSB, more quantization noise.
    reportConfig("+/-2 g,  100 Hz, no avg  ", BMI330_ACCEL_RANGE_2G, BMI330_ODR_100_HZ, BMI330_BW_ODR_HALF,
                 BMI330_AVG_1);
    reportConfig("+/-16 g, 100 Hz, no avg  ", BMI330_ACCEL_RANGE_16G, BMI330_ODR_100_HZ, BMI330_BW_ODR_HALF,
                 BMI330_AVG_1);

    // ODR: higher rate, wider noise bandwidth per sample.
    reportConfig("+/-2 g,  800 Hz, no avg  ", BMI330_ACCEL_RANGE_2G, BMI330_ODR_800_HZ, BMI330_BW_ODR_HALF,
                 BMI330_AVG_1);

    // Bandwidth: ODR/4 rolls off earlier than ODR/2.
    reportConfig("+/-2 g,  800 Hz, BW ODR/4", BMI330_ACCEL_RANGE_2G, BMI330_ODR_800_HZ,
                 BMI330_BW_ODR_QUARTER, BMI330_AVG_1);

    // Averaging: the cheapest noise reduction available, if you can spare the current.
    reportConfig("+/-2 g,  800 Hz, avg 8   ", BMI330_ACCEL_RANGE_2G, BMI330_ODR_800_HZ, BMI330_BW_ODR_HALF,
                 BMI330_AVG_8);
    reportConfig("+/-2 g,  800 Hz, avg 64  ", BMI330_ACCEL_RANGE_2G, BMI330_ODR_800_HZ, BMI330_BW_ODR_HALF,
                 BMI330_AVG_64);

    // And an illegal pairing, to show what the error flags are for: 6400 Hz is a
    // high-performance-only rate, so asking for it in low-power mode is refused.
    Serial.println();
    Serial.println("Deliberately asking for 6400 Hz in low-power mode:");
    myIMU.setAccelConfig(BMI330_ACCEL_MODE_LOW_POWER, BMI330_ODR_6400_HZ, BMI330_ACCEL_RANGE_2G);
    delay(50);

    sfe_bmi330_error_status_t errors;
    myIMU.getErrorStatus(errors);
    Serial.print("  accel configuration error flag: ");
    Serial.println(errors.accelConfError ? "SET, as expected" : "clear");

    Serial.println();
    Serial.println("Done. Reading the current configuration back from the device:");

    sfe_bmi330_accel_mode_t mode;
    sfe_bmi330_odr_t odr;
    sfe_bmi330_accel_range_t range;
    sfe_bmi330_bandwidth_t bandwidth;
    sfe_bmi330_averaging_t averaging;

    myIMU.getAccelConfig(mode, odr, range, bandwidth, averaging);

    Serial.print("  mode 0x");
    Serial.print(mode, HEX);
    Serial.print(", odr 0x");
    Serial.print(odr, HEX);
    Serial.print(", range 0x");
    Serial.print(range, HEX);
    Serial.print(", bandwidth ");
    Serial.print(bandwidth);
    Serial.print(", averaging ");
    Serial.println(averaging);
}

void loop()
{
    delay(1000);
}
