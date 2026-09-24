/**
 * @file sfDevBMI330.h
 * @brief Header file for the SparkFun Bosch BMI330 6-DoF IMU Driver.
 *
 * @details
 * sfDevBMI330 is a comms-agnostic driver for the Bosch BMI330 inertial measurement unit, built on
 * the SparkFun Toolkit. The BMI330 pairs a 16-bit triaxial accelerometer with a 16-bit triaxial
 * gyroscope, a temperature sensor, a 2 kB FIFO, two interrupt pins, and an on-chip feature engine
 * that detects motion events without host involvement.
 *
 * @note NO CONFIGURATION FILE. Unlike the BMI270 - which will not report a single sample until the
 * host uploads an 8 kB firmware blob and waits for it to boot - every BMI330 feature lives in ROM.
 * begin() is a chip ID check and a feature-engine enable, nothing more, and the library carries no
 * vendor binary. That is why this driver talks to the register map directly rather than wrapping
 * Bosch's SensorAPI: there is nothing the vendor code is uniquely required for, and going native
 * buys I2C and SPI from one code path through sfTkIBus.
 *
 * @note SIXTEEN-BIT REGISTERS, AND DUMMY BYTES. Every BMI330 register is a 16-bit word,
 * little-endian, and the register address auto-increments by one WORD per two bytes transferred. A
 * read burst is therefore always an even number of bytes. More surprising: the device prepends
 * DUMMY BYTES to the payload of every read - two on I2C, one on SPI - so a driver that forgets
 * them reads garbage shifted by one byte. The count is settled once, in begin(), from
 * sfTkIBus::type(), and readRegisters() skips them. See kDummyBytesI2C / kDummyBytesSPI.
 *
 * @note THE FEATURE ENGINE IS A SEPARATE ADDRESS SPACE. Any-motion, tap, step counting, self-test
 * and the rest are not configured through the main register map. They live in extended memory
 * reached through a two-register window: write a base address to FEATURE_DATA_ADDR (0x41), then
 * read or write consecutive words at FEATURE_DATA_TX (0x42), which auto-increments. Enabling a
 * feature is a third step again - a bit in FEATURE_IO0 (0x10), committed by writing 1 to
 * FEATURE_IO_STATUS (0x14). readFeatureWords() / writeFeatureWords() / enableFeature() wrap all of
 * this; the ::sfe_bmi330_feature_addr_t values are the window's base addresses.
 *
 * @note The feature engine must be running before any feature - including self-test and
 * self-calibration - will respond. It starts during softReset(), which begin() calls, and
 * getFeatureEngineStatus() reports whether it came up.
 *
 * @author SparkFun Electronics
 * @date September 2026
 * @copyright Copyright (c) 2026, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_BMI330_Arduino_Library
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

// SparkFun Toolkit core headers
#include <sfTk/sfTkIBus.h>
#include <sfTk/sfTkII2C.h>
#include <sfTk/sfTkISPI.h>
#include <sfTk/sfToolkit.h>

///////////////////////////////////////////////////////////////////////////////
// Device Identity
///////////////////////////////////////////////////////////////////////////////

/** @brief Factory-default 7-bit I2C address (SDO / ADR pin low). SparkFun ships the Qwiic board this way. */
#define SFE_BMI330_DEFAULT_ADDRESS 0x68

/** @brief Alternate 7-bit I2C address, selected by the board's ADR jumper (SDO / ADR pin high). */
#define SFE_BMI330_ALT_ADDRESS 0x69

/** @brief Value in the low byte of CHIP_ID (0x00) for a BMI330. The BMI323 reads 0x43 here. */
#define SFE_BMI330_CHIP_ID 0x48

///////////////////////////////////////////////////////////////////////////////
// Output Data Rate
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Output data rate settings, shared by the accelerometer and the gyroscope.
 *
 * @details Each sensor keeps its own ODR field, so the two may run at different rates. Not every
 * rate is legal in every power mode: low-power mode tops out at 400 Hz, and rates below 12.5 Hz are
 * only available in low-power mode. An illegal pairing is not rejected by the write - it raises
 * acc_conf_err / gyr_conf_err in ERR_REG, which getErrorStatus() reports.
 *
 * @see setAccelODR(), setGyroODR(), getErrorStatus()
 */
typedef enum sfe_bmi330_odr_t : uint8_t
{
    BMI330_ODR_0_78_HZ = 0x01, ///< 0.78125 Hz. Low-power mode only.
    BMI330_ODR_1_56_HZ = 0x02, ///< 1.5625 Hz. Low-power mode only.
    BMI330_ODR_3_12_HZ = 0x03, ///< 3.125 Hz. Low-power mode only.
    BMI330_ODR_6_25_HZ = 0x04, ///< 6.25 Hz. Low-power mode only.
    BMI330_ODR_12_5_HZ = 0x05, ///< 12.5 Hz.
    BMI330_ODR_25_HZ = 0x06,   ///< 25 Hz.
    BMI330_ODR_50_HZ = 0x07,   ///< 50 Hz. Required by several feature-engine features.
    BMI330_ODR_100_HZ = 0x08,  ///< 100 Hz. The self-test and self-calibration rate.
    BMI330_ODR_200_HZ = 0x09,  ///< 200 Hz.
    BMI330_ODR_400_HZ = 0x0A,  ///< 400 Hz. Highest rate available in low-power mode.
    BMI330_ODR_800_HZ = 0x0B,  ///< 800 Hz. Normal or high-performance mode only.
    BMI330_ODR_1600_HZ = 0x0C, ///< 1600 Hz. Normal or high-performance mode only.
    BMI330_ODR_3200_HZ = 0x0D, ///< 3200 Hz. High-performance mode only.
    BMI330_ODR_6400_HZ = 0x0E  ///< 6400 Hz. High-performance mode only, accelerometer-friendly.
} sfe_bmi330_odr_t;

///////////////////////////////////////////////////////////////////////////////
// Measurement Ranges
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Accelerometer full-scale range.
 *
 * @details The output is always 16-bit signed across the selected range, so sensitivity halves each
 * time the range doubles: 16384 LSB/g at +/-2 g down to 2048 LSB/g at +/-16 g. Picking the smallest
 * range the application can survive is free resolution.
 */
typedef enum sfe_bmi330_accel_range_t : uint8_t
{
    BMI330_ACCEL_RANGE_2G = 0x00,  ///< +/-2 g, 16384 LSB/g.
    BMI330_ACCEL_RANGE_4G = 0x01,  ///< +/-4 g, 8192 LSB/g.
    BMI330_ACCEL_RANGE_8G = 0x02,  ///< +/-8 g, 4096 LSB/g. Required for gyro self-calibration.
    BMI330_ACCEL_RANGE_16G = 0x03  ///< +/-16 g, 2048 LSB/g.
} sfe_bmi330_accel_range_t;

/**
 * @brief Gyroscope full-scale range.
 *
 * @details As with the accelerometer, the 16-bit output spans the selected range: 262.144 LSB/dps
 * at +/-125 dps down to 16.384 LSB/dps at +/-2000 dps.
 */
typedef enum sfe_bmi330_gyro_range_t : uint8_t
{
    BMI330_GYRO_RANGE_125DPS = 0x00,  ///< +/-125 dps, 262.144 LSB/dps.
    BMI330_GYRO_RANGE_250DPS = 0x01,  ///< +/-250 dps, 131.072 LSB/dps.
    BMI330_GYRO_RANGE_500DPS = 0x02,  ///< +/-500 dps, 65.536 LSB/dps.
    BMI330_GYRO_RANGE_1000DPS = 0x03, ///< +/-1000 dps, 32.768 LSB/dps.
    BMI330_GYRO_RANGE_2000DPS = 0x04  ///< +/-2000 dps, 16.384 LSB/dps.
} sfe_bmi330_gyro_range_t;

///////////////////////////////////////////////////////////////////////////////
// Filtering
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief 3 dB cutoff of the sensor's low-pass filter, expressed as a fraction of the ODR.
 */
typedef enum sfe_bmi330_bandwidth_t : uint8_t
{
    BMI330_BW_ODR_HALF = 0x00,   ///< Cutoff at ODR / 2.
    BMI330_BW_ODR_QUARTER = 0x01 ///< Cutoff at ODR / 4. Quieter, more group delay.
} sfe_bmi330_bandwidth_t;

/**
 * @brief Number of samples averaged before a result is published.
 *
 * @details Hardware averaging trades bandwidth and current for noise: averaging N samples drops
 * broadband noise by sqrt(N). It does NOT change the output data rate - the device oversamples
 * internally - so it is free bandwidth from the host's point of view, but it does raise the
 * sensor's own sample rate and therefore its current draw.
 */
typedef enum sfe_bmi330_averaging_t : uint8_t
{
    BMI330_AVG_1 = 0x00,  ///< No averaging.
    BMI330_AVG_2 = 0x01,  ///< Average 2 samples.
    BMI330_AVG_4 = 0x02,  ///< Average 4 samples.
    BMI330_AVG_8 = 0x03,  ///< Average 8 samples.
    BMI330_AVG_16 = 0x04, ///< Average 16 samples.
    BMI330_AVG_32 = 0x05, ///< Average 32 samples.
    BMI330_AVG_64 = 0x06  ///< Average 64 samples.
} sfe_bmi330_averaging_t;

///////////////////////////////////////////////////////////////////////////////
// Power Modes
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Accelerometer power mode.
 *
 * @details Both sensors are DISABLED at power-on - a fresh BMI330 produces no data until a mode is
 * set. enableAccel() picks high-performance mode, which is what most sketches want.
 */
typedef enum sfe_bmi330_accel_mode_t : uint8_t
{
    BMI330_ACCEL_MODE_DISABLE = 0x00,  ///< Accelerometer off. Power-on default.
    BMI330_ACCEL_MODE_LOW_POWER = 0x03,///< Duty-cycled. ODR limited to 400 Hz and below.
    BMI330_ACCEL_MODE_NORMAL = 0x04,   ///< Continuous, reduced current. ODR up to 1600 Hz.
    BMI330_ACCEL_MODE_HIGH_PERF = 0x07 ///< Continuous, lowest noise. All ODRs available.
} sfe_bmi330_accel_mode_t;

/**
 * @brief Gyroscope power mode.
 *
 * @note SUSPEND versus DISABLE. In DISABLE the drive circuit is off and the gyro needs its full
 * start-up time (tens of milliseconds) to produce valid data again. SUSPEND keeps the drive running
 * and the ADC off, so waking is fast, at the cost of standing current. Choose SUSPEND for something
 * that must respond to an interrupt promptly; choose DISABLE for something that sleeps for seconds.
 */
typedef enum sfe_bmi330_gyro_mode_t : uint8_t
{
    BMI330_GYRO_MODE_DISABLE = 0x00,   ///< Gyroscope off, drive off. Power-on default.
    BMI330_GYRO_MODE_SUSPEND = 0x01,   ///< Drive running, ADC off. Fast wake, higher idle current.
    BMI330_GYRO_MODE_LOW_POWER = 0x03, ///< Duty-cycled. ODR limited to 400 Hz and below.
    BMI330_GYRO_MODE_NORMAL = 0x04,    ///< Continuous, reduced current. ODR up to 1600 Hz.
    BMI330_GYRO_MODE_HIGH_PERF = 0x07  ///< Continuous, lowest noise. All ODRs available.
} sfe_bmi330_gyro_mode_t;

///////////////////////////////////////////////////////////////////////////////
// Interrupts
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Interrupt routing target.
 *
 * @details Every interrupt source is routed independently. The SparkFun Qwiic BMI330 breakout
 * brings both pins out, so a sketch can (for example) put data-ready on INT1 and motion events on
 * INT2 and tell them apart without reading a status register.
 */
typedef enum sfe_bmi330_int_pin_t : uint8_t
{
    BMI330_INT_NONE = 0x00, ///< Source disabled - not routed to a pin.
    BMI330_INT1 = 0x01,     ///< Route to the INT1 pin.
    BMI330_INT2 = 0x02      ///< Route to the INT2 pin.
} sfe_bmi330_int_pin_t;

/**
 * @brief Interrupt sources that can be routed to a pin.
 *
 * @details These are the routable sources, as laid out across INT_MAP1 (0x3A) and INT_MAP2 (0x3B).
 * mapInterrupt() hides which map register a source lives in.
 */
typedef enum sfe_bmi330_int_source_t : uint8_t
{
    BMI330_INT_SRC_NO_MOTION = 0,      ///< No-motion detected.
    BMI330_INT_SRC_ANY_MOTION,         ///< Any-motion detected.
    BMI330_INT_SRC_FLAT,               ///< Flat position detected.
    BMI330_INT_SRC_ORIENTATION,        ///< Orientation changed.
    BMI330_INT_SRC_STEP_DETECTOR,      ///< A single step was detected.
    BMI330_INT_SRC_STEP_COUNTER,       ///< Step counter reached its watermark.
    BMI330_INT_SRC_SIG_MOTION,         ///< Significant motion detected.
    BMI330_INT_SRC_TILT,               ///< Tilt detected.
    BMI330_INT_SRC_TAP,                ///< Single, double or triple tap detected.
    BMI330_INT_SRC_I3C,                ///< I3C sync event.
    BMI330_INT_SRC_ERR_STATUS,         ///< Feature engine error or status change.
    BMI330_INT_SRC_TEMP_DRDY,          ///< Temperature data ready.
    BMI330_INT_SRC_GYRO_DRDY,          ///< Gyroscope data ready.
    BMI330_INT_SRC_ACCEL_DRDY,         ///< Accelerometer data ready.
    BMI330_INT_SRC_FIFO_WATERMARK,     ///< FIFO reached its watermark.
    BMI330_INT_SRC_FIFO_FULL,          ///< FIFO full.
    BMI330_INT_SRC_COUNT               ///< Number of sources. Not a valid source.
} sfe_bmi330_int_source_t;

/**
 * @brief Bit masks for the INT_STATUS_INT1 (0x0D) and INT_STATUS_INT2 (0x0E) registers.
 *
 * @details Both status registers are CLEAR-ON-READ, and getInterruptStatus() reads the whole word,
 * so a sketch must read once and test every bit it cares about in the returned value. Reading twice
 * loses the first read's events.
 */
typedef enum sfe_bmi330_int_status_t : uint16_t
{
    BMI330_INT_STATUS_NO_MOTION = 0x0001,      ///< No-motion detected.
    BMI330_INT_STATUS_ANY_MOTION = 0x0002,     ///< Any-motion detected.
    BMI330_INT_STATUS_FLAT = 0x0004,           ///< Flat position detected.
    BMI330_INT_STATUS_ORIENTATION = 0x0008,    ///< Orientation changed.
    BMI330_INT_STATUS_STEP_DETECTOR = 0x0010,  ///< A step was detected.
    BMI330_INT_STATUS_STEP_COUNTER = 0x0020,   ///< Step counter watermark reached.
    BMI330_INT_STATUS_SIG_MOTION = 0x0040,     ///< Significant motion detected.
    BMI330_INT_STATUS_TILT = 0x0080,           ///< Tilt detected.
    BMI330_INT_STATUS_TAP = 0x0100,            ///< Tap detected - see getTapEvent() for which kind.
    BMI330_INT_STATUS_I3C = 0x0200,            ///< I3C sync event.
    BMI330_INT_STATUS_ERROR = 0x0400,          ///< Feature engine error or status change.
    BMI330_INT_STATUS_TEMP_DRDY = 0x0800,      ///< Temperature data ready.
    BMI330_INT_STATUS_GYRO_DRDY = 0x1000,      ///< Gyroscope data ready.
    BMI330_INT_STATUS_ACCEL_DRDY = 0x2000,     ///< Accelerometer data ready.
    BMI330_INT_STATUS_FIFO_WATERMARK = 0x4000, ///< FIFO watermark reached.
    BMI330_INT_STATUS_FIFO_FULL = 0x8000       ///< FIFO full.
} sfe_bmi330_int_status_t;

/**
 * @brief Electrical configuration of an interrupt pin.
 *
 * @details Defaults match the SparkFun Qwiic BMI330 breakout: push-pull, active high, pulsed. Use
 * open-drain with @p activeHigh false when several devices share one interrupt line into the host -
 * the board has no interrupt pull-up fitted, so supply one externally if you do.
 */
struct sfe_bmi330_int_pin_config_t
{
    bool enabled = true;     ///< Drive the pin at all. False leaves it high-impedance.
    bool activeHigh = true;  ///< true: idle low, assert high. false: idle high, assert low.
    bool openDrain = false;  ///< true: open-drain output (needs a pull-up). false: push-pull.
};

///////////////////////////////////////////////////////////////////////////////
// Sensor Data
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief One triaxial reading in raw 16-bit signed counts, as the registers hold it.
 *
 * @details Convert with accelRawToG() / gyroRawToDps(), or let getSensorData() do it. Keeping the
 * raw form available matters for anything logging at rate: three int16_t are six bytes, where three
 * floats are twelve, and the conversion can happen off the critical path.
 *
 * @note A value pinned at -32768 or 32767 means the reading saturated the selected range, not that
 * the motion really was that large. getSaturationFlags() reports which axes clipped.
 */
struct sfe_bmi330_raw_data_t
{
    int16_t x = 0; ///< X axis, raw counts.
    int16_t y = 0; ///< Y axis, raw counts.
    int16_t z = 0; ///< Z axis, raw counts.
};

/**
 * @brief One triaxial reading converted to physical units - g for the accelerometer, dps for the
 * gyroscope.
 */
struct sfe_bmi330_data_t
{
    float x = 0.0f; ///< X axis, g or dps.
    float y = 0.0f; ///< Y axis, g or dps.
    float z = 0.0f; ///< Z axis, g or dps.
};

/**
 * @brief Everything one burst read of the data block produces.
 *
 * @details getSensorData() fills this from a single 9-word read of ACC_DATA_X (0x03) through
 * SENSOR_TIME_1 (0x0B). One transaction, so the accelerometer, gyroscope, temperature and timestamp
 * are all from the same instant - which is the whole point of reading them together rather than
 * axis by axis.
 */
struct sfe_bmi330_sensor_data_t
{
    sfe_bmi330_data_t accel;        ///< Acceleration in g.
    sfe_bmi330_data_t gyro;         ///< Angular rate in degrees per second.
    sfe_bmi330_raw_data_t accelRaw; ///< Acceleration in raw counts.
    sfe_bmi330_raw_data_t gyroRaw;  ///< Angular rate in raw counts.
    float temperature = 0.0f;       ///< Die temperature in degrees Celsius.
    uint32_t sensorTime = 0;        ///< Sensor timestamp, 39.0625 us per LSB.
};

/**
 * @brief Per-axis saturation flags from SAT_FLAGS (0x0C).
 *
 * @details A set flag means that axis hit the end of its full-scale range during the last
 * conversion, so the reported value is a floor or a ceiling rather than a measurement. The usual
 * fix is a wider range; the usual mistake is to average the clipped samples in anyway.
 */
struct sfe_bmi330_saturation_t
{
    bool accelX = false; ///< Accelerometer X saturated.
    bool accelY = false; ///< Accelerometer Y saturated.
    bool accelZ = false; ///< Accelerometer Z saturated.
    bool gyroX = false;  ///< Gyroscope X saturated.
    bool gyroY = false;  ///< Gyroscope Y saturated.
    bool gyroZ = false;  ///< Gyroscope Z saturated.
};

/**
 * @brief Decoded contents of ERR_REG (0x01).
 *
 * @note accelConfError / gyroConfError are how an illegal ODR-and-power-mode pairing surfaces. The
 * register write that caused it returned success; only this register says the device refused the
 * combination. Anything unexpected about a sensor's data rate is worth one call here.
 */
struct sfe_bmi330_error_status_t
{
    bool fatalError = false;     ///< Chip not operational. Cleared only by reset or power cycle.
    bool featEngOverload = false;///< Feature engine overloaded. Clear-on-read.
    bool featEngWatchdog = false;///< Feature engine watchdog fired. Clear-on-read.
    bool accelConfError = false; ///< Unsupported accelerometer configuration. Clears when corrected.
    bool gyroConfError = false;  ///< Unsupported gyroscope configuration. Clears when corrected.
};

///////////////////////////////////////////////////////////////////////////////
// FIFO
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Which data sources the FIFO records, as bits of FIFO_CONF (0x36).
 *
 * @details Combine with bitwise OR. The BMI330 FIFO is HEADERLESS: frames carry no tag, and the
 * order inside a frame is fixed by the hardware - accelerometer (3 words), gyroscope (3 words),
 * temperature (1 word), sensor time (1 word) - with disabled sources simply absent. The parser has
 * to be told the same enable set that was programmed, which is why readFifoData() takes it as an
 * argument rather than guessing.
 */
typedef enum sfe_bmi330_fifo_source_t : uint16_t
{
    BMI330_FIFO_TIME = 0x0100,  ///< Include the sensor timestamp (1 word per frame).
    BMI330_FIFO_ACCEL = 0x0200, ///< Include accelerometer data (3 words per frame).
    BMI330_FIFO_GYRO = 0x0400,  ///< Include gyroscope data (3 words per frame).
    BMI330_FIFO_TEMP = 0x0800,  ///< Include temperature (1 word per frame).
    BMI330_FIFO_ALL = 0x0F00    ///< All four sources (8 words per frame).
} sfe_bmi330_fifo_source_t;

/**
 * @brief One decoded FIFO frame.
 *
 * @details Fields belonging to sources that were not enabled are left at their defaults, and the
 * matching @p has... flag stays false. A frame whose sensor was idle when it was captured carries
 * the invalid pattern for that source, which readFifoData() reports as a false @p has... flag
 * rather than silently handing back 0x7F01 as a reading.
 */
struct sfe_bmi330_fifo_frame_t
{
    sfe_bmi330_raw_data_t accel; ///< Accelerometer sample, raw counts.
    sfe_bmi330_raw_data_t gyro;  ///< Gyroscope sample, raw counts.
    int16_t temperature = 0;     ///< Temperature sample, raw counts. See rawToCelsius().
    uint16_t sensorTime = 0;     ///< Low word of the sensor timestamp at capture.
    bool hasAccel = false;       ///< Accelerometer data present and valid.
    bool hasGyro = false;        ///< Gyroscope data present and valid.
    bool hasTemp = false;        ///< Temperature data present and valid.
    bool hasTime = false;        ///< Timestamp present.
};

///////////////////////////////////////////////////////////////////////////////
// Feature Engine
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Base addresses inside the feature engine's extended memory.
 *
 * @details These index the FEATURE_DATA_ADDR (0x41) / FEATURE_DATA_TX (0x42) window, not the main
 * register map - readFeatureWords() and writeFeatureWords() take one of these. The spacing between
 * consecutive entries is each feature's size in words.
 */
typedef enum sfe_bmi330_feature_addr_t : uint8_t
{
    BMI330_FEAT_ADDR_CONFIG_VERSION = 0x00, ///< Feature engine configuration version (1 word).
    BMI330_FEAT_ADDR_AXIS_REMAP = 0x03,     ///< Axis remap (2 words).
    BMI330_FEAT_ADDR_ANY_MOTION = 0x05,     ///< Any-motion configuration (3 words).
    BMI330_FEAT_ADDR_NO_MOTION = 0x08,      ///< No-motion configuration (3 words).
    BMI330_FEAT_ADDR_FLAT = 0x0B,           ///< Flat configuration (2 words).
    BMI330_FEAT_ADDR_SIG_MOTION = 0x0D,     ///< Significant motion configuration (3 words).
    BMI330_FEAT_ADDR_STEP_COUNTER = 0x10,   ///< Step counter configuration (12 words).
    BMI330_FEAT_ADDR_ORIENTATION = 0x1C,    ///< Orientation configuration (2 words).
    BMI330_FEAT_ADDR_TAP = 0x1E,            ///< Tap detector configuration (3 words).
    BMI330_FEAT_ADDR_TILT = 0x21,           ///< Tilt configuration (2 words).
    BMI330_FEAT_ADDR_ST_RESULT = 0x24,      ///< Self-test per-axis results (1 word).
    BMI330_FEAT_ADDR_ST_SELECT = 0x25,      ///< Self-test selection (1 word).
    BMI330_FEAT_ADDR_SC_SELECT = 0x26,      ///< Gyro self-calibration selection (1 word).
    BMI330_FEAT_ADDR_SC_ST_CONF = 0x27,     ///< Self-calibration / self-test configuration (1 word).
    BMI330_FEAT_ADDR_SC_ST_COEFF = 0x28     ///< Gyro filter coefficients (9 words). See performSelfTest().
    // User offsets are NOT in feature memory - they are ordinary registers (ACC_DP_OFF_X 0x60,
    // GYR_DP_OFF_X 0x66). Use setAccelOffset() / setGyroOffset().
} sfe_bmi330_feature_addr_t;

/**
 * @brief Feature enable bits in FEATURE_IO0 (0x10).
 *
 * @details Combine with bitwise OR and pass to enableFeature(). Note that any-motion and no-motion
 * are enabled PER AXIS - the feature runs when at least one of its axis bits is set - while the
 * others are a single bit each.
 */
typedef enum sfe_bmi330_feature_t : uint16_t
{
    BMI330_FEAT_NO_MOTION_X = 0x0001,  ///< No-motion on the X axis.
    BMI330_FEAT_NO_MOTION_Y = 0x0002,  ///< No-motion on the Y axis.
    BMI330_FEAT_NO_MOTION_Z = 0x0004,  ///< No-motion on the Z axis.
    BMI330_FEAT_NO_MOTION = 0x0007,    ///< No-motion on all three axes.
    BMI330_FEAT_ANY_MOTION_X = 0x0008, ///< Any-motion on the X axis.
    BMI330_FEAT_ANY_MOTION_Y = 0x0010, ///< Any-motion on the Y axis.
    BMI330_FEAT_ANY_MOTION_Z = 0x0020, ///< Any-motion on the Z axis.
    BMI330_FEAT_ANY_MOTION = 0x0038,   ///< Any-motion on all three axes.
    BMI330_FEAT_FLAT = 0x0040,         ///< Flat detection.
    BMI330_FEAT_ORIENTATION = 0x0080,  ///< Orientation detection.
    BMI330_FEAT_STEP_DETECTOR = 0x0100,///< Step detector (one interrupt per step).
    BMI330_FEAT_STEP_COUNTER = 0x0200, ///< Step counter (counts, interrupts at the watermark).
    BMI330_FEAT_SIG_MOTION = 0x0400,   ///< Significant motion detection.
    BMI330_FEAT_TILT = 0x0800,         ///< Tilt detection.
    BMI330_FEAT_SINGLE_TAP = 0x1000,   ///< Single tap detection.
    BMI330_FEAT_DOUBLE_TAP = 0x2000,   ///< Double tap detection.
    BMI330_FEAT_TRIPLE_TAP = 0x4000    ///< Triple tap detection.
} sfe_bmi330_feature_t;

/**
 * @brief Any-motion and no-motion detector settings.
 *
 * @details The two detectors are mirror images and share this structure. Any-motion fires when the
 * slope of the acceleration signal stays ABOVE @p slopeThreshold for @p duration consecutive
 * samples; no-motion fires when it stays BELOW.
 *
 * @note Units are the hardware's, not SI: slope threshold and hysteresis are 1.953 mg per LSB
 * (1/512 g), and duration and wait time are 20 ms per LSB (a 50 Hz internal tick). The defaults
 * here are the device's own power-on values.
 */
struct sfe_bmi330_motion_config_t
{
    uint16_t slopeThreshold = 10; ///< Slope threshold, 1.953 mg/LSB. Range 0 to 4095.
    bool accelRefUpdate = true;   ///< Continuously update the acceleration reference.
    uint16_t hysteresis = 2;      ///< Slope hysteresis, 1.953 mg/LSB. Range 0 to 1023.
    uint16_t duration = 10;       ///< Samples the condition must hold, 20 ms/LSB. Range 0 to 8191.
    uint8_t waitTime = 3;         ///< Delay before clearing the event, 20 ms/LSB. Range 0 to 7.
};

/**
 * @brief Significant motion detector settings.
 *
 * @details Significant motion looks for sustained, human-scale movement - walking across a room
 * rather than a door slamming nearby - by measuring the peak-to-peak acceleration magnitude and the
 * mean-crossing rate over a block of samples, and requiring both to fall inside a window. Defaults
 * are the device's power-on values.
 */
struct sfe_bmi330_sig_motion_config_t
{
    uint16_t blockSize = 250;    ///< Samples evaluated per decision, 20 ms/LSB.
    uint16_t peakToPeakMin = 30; ///< Minimum peak-to-peak magnitude, 1.953 mg/LSB. Range 0 to 1023.
    uint8_t mcrMin = 17;         ///< Minimum mean crossings per second. Range 0 to 63.
    uint16_t peakToPeakMax = 253;///< Maximum peak-to-peak magnitude, 1.953 mg/LSB. Range 0 to 1023.
    uint8_t mcrMax = 17;         ///< Maximum mean crossings per second. Range 0 to 63.
};

/**
 * @brief Tilt detector settings.
 *
 * @details Tilt fires when the device's gravity vector has moved more than @p minTiltAngle away
 * from a slowly-updated reference vector. Defaults are the device's power-on values.
 */
struct sfe_bmi330_tilt_config_t
{
    uint8_t segmentSize = 100;  ///< Samples averaged into the reference vector, 20 ms/LSB.
    uint8_t minTiltAngle = 210; ///< Minimum angle to report, as 256 * cos(angle). 210 is about 35 degrees.
    uint16_t betaAccelMean = 0xF069; ///< Exponential smoothing factor for the mean, Q15.
};

/**
 * @brief Orientation detector settings.
 *
 * @details Orientation classifies the device as portrait or landscape (and, optionally, face up or
 * face down) and interrupts on a change. Defaults are the device's power-on values.
 */
struct sfe_bmi330_orientation_config_t
{
    bool faceUpDownEnabled = false; ///< Also classify face up versus face down.
    uint8_t mode = 0;               ///< 0 symmetrical, 1 high asymmetrical, 2 low asymmetrical.
    uint8_t blocking = 3;           ///< Suppress detection during motion. 0 none to 3 most aggressive.
    uint8_t theta = 40;             ///< Blocking angle threshold. Range 0 to 63.
    uint8_t holdTime = 5;           ///< Stability required before reporting, 20 ms/LSB. Range 0 to 31.
    uint8_t slopeThreshold = 0xC8;  ///< Slope threshold used by blocking, 1.953 mg/LSB.
    uint8_t hysteresis = 0x80;      ///< Acceleration hysteresis, 1.953 mg/LSB.
};

/**
 * @brief Flat detector settings.
 *
 * @details Flat fires when the device has been lying within @p theta of horizontal for
 * @p holdTime. Defaults are the device's power-on values.
 */
struct sfe_bmi330_flat_config_t
{
    uint8_t theta = 8;             ///< Angle threshold for "flat". Range 0 to 63.
    uint8_t blocking = 2;          ///< Suppress detection during motion. 0 none to 3 most aggressive.
    uint8_t holdTime = 32;         ///< Stability required before reporting, 20 ms/LSB.
    uint8_t hysteresis = 9;        ///< Angle hysteresis, same units as theta.
    uint8_t slopeThreshold = 0xCD; ///< Slope threshold used by blocking, 1.953 mg/LSB.
};

/**
 * @brief Which axis the tap detector watches.
 */
typedef enum sfe_bmi330_tap_axis_t : uint8_t
{
    BMI330_TAP_AXIS_X = 0x00, ///< Detect taps along X.
    BMI330_TAP_AXIS_Y = 0x01, ///< Detect taps along Y.
    BMI330_TAP_AXIS_Z = 0x02  ///< Detect taps along Z. Power-on default - the usual "tap the lid" axis.
} sfe_bmi330_tap_axis_t;

/**
 * @brief Tap detector settings.
 *
 * @details Tap detection is the fussiest feature on the part, and @p peakThreshold is the knob that
 * matters: too low and footsteps register, too high and nothing does. It is worth sweeping against
 * the actual enclosure - a tap through 3 mm of acrylic is a very different signal from a tap on a
 * bare board. Defaults are the device's power-on values.
 *
 * @note All the duration fields are in 20 ms ticks.
 */
struct sfe_bmi330_tap_config_t
{
    sfe_bmi330_tap_axis_t axis = BMI330_TAP_AXIS_Z; ///< Axis to watch.
    bool waitForTimeout = false;   ///< Wait out the full gesture window before reporting.
    uint8_t maxPeaksForTap = 6;    ///< Zero crossings allowed around one tap. Range 0 to 7.
    uint8_t mode = 0;              ///< 0 sensitive, 1 normal, 2 robust.
    uint16_t peakThreshold = 0x2D; ///< Minimum peak amplitude for a tap. Range 0 to 1023.
    uint8_t maxGestureDuration = 16;   ///< Window for a whole multi-tap gesture. Range 0 to 63.
    uint8_t maxDurationBetweenPeaks = 4; ///< Positive-to-negative peak window. Range 0 to 15.
    uint8_t shockSettlingDuration = 6;   ///< How long a tap's impact is observed. Range 0 to 15.
    uint8_t minQuietBetweenTaps = 8;     ///< Quiet time required between taps. Range 0 to 15.
    uint8_t quietTimeAfterGesture = 12;  ///< Quiet time required after a gesture. Range 0 to 15.
};

/**
 * @brief Which tap gesture the device just reported.
 *
 * @details Read from FEATURE_EVENT_EXT (0x47) after a ::BMI330_INT_STATUS_TAP interrupt. More than
 * one flag can be set if gestures land close together.
 */
struct sfe_bmi330_tap_event_t
{
    bool singleTap = false; ///< A single tap was detected.
    bool doubleTap = false; ///< A double tap was detected.
    bool tripleTap = false; ///< A triple tap was detected.
};

/**
 * @brief Device orientation as classified by the orientation feature.
 */
typedef enum sfe_bmi330_orientation_t : uint8_t
{
    BMI330_ORIENT_PORTRAIT_UPRIGHT = 0x00, ///< Portrait, upright.
    BMI330_ORIENT_LANDSCAPE_LEFT = 0x01,   ///< Landscape, left.
    BMI330_ORIENT_PORTRAIT_UPSIDE_DOWN = 0x02, ///< Portrait, upside down.
    BMI330_ORIENT_LANDSCAPE_RIGHT = 0x03   ///< Landscape, right.
} sfe_bmi330_orientation_t;

///////////////////////////////////////////////////////////////////////////////
// Axis Remapping
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Which physical axis feeds which reported axis.
 *
 * @details Read the name as "reported X, Y, Z come from physical ...". BMI330_AXIS_MAP_YXZ means
 * reported X is the physical Y axis, reported Y is the physical X axis, and Z is unchanged.
 */
typedef enum sfe_bmi330_axis_map_t : uint8_t
{
    BMI330_AXIS_MAP_XYZ = 0x00, ///< No swap. Power-on default.
    BMI330_AXIS_MAP_YXZ = 0x01, ///< Swap X and Y.
    BMI330_AXIS_MAP_XZY = 0x02, ///< Swap Y and Z.
    BMI330_AXIS_MAP_ZXY = 0x03, ///< Rotate: X<-Z, Y<-X, Z<-Y.
    BMI330_AXIS_MAP_YZX = 0x04, ///< Rotate: X<-Y, Y<-Z, Z<-X.
    BMI330_AXIS_MAP_ZYX = 0x05  ///< Swap X and Z.
} sfe_bmi330_axis_map_t;

/**
 * @brief A complete axis remapping: one permutation plus three sign flips.
 *
 * @details This is applied inside the device, so it corrects the accelerometer, the gyroscope, the
 * FIFO and every feature-engine feature at once. A board mounted upside down under a chassis is one
 * setAxisRemap() call, not a scatter of minus signs through the sketch.
 *
 * @note Remapping is handled by the feature engine, so it takes effect only while the engine is
 * running. The device only accepts the commit while the accelerometer is SUSPENDED;
 * setAxisRemap() suspends and restores it around the commit.
 */
struct sfe_bmi330_axis_remap_t
{
    sfe_bmi330_axis_map_t map = BMI330_AXIS_MAP_XYZ; ///< Axis permutation.
    bool invertX = false; ///< Negate the reported X axis.
    bool invertY = false; ///< Negate the reported Y axis.
    bool invertZ = false; ///< Negate the reported Z axis.
};

///////////////////////////////////////////////////////////////////////////////
// Self-Test and Self-Calibration
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Which sensors a self-test exercises.
 */
typedef enum sfe_bmi330_self_test_t : uint8_t
{
    BMI330_SELF_TEST_ACCEL = 0x01, ///< Accelerometer only.
    BMI330_SELF_TEST_GYRO = 0x02,  ///< Gyroscope only.
    BMI330_SELF_TEST_BOTH = 0x03   ///< Both sensors.
} sfe_bmi330_self_test_t;

/**
 * @brief Outcome of a self-test.
 *
 * @details @p completed distinguishes "the test ran and the part failed" from "the test never
 * finished", which are different problems. Check it before trusting anything else in here.
 */
struct sfe_bmi330_self_test_result_t
{
    bool completed = false;    ///< The test ran to completion.
    bool passed = false;       ///< Overall pass. Valid only when @p completed is true.
    bool accelXOk = false;     ///< Accelerometer X sensitivity within limits.
    bool accelYOk = false;     ///< Accelerometer Y sensitivity within limits.
    bool accelZOk = false;     ///< Accelerometer Z sensitivity within limits.
    bool gyroXOk = false;      ///< Gyroscope X sensitivity within limits.
    bool gyroYOk = false;      ///< Gyroscope Y sensitivity within limits.
    bool gyroZOk = false;      ///< Gyroscope Z sensitivity within limits.
    bool gyroDriveOk = false;  ///< Gyroscope drive circuit healthy.
    uint8_t errorStatus = 0;   ///< Raw feature engine error nibble. 0x05 means no error.
};

/**
 * @brief What a gyroscope self-calibration corrects.
 */
typedef enum sfe_bmi330_self_calib_t : uint8_t
{
    BMI330_SELF_CALIB_SENSITIVITY = 0x01, ///< Correct gyroscope sensitivity only.
    BMI330_SELF_CALIB_OFFSET = 0x02,      ///< Correct gyroscope offset only.
    BMI330_SELF_CALIB_BOTH = 0x03         ///< Correct both.
} sfe_bmi330_self_calib_t;

/**
 * @brief Outcome of a gyroscope self-calibration.
 */
struct sfe_bmi330_self_calib_result_t
{
    bool completed = false;  ///< The routine ran to completion.
    bool passed = false;     ///< Calibration succeeded. Valid only when @p completed is true.
    uint8_t errorStatus = 0; ///< Raw feature engine error nibble. 0x05 means no error.
};

///////////////////////////////////////////////////////////////////////////////
// sfDevBMI330
///////////////////////////////////////////////////////////////////////////////

/**
 * @class sfDevBMI330
 * @brief Platform-independent driver for the Bosch BMI330 6-DoF IMU.
 *
 * @details
 * Holds every register address, bit field and conversion the part needs, and talks to the device
 * through an sfTkIBus supplied by begin(). It knows nothing about Arduino, Wire or SPI - the
 * Arduino wrappers in SparkFun_BMI330.h provide those. A sketch normally uses SfeBMI330ArdI2C or
 * SfeBMI330ArdSPI, both of which inherit everything documented here.
 *
 * Methods that touch the bus return sfTkError_t (::ksfTkErrOk on success); methods that only read
 * cached or computed state return the value directly.
 *
 * @see SfeBMI330ArdI2C
 * @see SfeBMI330ArdSPI
 */
class sfDevBMI330
{
  public:
    /**
     * @brief Default constructor. The device is not touched until begin().
     */
    sfDevBMI330() : _theBus{nullptr}, _dummyBytes{kDummyBytesI2C}, _accelRange{BMI330_ACCEL_RANGE_2G},
                    _gyroRange{BMI330_GYRO_RANGE_2000DPS}, _accelOffsetMask{kAccelOffsetMask14},
                    _pendingInt1{0}
    {
    }

    virtual ~sfDevBMI330()
    {
    }

    ///////////////////////////////////////////////////////////////////////////
    // Setup and Identity
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Adopt a bus, verify the device, and bring the feature engine up.
     *
     * @details Determines the read dummy-byte count from the bus type, checks CHIP_ID, picks the
     * accelerometer offset field width from the silicon revision, soft resets - which also starts
     * the feature engine - and then reads ACC_CONF and GYR_CONF back so the cached ranges used by
     * accelRawToG() and gyroRawToDps() describe the device rather than an assumed default. Both
     * sensors are left DISABLED, matching the power-on state; call enableAccel() and enableGyro(),
     * or setAccelConfig() / setGyroConfig(), before expecting data.
     *
     * @param theBus Bus to communicate over. Pass nullptr to keep a bus set earlier.
     * @return ::ksfTkErrOk on success, ::ksfTkErrBusNotInit with no bus, or
     * ::ksfTkErrBusNoResponse if the chip ID does not match.
     */
    sfTkError_t begin(sfTkIBus *theBus = nullptr);

    /**
     * @brief Set the communication bus without initializing the device.
     *
     * @details Also re-derives the dummy-byte count, since it follows the bus type.
     *
     * @param theBus Bus to use for communication.
     */
    void setCommunicationBus(sfTkIBus *theBus);

    /**
     * @brief Check that a BMI330 is responding.
     *
     * @return true if CHIP_ID reads ::SFE_BMI330_CHIP_ID.
     */
    bool isConnected(void);

    /**
     * @brief Read the chip and revision IDs from CHIP_ID (0x00).
     *
     * @param chipId Receives the chip ID - 0x48 for a BMI330, 0x43 for a BMI323.
     * @param revId Receives the silicon revision from the upper nibble.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getChipID(uint8_t &chipId, uint8_t &revId);

    /**
     * @brief Soft reset the device and restart the feature engine.
     *
     * @details Writes the soft-reset command, waits out the 1.5 ms reset time, performs the dummy
     * read that re-selects SPI when on SPI, then runs the feature engine start-up sequence and
     * polls for it to report active. All registers return to their power-on defaults, which means
     * both sensors end up disabled and any configuration is lost.
     *
     * @note The cached full-scale ranges used by accelRawToG() and gyroRawToDps() are
     * resynchronized from the device afterwards, so conversions stay correct across a reset.
     *
     * @return ::ksfTkErrOk on success, ::ksfTkErrFail if the feature engine does not start.
     */
    sfTkError_t softReset(void);

    /**
     * @brief Start the feature engine without a full reset.
     *
     * @details softReset() already does this, so it is rarely needed directly. Useful after the
     * engine has reported an overload or watchdog error in ERR_REG.
     *
     * @return ::ksfTkErrOk if the engine reports active within the start-up window.
     */
    sfTkError_t enableFeatureEngine(void);

    /**
     * @brief Check whether the feature engine is running.
     *
     * @return true if FEATURE_IO1 reports the engine active.
     */
    bool getFeatureEngineStatus(void);

    /**
     * @brief Read the raw FEATURE_IO1 (0x11) word.
     *
     * @details The low nibble is an error code: 0x05 means no error, 0x03 a configuration CRC
     * failure, 0x06 an axis-map error, 0x0C a self-test or self-calibration precondition failure.
     * Higher bits carry the self-test and self-calibration completion and result flags.
     *
     * @param status Receives the register word.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getFeatureEngineError(uint16_t &status);

    ///////////////////////////////////////////////////////////////////////////
    // Accelerometer Configuration
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the whole accelerometer configuration in one register write.
     *
     * @details ACC_CONF (0x20) is a single word holding mode, ODR, range, bandwidth and averaging,
     * so setting them together is one transaction rather than five read-modify-writes. The range is
     * cached here for the raw-to-g conversion.
     *
     * @param mode Power mode.
     * @param odr Output data rate.
     * @param range Full-scale range.
     * @param bandwidth Filter cutoff as a fraction of ODR.
     * @param averaging Hardware averaging.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelConfig(sfe_bmi330_accel_mode_t mode, sfe_bmi330_odr_t odr,
                               sfe_bmi330_accel_range_t range,
                               sfe_bmi330_bandwidth_t bandwidth = BMI330_BW_ODR_HALF,
                               sfe_bmi330_averaging_t averaging = BMI330_AVG_1);

    /**
     * @brief Read the accelerometer configuration back from ACC_CONF (0x20).
     *
     * @details Also refreshes the cached range, so a sketch that configured the part in a previous
     * power cycle gets correct conversions after calling this.
     *
     * @param mode Receives the power mode.
     * @param odr Receives the output data rate.
     * @param range Receives the full-scale range.
     * @param bandwidth Receives the filter setting.
     * @param averaging Receives the averaging setting.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getAccelConfig(sfe_bmi330_accel_mode_t &mode, sfe_bmi330_odr_t &odr,
                               sfe_bmi330_accel_range_t &range, sfe_bmi330_bandwidth_t &bandwidth,
                               sfe_bmi330_averaging_t &averaging);

    /**
     * @brief Turn the accelerometer on in high-performance mode, or off.
     *
     * @param enable true for high-performance mode, false to disable the sensor.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t enableAccel(bool enable = true);

    /**
     * @brief Set the accelerometer power mode, leaving the rest of ACC_CONF alone.
     * @param mode Power mode.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelMode(sfe_bmi330_accel_mode_t mode);

    /**
     * @brief Set the accelerometer output data rate.
     * @param odr Output data rate.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelODR(sfe_bmi330_odr_t odr);

    /**
     * @brief Set the accelerometer full-scale range.
     * @param range Full-scale range.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelRange(sfe_bmi330_accel_range_t range);

    /**
     * @brief Set the accelerometer filter bandwidth.
     * @param bandwidth Cutoff as a fraction of ODR.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelBandwidth(sfe_bmi330_bandwidth_t bandwidth);

    /**
     * @brief Set the accelerometer hardware averaging.
     * @param averaging Samples to average.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelAveraging(sfe_bmi330_averaging_t averaging);

    /**
     * @brief Get the cached accelerometer full-scale range.
     *
     * @details This is what the conversion math uses. It tracks setAccelConfig(), setAccelRange()
     * and getAccelConfig(); it does not read the device.
     *
     * @return The cached range.
     */
    sfe_bmi330_accel_range_t getAccelRange(void)
    {
        return _accelRange;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Gyroscope Configuration
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the whole gyroscope configuration in one register write.
     *
     * @param mode Power mode.
     * @param odr Output data rate.
     * @param range Full-scale range.
     * @param bandwidth Filter cutoff as a fraction of ODR.
     * @param averaging Hardware averaging.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroConfig(sfe_bmi330_gyro_mode_t mode, sfe_bmi330_odr_t odr,
                              sfe_bmi330_gyro_range_t range,
                              sfe_bmi330_bandwidth_t bandwidth = BMI330_BW_ODR_HALF,
                              sfe_bmi330_averaging_t averaging = BMI330_AVG_1);

    /**
     * @brief Read the gyroscope configuration back from GYR_CONF (0x21).
     *
     * @param mode Receives the power mode.
     * @param odr Receives the output data rate.
     * @param range Receives the full-scale range.
     * @param bandwidth Receives the filter setting.
     * @param averaging Receives the averaging setting.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getGyroConfig(sfe_bmi330_gyro_mode_t &mode, sfe_bmi330_odr_t &odr,
                              sfe_bmi330_gyro_range_t &range, sfe_bmi330_bandwidth_t &bandwidth,
                              sfe_bmi330_averaging_t &averaging);

    /**
     * @brief Turn the gyroscope on in high-performance mode, or off.
     * @param enable true for high-performance mode, false to disable the sensor.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t enableGyro(bool enable = true);

    /**
     * @brief Set the gyroscope power mode, leaving the rest of GYR_CONF alone.
     * @param mode Power mode.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroMode(sfe_bmi330_gyro_mode_t mode);

    /**
     * @brief Set the gyroscope output data rate.
     * @param odr Output data rate.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroODR(sfe_bmi330_odr_t odr);

    /**
     * @brief Set the gyroscope full-scale range.
     * @param range Full-scale range.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroRange(sfe_bmi330_gyro_range_t range);

    /**
     * @brief Set the gyroscope filter bandwidth.
     * @param bandwidth Cutoff as a fraction of ODR.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroBandwidth(sfe_bmi330_bandwidth_t bandwidth);

    /**
     * @brief Set the gyroscope hardware averaging.
     * @param averaging Samples to average.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroAveraging(sfe_bmi330_averaging_t averaging);

    /**
     * @brief Get the cached gyroscope full-scale range.
     * @return The cached range.
     */
    sfe_bmi330_gyro_range_t getGyroRange(void)
    {
        return _gyroRange;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Measurement
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Read accelerometer, gyroscope, temperature and timestamp in one transaction.
     *
     * @details One 9-word burst from ACC_DATA_X (0x03) to SENSOR_TIME_1 (0x0B), so every field
     * describes the same instant. Both raw and converted forms are filled in - the conversion is a
     * multiply per axis and costs nothing next to the bus traffic.
     *
     * @param data Receives the reading.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getSensorData(sfe_bmi330_sensor_data_t &data);

    /**
     * @brief Read just the accelerometer, converted to g.
     * @param data Receives the acceleration.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getAccelData(sfe_bmi330_data_t &data);

    /**
     * @brief Read just the accelerometer, in raw counts.
     * @param data Receives the raw acceleration.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getAccelDataRaw(sfe_bmi330_raw_data_t &data);

    /**
     * @brief Read just the gyroscope, converted to degrees per second.
     * @param data Receives the angular rate.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getGyroData(sfe_bmi330_data_t &data);

    /**
     * @brief Read just the gyroscope, in raw counts.
     * @param data Receives the raw angular rate.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getGyroDataRaw(sfe_bmi330_raw_data_t &data);

    /**
     * @brief Read the die temperature in degrees Celsius.
     *
     * @note This is the temperature of the silicon, which runs warm - it tracks ambient but is not
     * an ambient sensor. Its value is in compensating the gyroscope, not in reporting the weather.
     *
     * @param celsius Receives the temperature.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getTemperature(float &celsius);

    /**
     * @brief Read the 32-bit sensor timestamp, in ticks of 39.0625 us.
     *
     * @details The counter runs from the device's own oscillator, so it timestamps samples far more
     * precisely than millis() can - the host's read latency is not in the number. It wraps about
     * about every 46.6 hours (2^32 ticks).
     *
     * @param sensorTime Receives the timestamp.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getSensorTime(uint32_t &sensorTime);

    /**
     * @brief Read the STATUS register (0x02) data-ready flags.
     *
     * @note The register is CLEAR-ON-READ: this call consumes the flags. Read once and use all
     * three results.
     *
     * @param accelReady Receives the accelerometer data-ready flag.
     * @param gyroReady Receives the gyroscope data-ready flag.
     * @param tempReady Receives the temperature data-ready flag.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getDataReady(bool &accelReady, bool &gyroReady, bool &tempReady);

    /**
     * @brief Read and decode ERR_REG (0x01).
     * @param status Receives the decoded error flags.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getErrorStatus(sfe_bmi330_error_status_t &status);

    /**
     * @brief Read and decode the per-axis saturation flags in SAT_FLAGS (0x0C).
     * @param flags Receives the decoded flags.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getSaturationFlags(sfe_bmi330_saturation_t &flags);

    ///////////////////////////////////////////////////////////////////////////
    // Unit Conversion
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Convert a raw accelerometer count to g using the cached range.
     * @param raw Raw 16-bit signed count.
     * @return Acceleration in g.
     */
    float accelRawToG(int16_t raw);

    /**
     * @brief Convert a raw gyroscope count to degrees per second using the cached range.
     * @param raw Raw 16-bit signed count.
     * @return Angular rate in degrees per second.
     */
    float gyroRawToDps(int16_t raw);

    /**
     * @brief Convert a raw temperature count to degrees Celsius.
     *
     * @details The BMI330 reports 256 LSB per kelvin with zero at 23 degrees C, so the conversion
     * is raw / 256 + 23.
     *
     * @note This divisor is NOT the same on the BMI323, which uses 512. A driver ported from the
     * BMI323 without changing this reads roughly half the true excursion from 23 degrees.
     *
     * @param raw Raw 16-bit signed count.
     * @return Temperature in degrees Celsius.
     */
    static float rawToCelsius(int16_t raw);

    /**
     * @brief Sensitivity of a given accelerometer range, in LSB per g.
     * @param range Full-scale range.
     * @return LSB per g: 16384, 8192, 4096 or 2048.
     */
    static float accelSensitivity(sfe_bmi330_accel_range_t range);

    /**
     * @brief Sensitivity of a given gyroscope range, in LSB per degree per second.
     * @param range Full-scale range.
     * @return LSB per dps.
     */
    static float gyroSensitivity(sfe_bmi330_gyro_range_t range);

    ///////////////////////////////////////////////////////////////////////////
    // Interrupts
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Configure the electrical behavior of one interrupt pin.
     *
     * @details Both pins share IO_INT_CTRL (0x38), so this is a read-modify-write and the other
     * pin's settings are preserved.
     *
     * @param pin Which pin to configure. ::BMI330_INT_NONE is ignored.
     * @param config Drive strength, polarity and enable.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setInterruptPinConfig(sfe_bmi330_int_pin_t pin, const sfe_bmi330_int_pin_config_t &config);

    /**
     * @brief Choose between latched and pulsed interrupts.
     *
     * @details Pulsed (the default) asserts the pin briefly per event. Latched holds it asserted
     * until the status register is read, which is what an edge-triggered host ISR that might miss a
     * short pulse wants - at the cost of requiring the status read to happen.
     *
     * @note The setting is global: it applies to both pins and every source.
     *
     * @param latched true to latch until the status register is read.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setInterruptLatch(bool latched);

    /**
     * @brief Route one interrupt source to a pin, or disconnect it.
     *
     * @param source The source to route.
     * @param pin Destination pin, or ::BMI330_INT_NONE to disconnect.
     * @return ::ksfTkErrOk on success, ::ksfTkErrInvalidParam for an unknown source.
     */
    sfTkError_t mapInterrupt(sfe_bmi330_int_source_t source, sfe_bmi330_int_pin_t pin);

    /**
     * @brief Read and clear a pin's interrupt status word.
     *
     * @note CLEAR-ON-READ. Every pending flag for that pin is consumed by this call, so test the
     * returned word against the ::sfe_bmi330_int_status_t masks rather than calling again per bit.
     *
     * @param pin Which pin's status to read.
     * @param status Receives the status word.
     * @return ::ksfTkErrOk on success, ::ksfTkErrInvalidParam for ::BMI330_INT_NONE.
     */
    sfTkError_t getInterruptStatus(sfe_bmi330_int_pin_t pin, uint16_t &status);

    ///////////////////////////////////////////////////////////////////////////
    // FIFO
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Choose what the FIFO records, and what it does when full.
     *
     * @param sources Bitwise OR of ::sfe_bmi330_fifo_source_t values. Zero disables the FIFO.
     * @param stopOnFull true to stop writing when full (keep the oldest data), false to overwrite
     * the oldest data and keep the newest.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setFifoConfig(uint16_t sources, bool stopOnFull = false);

    /**
     * @brief Read back the configured FIFO sources and full behavior.
     * @param sources Receives the enabled source bits.
     * @param stopOnFull Receives the stop-on-full setting.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getFifoConfig(uint16_t &sources, bool &stopOnFull);

    /**
     * @brief Set the FIFO watermark, in 16-bit words.
     *
     * @details The watermark interrupt fires when the fill level reaches this many WORDS - not
     * bytes and not frames. A frame is 3 words of accelerometer plus 3 of gyroscope plus 1 each of
     * temperature and time, counting only enabled sources, so a watermark of 80 words is 10 frames
     * of accelerometer-plus-gyroscope. frameLengthWords() does the arithmetic.
     *
     * @param words Watermark level in words. Range 0 to 1023.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setFifoWatermark(uint16_t words);

    /**
     * @brief Read the FIFO watermark, in words.
     * @param words Receives the watermark level.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getFifoWatermark(uint16_t &words);

    /**
     * @brief Read how many words the FIFO currently holds.
     * @param words Receives the fill level in words. The buffer holds 1024 words (2 kB).
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getFifoFillLevel(uint16_t &words);

    /**
     * @brief Discard everything in the FIFO.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t flushFifo(void);

    /**
     * @brief Read raw words out of FIFO_DATA (0x16) without interpreting them.
     *
     * @details The FIFO_DATA register does not auto-increment - the whole burst streams from one
     * address. Useful for logging the FIFO verbatim and decoding later, off the device.
     *
     * @note Reads longer than ::kMaxBurstWords are split into several transactions, always on word
     * boundaries. Because the device retransmits a frame that was only partially read out, ask for
     * a whole number of frames, or an exact multiple of ::kMaxBurstWords - readFifoData() already
     * does the former.
     *
     * @param words Destination buffer, at least @p count words.
     * @param count Number of words to read.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t readFifoRaw(uint16_t *words, uint16_t count);

    /**
     * @brief Read and decode complete frames from the FIFO.
     *
     * @details Reads only whole frames - if the FIFO holds a partial one, it is left in place for
     * the next call rather than being torn in half. Frames whose source was idle at capture carry
     * an invalid pattern, which is reported as a cleared @p has... flag.
     *
     * @param frames Destination array.
     * @param maxFrames Capacity of @p frames.
     * @param numFrames Receives the number of frames actually decoded.
     * @param sources The same source bits that were passed to setFifoConfig(). The FIFO is
     * headerless, so the parser cannot work this out on its own.
     * @return ::ksfTkErrOk on success, including when zero frames were available.
     */
    sfTkError_t readFifoData(sfe_bmi330_fifo_frame_t *frames, uint8_t maxFrames, uint8_t &numFrames,
                             uint16_t sources);

    /**
     * @brief Words per FIFO frame for a given set of sources.
     * @param sources Bitwise OR of ::sfe_bmi330_fifo_source_t values.
     * @return Frame length in 16-bit words, or 0 if no source is enabled.
     */
    static uint8_t frameLengthWords(uint16_t sources);

    ///////////////////////////////////////////////////////////////////////////
    // Feature Engine Access
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Enable or disable feature-engine features.
     *
     * @details Read-modify-write of FEATURE_IO0 (0x10) followed by the commit write to
     * FEATURE_IO_STATUS (0x14) that tells the engine to pick the change up. Without the commit the
     * bits sit in the register and nothing happens.
     *
     * @param features Bitwise OR of ::sfe_bmi330_feature_t values.
     * @param enable true to set those bits, false to clear them.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t enableFeature(uint16_t features, bool enable = true);

    /**
     * @brief Read which features are currently enabled.
     * @param features Receives the FEATURE_IO0 word.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getEnabledFeatures(uint16_t &features);

    /**
     * @brief Read consecutive words from feature engine extended memory.
     *
     * @param baseAddr Feature base address - see ::sfe_bmi330_feature_addr_t.
     * @param words Destination buffer.
     * @param count Number of words to read. Maximum ::kMaxBurstWords.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t readFeatureWords(uint8_t baseAddr, uint16_t *words, uint8_t count);

    /**
     * @brief Write consecutive words to feature engine extended memory.
     *
     * @param baseAddr Feature base address - see ::sfe_bmi330_feature_addr_t.
     * @param words Source buffer.
     * @param count Number of words to write. Maximum ::kMaxBurstWords.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t writeFeatureWords(uint8_t baseAddr, const uint16_t *words, uint8_t count);

    ///////////////////////////////////////////////////////////////////////////
    // Motion Features
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the any-motion detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAnyMotionConfig(const sfe_bmi330_motion_config_t &config);

    /**
     * @brief Read the any-motion detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getAnyMotionConfig(sfe_bmi330_motion_config_t &config);

    /**
     * @brief Write the no-motion detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setNoMotionConfig(const sfe_bmi330_motion_config_t &config);

    /**
     * @brief Read the no-motion detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getNoMotionConfig(sfe_bmi330_motion_config_t &config);

    /**
     * @brief Write the significant motion detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setSigMotionConfig(const sfe_bmi330_sig_motion_config_t &config);

    /**
     * @brief Read the significant motion detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getSigMotionConfig(sfe_bmi330_sig_motion_config_t &config);

    /**
     * @brief Write the tilt detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setTiltConfig(const sfe_bmi330_tilt_config_t &config);

    /**
     * @brief Read the tilt detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getTiltConfig(sfe_bmi330_tilt_config_t &config);

    /**
     * @brief Write the orientation detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setOrientationConfig(const sfe_bmi330_orientation_config_t &config);

    /**
     * @brief Read the orientation detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getOrientationConfig(sfe_bmi330_orientation_config_t &config);

    /**
     * @brief Read the current orientation classification.
     * @param orientation Receives the portrait / landscape classification.
     * @param faceDown Receives the face-down flag. Meaningful only with face up / down enabled.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getOrientation(sfe_bmi330_orientation_t &orientation, bool &faceDown);

    /**
     * @brief Write the flat detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setFlatConfig(const sfe_bmi330_flat_config_t &config);

    /**
     * @brief Read the flat detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getFlatConfig(sfe_bmi330_flat_config_t &config);

    ///////////////////////////////////////////////////////////////////////////
    // Tap Detection
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the tap detector configuration.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setTapConfig(const sfe_bmi330_tap_config_t &config);

    /**
     * @brief Read the tap detector configuration.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getTapConfig(sfe_bmi330_tap_config_t &config);

    /**
     * @brief Read which tap gesture was last detected.
     *
     * @details Call after a ::BMI330_INT_STATUS_TAP interrupt. The single, double and triple flags
     * live in FEATURE_EVENT_EXT (0x47).
     *
     * @param event Receives the gesture flags.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getTapEvent(sfe_bmi330_tap_event_t &event);

    ///////////////////////////////////////////////////////////////////////////
    // Step Counter
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Read the step count.
     *
     * @details The counter is a 32-bit value split across FEATURE_IO2 (0x12) and FEATURE_IO3
     * (0x13), read here as one two-word burst so the halves cannot disagree.
     *
     * @param steps Receives the step count.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getStepCount(uint32_t &steps);

    /**
     * @brief Reset the step count to zero.
     *
     * @details Sets and then clears the reset bit in the step counter's first configuration word,
     * leaving the rest of that 12-word block untouched.
     *
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t resetStepCount(void);

    /**
     * @brief Set how many steps trigger the step counter interrupt.
     *
     * @details The count is in units of 20 steps: a watermark of 1 interrupts every 20 steps. Zero
     * disables the watermark interrupt while leaving the counter running, which is what a sketch
     * that polls getStepCount() wants.
     *
     * @param watermark Watermark in units of 20 steps. Range 0 to 1023.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setStepCounterWatermark(uint16_t watermark);

    /**
     * @brief Read the step counter watermark.
     * @param watermark Receives the watermark, in units of 20 steps.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getStepCounterWatermark(uint16_t &watermark);

    ///////////////////////////////////////////////////////////////////////////
    // Axis Remapping
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Apply an axis remapping inside the device.
     *
     * @details Writes the map into feature memory, then commits it. The commit is the fiddly part:
     * the device will only accept an axis-map update while the ACCELEROMETER IS SUSPENDED, so this
     * method saves the current accelerometer power mode, disables the sensor, issues the
     * axis-map-update command, waits for the feature engine to acknowledge it, clears the INT1
     * status the update leaves behind, and restores the saved mode. INT1 events that were already
     * pending are kept and returned by the next getInterruptStatus(BMI330_INT1). Sampling is interrupted for a
     * few tens of milliseconds; the caller's configuration is not otherwise disturbed.
     *
     * @note If the accelerometer is already disabled when this is called, the command is issued and
     * the method returns without polling - which is what Bosch's own driver does, because the
     * acknowledgement is only produced on the suspend-and-restore path.
     *
     * @param remap Axis permutation and sign flips.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAxisRemap(const sfe_bmi330_axis_remap_t &remap);

    /**
     * @brief Read the current axis remapping.
     * @param remap Receives the permutation and sign flips.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getAxisRemap(sfe_bmi330_axis_remap_t &remap);

    ///////////////////////////////////////////////////////////////////////////
    // Self-Test and Self-Calibration
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Run the built-in self-test.
     *
     * @details Drives a known stimulus into the selected sensors and checks the response against
     * factory limits. On the BMI330 this also involves seeding the gyroscope filter coefficients in
     * feature memory, which the driver handles - the values differ from the BMI323's, and a test
     * run without them reports failures on a healthy part.
     *
     * @note DISRUPTIVE AND SLOW. The routine forces the accelerometer to high-performance mode at
     * 100 Hz, takes up to 350 ms, and leaves both sensors reconfigured. Run it at start-up or on
     * demand, restore the configuration afterwards, and keep the device STILL while it runs.
     *
     * @note Per-axis flags are filled in only for the sensors named by @p selection. The bits
     * belonging to a sensor that was not tested are not meaningful for that run, so they are left
     * false rather than reported as failures.
     *
     * @param selection Which sensors to test.
     * @param result Receives the outcome.
     * @return ::ksfTkErrOk if the routine ran. Check @p result for the verdict.
     */
    sfTkError_t performSelfTest(sfe_bmi330_self_test_t selection, sfe_bmi330_self_test_result_t &result);

    /**
     * @brief Run the gyroscope self-calibration.
     *
     * @details Measures and corrects the gyroscope's offset, its sensitivity, or both, storing the
     * result in the device's user registers.
     *
     * @note PRECONDITIONS MATTER. The device must be MOTIONLESS for the whole run, and the
     * accelerometer must be in high-performance mode at 100 Hz and +/-8 g - which this method sets
     * up and then restores. Expect up to 430 ms. A calibration run on a moving device reports
     * success and leaves a worse offset than it started with.
     *
     * @param selection What to correct.
     * @param applyCorrection true to have the device apply the result to its output immediately.
     * @param result Receives the outcome.
     * @return ::ksfTkErrOk if the routine ran. Check @p result for the verdict.
     */
    sfTkError_t performGyroSelfCalibration(sfe_bmi330_self_calib_t selection, bool applyCorrection,
                                           sfe_bmi330_self_calib_result_t &result);

    ///////////////////////////////////////////////////////////////////////////
    // User Offsets
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Write the accelerometer user offsets.
     *
     * @details Offsets are signed values applied in the data path before the output registers, at a
     * fixed 0.0039 g per LSB REGARDLESS of the selected range - changing the range does not rescale
     * them.
     *
     * @note The field is 14 bits on revision 1 and later silicon and 13 bits on earlier parts.
     * begin() reads the revision and picks; values are masked to whichever width this device has.
     *
     * @param x X axis offset, 14-bit signed.
     * @param y Y axis offset, 14-bit signed.
     * @param z Z axis offset, 14-bit signed.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setAccelOffset(int16_t x, int16_t y, int16_t z);

    /**
     * @brief Read the accelerometer user offsets.
     * @param x Receives the X axis offset.
     * @param y Receives the Y axis offset.
     * @param z Receives the Z axis offset.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getAccelOffset(int16_t &x, int16_t &y, int16_t &z);

    /**
     * @brief Write the gyroscope user offsets.
     *
     * @details Offsets are 10-bit signed values at a fixed 0.061 dps per LSB, applied before the
     * output registers and independent of the selected range.
     *
     * @param x X axis offset, 10-bit signed.
     * @param y Y axis offset, 10-bit signed.
     * @param z Z axis offset, 10-bit signed.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setGyroOffset(int16_t x, int16_t y, int16_t z);

    /**
     * @brief Read the gyroscope user offsets.
     * @param x Receives the X axis offset.
     * @param y Receives the Y axis offset.
     * @param z Receives the Z axis offset.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getGyroOffset(int16_t &x, int16_t &y, int16_t &z);

    ///////////////////////////////////////////////////////////////////////////
    // Low-Level Register Access
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Read one 16-bit register, dummy bytes handled.
     * @param reg Register address.
     * @param value Receives the register contents.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t readRegister(uint8_t reg, uint16_t &value);

    /**
     * @brief Write one 16-bit register.
     * @param reg Register address.
     * @param value Value to write.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t writeRegister(uint8_t reg, uint16_t value);

    /**
     * @brief Read consecutive 16-bit registers in one transaction, dummy bytes handled.
     * @param reg First register address.
     * @param values Destination buffer.
     * @param count Number of words. Maximum ::kMaxBurstWords.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t readRegisters(uint8_t reg, uint16_t *values, uint8_t count);

    /**
     * @brief Write consecutive 16-bit registers in one transaction.
     * @param reg First register address.
     * @param values Source buffer.
     * @param count Number of words. Maximum ::kMaxBurstWords.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t writeRegisters(uint8_t reg, const uint16_t *values, uint8_t count);

    /**
     * @brief Send a command through the CMD register (0x7E).
     * @param command Command word.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t sendCommand(uint16_t command);

  protected:
    ///////////////////////////////////////////////////////////////////////////
    // Internal Helpers
    ///////////////////////////////////////////////////////////////////////////

    /**
     * @brief Read-modify-write a bit field of a 16-bit register.
     * @param reg Register address.
     * @param mask Field mask.
     * @param value Field value, already shifted into position.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t updateRegister(uint8_t reg, uint16_t mask, uint16_t value);

    /**
     * @brief Read-modify-write a bit field of a feature engine word.
     * @param baseAddr Feature base address.
     * @param wordIndex Word offset within the feature block.
     * @param mask Field mask.
     * @param value Field value, already shifted into position.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t updateFeatureWord(uint8_t baseAddr, uint8_t wordIndex, uint16_t mask, uint16_t value);

    /**
     * @brief Shared implementation for the any-motion and no-motion configuration writers.
     * @param baseAddr ::BMI330_FEAT_ADDR_ANY_MOTION or ::BMI330_FEAT_ADDR_NO_MOTION.
     * @param config Detector settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t setMotionConfig(uint8_t baseAddr, const sfe_bmi330_motion_config_t &config);

    /**
     * @brief Shared implementation for the any-motion and no-motion configuration readers.
     * @param baseAddr ::BMI330_FEAT_ADDR_ANY_MOTION or ::BMI330_FEAT_ADDR_NO_MOTION.
     * @param config Receives the settings.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t getMotionConfig(uint8_t baseAddr, sfe_bmi330_motion_config_t &config);

    /**
     * @brief Seed the gyroscope filter coefficients required by BMI330 self-test and
     * self-calibration.
     *
     * @details The BMI330 ships with coefficients that differ from the BMI323's, and the routines
     * only produce meaningful results against the BMI330 set. This writes them if what is already
     * there does not match.
     *
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t primeGyroFilterCoefficients(void);

    /**
     * @brief Poll FEATURE_IO1 until the self-test / self-calibration completion bit is set.
     * @param featureIO1 Receives the final FEATURE_IO1 word.
     * @param stepDelayMs Delay between polls, in milliseconds.
     * @return ::ksfTkErrOk if the bit set within the polling window, ::ksfTkErrBusTimeout otherwise.
     */
    sfTkError_t waitForSelfTestComplete(uint16_t &featureIO1, uint16_t stepDelayMs);

    /**
     * @brief Read up to ::kMaxBurstWords words from the FIFO in one transaction.
     *
     * @details Splitting a FIFO read at a frame boundary is safe; splitting it inside a frame is
     * not, because the device retransmits a partially-read frame. Callers are responsible for
     * choosing a frame-aligned @p count.
     *
     * @param words Destination buffer.
     * @param count Number of words. Maximum ::kMaxBurstWords.
     * @return ::ksfTkErrOk on success.
     */
    sfTkError_t readFifoWords(uint16_t *words, uint8_t count);

    /**
     * @brief Sign-extend a right-aligned two's complement field to a full int16_t.
     * @param value Field value, right-aligned, upper bits zero.
     * @param bits Width of the field in bits.
     * @return The sign-extended value.
     */
    static int16_t signExtend(uint16_t value, uint8_t bits);

    /** @brief The bus this driver communicates over. */
    sfTkIBus *_theBus;

    /** @brief Dummy bytes the device prepends to every read: 2 on I2C, 1 on SPI. */
    uint8_t _dummyBytes;

    /** @brief Cached accelerometer range, used by accelRawToG(). */
    sfe_bmi330_accel_range_t _accelRange;

    /** @brief Cached gyroscope range, used by gyroRawToDps(). */
    sfe_bmi330_gyro_range_t _gyroRange;

    /**
     * @brief Mask for the accelerometer user offset field, set from the silicon revision.
     *
     * @details Revision 1 and later carry a 14-bit offset field; earlier silicon carries 13 bits.
     * begin() reads the revision and picks. Using the wrong width writes a bit outside the field
     * and sign-extends reads from the wrong place.
     */
    uint16_t _accelOffsetMask;

    /**
     * @brief INT1 status bits that setAxisRemap() had to read (and so clear) before its commit.
     *
     * @details INT_STATUS_INT1 is clear-on-read. setAxisRemap() captures whatever is pending before
     * it suspends the accelerometer, and the next getInterruptStatus(BMI330_INT1) merges these bits
     * back in, so a remap never swallows a caller's event.
     */
    uint16_t _pendingInt1;

    ///////////////////////////////////////////////////////////////////////////
    // Register Map
    ///////////////////////////////////////////////////////////////////////////
    //
    // Every register is a 16-bit little-endian word. Reads carry leading dummy bytes; see the file
    // header.

    static const uint8_t kRegChipId = 0x00;             ///< Chip and revision ID.
    static const uint8_t kRegErrReg = 0x01;             ///< Error flags.
    static const uint8_t kRegStatus = 0x02;             ///< Data ready and POR flags. Clear-on-read.
    static const uint8_t kRegAccelDataX = 0x03;         ///< Accelerometer X. Start of the data block.
    static const uint8_t kRegGyroDataX = 0x06;          ///< Gyroscope X.
    static const uint8_t kRegTempData = 0x09;           ///< Temperature.
    static const uint8_t kRegSensorTime0 = 0x0A;        ///< Sensor time, low word.
    static const uint8_t kRegSatFlags = 0x0C;           ///< Per-axis saturation flags.
    static const uint8_t kRegIntStatusInt1 = 0x0D;      ///< INT1 status. Clear-on-read.
    static const uint8_t kRegIntStatusInt2 = 0x0E;      ///< INT2 status. Clear-on-read.
    static const uint8_t kRegFeatureIO0 = 0x10;         ///< Feature enable bits.
    static const uint8_t kRegFeatureIO1 = 0x11;         ///< Feature engine status and errors.
    static const uint8_t kRegFeatureIO2 = 0x12;         ///< Step count low word / start-up config.
    static const uint8_t kRegFeatureIO3 = 0x13;         ///< Step count high word / start-up config.
    static const uint8_t kRegFeatureIOStatus = 0x14;    ///< Commit register for FEATURE_IO0 changes.
    static const uint8_t kRegFifoFillLevel = 0x15;      ///< FIFO fill level, in words.
    static const uint8_t kRegFifoData = 0x16;           ///< FIFO read port. Does not auto-increment.
    static const uint8_t kRegAccelConf = 0x20;          ///< Accelerometer configuration.
    static const uint8_t kRegGyroConf = 0x21;           ///< Gyroscope configuration.
    static const uint8_t kRegAltAccelConf = 0x28;       ///< Alternate accelerometer configuration.
    static const uint8_t kRegAltGyroConf = 0x29;        ///< Alternate gyroscope configuration.
    static const uint8_t kRegFifoWatermark = 0x35;      ///< FIFO watermark, in words.
    static const uint8_t kRegFifoConf = 0x36;           ///< FIFO source enables and stop-on-full.
    static const uint8_t kRegFifoCtrl = 0x37;           ///< FIFO flush.
    static const uint8_t kRegIoIntCtrl = 0x38;          ///< Interrupt pin electrical configuration.
    static const uint8_t kRegIntConf = 0x39;            ///< Interrupt latch mode.
    static const uint8_t kRegIntMap1 = 0x3A;            ///< Interrupt routing, feature sources.
    static const uint8_t kRegIntMap2 = 0x3B;            ///< Interrupt routing, data and FIFO sources.
    static const uint8_t kRegFeatureCtrl = 0x40;        ///< Feature engine enable.
    static const uint8_t kRegFeatureDataAddr = 0x41;    ///< Feature memory address window.
    static const uint8_t kRegFeatureDataTx = 0x42;      ///< Feature memory data port. Auto-increments.
    static const uint8_t kRegFeatureEventExt = 0x47;    ///< Orientation and tap event outputs.
    static const uint8_t kRegAccelDpOffX = 0x60;        ///< Accelerometer X user offset.
    static const uint8_t kRegGyroDpOffX = 0x66;         ///< Gyroscope X user offset.
    static const uint8_t kRegCmd = 0x7E;                ///< Command register.

    ///////////////////////////////////////////////////////////////////////////
    // Commands
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kCmdSelfTestTrigger = 0x0100;  ///< Start a self-test.
    static const uint16_t kCmdSelfCalibTrigger = 0x0101; ///< Start a gyroscope self-calibration.
    static const uint16_t kCmdSelfCalibAbort = 0x0200;   ///< Abort a self-calibration.
    static const uint16_t kCmdAxisMapUpdate = 0x0300;    ///< Commit an axis remap.
    static const uint16_t kCmdSoftReset = 0xDEAF;        ///< Soft reset.

    ///////////////////////////////////////////////////////////////////////////
    // ACC_CONF / GYR_CONF Bit Fields
    ///////////////////////////////////////////////////////////////////////////
    //
    // The two registers share a layout, so one set of masks serves both.

    static const uint16_t kConfOdrMask = 0x000F;   ///< Output data rate, bits 3:0.
    static const uint16_t kConfRangeMask = 0x0070; ///< Full-scale range, bits 6:4.
    static const uint8_t kConfRangeShift = 4;      ///< Range field position.
    static const uint16_t kConfBwMask = 0x0080;    ///< Bandwidth, bit 7.
    static const uint8_t kConfBwShift = 7;         ///< Bandwidth field position.
    static const uint16_t kConfAvgMask = 0x0700;   ///< Averaging, bits 10:8.
    static const uint8_t kConfAvgShift = 8;        ///< Averaging field position.
    static const uint16_t kConfModeMask = 0x7000;  ///< Power mode, bits 14:12.
    static const uint8_t kConfModeShift = 12;      ///< Power mode field position.

    ///////////////////////////////////////////////////////////////////////////
    // STATUS, ERR_REG and SAT_FLAGS Bits
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kStatusTempDrdy = 0x0020;  ///< Temperature data ready.
    static const uint16_t kStatusGyroDrdy = 0x0040;  ///< Gyroscope data ready.
    static const uint16_t kStatusAccelDrdy = 0x0080; ///< Accelerometer data ready.

    static const uint16_t kErrFatal = 0x0001;        ///< Fatal error.
    static const uint16_t kErrFeatEngOverload = 0x0004; ///< Feature engine overload.
    static const uint16_t kErrFeatEngWatchdog = 0x0010; ///< Feature engine watchdog.
    static const uint16_t kErrAccelConf = 0x0020;    ///< Unsupported accelerometer configuration.
    static const uint16_t kErrGyroConf = 0x0040;     ///< Unsupported gyroscope configuration.

    static const uint16_t kSatAccelX = 0x0001; ///< Accelerometer X saturated.
    static const uint16_t kSatAccelY = 0x0002; ///< Accelerometer Y saturated.
    static const uint16_t kSatAccelZ = 0x0004; ///< Accelerometer Z saturated.
    static const uint16_t kSatGyroX = 0x0008;  ///< Gyroscope X saturated.
    static const uint16_t kSatGyroY = 0x0010;  ///< Gyroscope Y saturated.
    static const uint16_t kSatGyroZ = 0x0020;  ///< Gyroscope Z saturated.

    ///////////////////////////////////////////////////////////////////////////
    // Interrupt Configuration Bits
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kInt1LevelMask = 0x0001;    ///< INT1 polarity, 1 = active high.
    static const uint16_t kInt1OpenDrainMask = 0x0002;///< INT1 open-drain.
    static const uint16_t kInt1OutputEnMask = 0x0004; ///< INT1 output enable.
    static const uint16_t kInt2LevelMask = 0x0100;    ///< INT2 polarity, 1 = active high.
    static const uint16_t kInt2OpenDrainMask = 0x0200;///< INT2 open-drain.
    static const uint16_t kInt2OutputEnMask = 0x0400; ///< INT2 output enable.
    static const uint16_t kIntLatchMask = 0x0001;     ///< Latched interrupt mode.

    ///////////////////////////////////////////////////////////////////////////
    // FIFO Bits
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kFifoStopOnFullMask = 0x0001; ///< Stop writing when full.
    static const uint16_t kFifoSourceMask = 0x0F00;     ///< Source enable field.
    static const uint16_t kFifoFlushMask = 0x0001;      ///< Flush command bit in FIFO_CTRL.
    static const uint16_t kFifoWatermarkMask = 0x03FF;  ///< Watermark field.
    static const uint16_t kFifoFillLevelMask = 0x07FF;  ///< Fill level field.

    /** @brief Sentinel word written into a FIFO accelerometer slot when the sensor was idle. */
    static const uint16_t kFifoAccelInvalid = 0x7F01;
    /** @brief Sentinel word written into a FIFO gyroscope slot when the sensor was idle. */
    static const uint16_t kFifoGyroInvalid = 0x7F02;
    /** @brief Sentinel word written into a FIFO temperature slot when the sensor was idle. */
    static const uint16_t kFifoTempInvalid = 0x8000;

    ///////////////////////////////////////////////////////////////////////////
    // Feature Engine Bits
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kFeatureEngineEnable = 0x0001;  ///< Engine enable / active flag.
    static const uint16_t kFeatureIOStatusCommit = 0x0001;///< Commit bit for FEATURE_IO0 changes.
    static const uint16_t kFeatureStartupConfig = 0x012C; ///< Start-up value written to FEATURE_IO2.
    static const uint16_t kFeatureErrorMask = 0x000F;     ///< Error nibble in FEATURE_IO1.
    static const uint16_t kFeatureNoError = 0x0005;       ///< Error nibble value meaning "no error".
    static const uint16_t kScStComplete = 0x0010;         ///< Self-test / self-calibration complete.
    static const uint16_t kScResult = 0x0020;             ///< Gyroscope self-calibration result.
    static const uint16_t kStResult = 0x0040;             ///< Self-test result.
    static const uint16_t kAxisMapComplete = 0x0400;      ///< Axis map applied.
    static const uint16_t kFeatureAddrMask = 0x07FF;      ///< Address field of FEATURE_DATA_ADDR.

    ///////////////////////////////////////////////////////////////////////////
    // Feature Output Bits (FEATURE_EVENT_EXT, 0x47)
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kEventOrientationMask = 0x0003; ///< Portrait / landscape classification.
    static const uint16_t kEventFaceDownMask = 0x0004;    ///< Face down flag.
    static const uint16_t kEventSingleTap = 0x0008;       ///< Single tap detected.
    static const uint16_t kEventDoubleTap = 0x0010;       ///< Double tap detected.
    static const uint16_t kEventTripleTap = 0x0020;       ///< Triple tap detected.

    ///////////////////////////////////////////////////////////////////////////
    // Feature Configuration Field Masks
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kMotionSlopeThresholdMask = 0x0FFF; ///< Any/no-motion slope threshold.
    static const uint16_t kMotionAccelRefUpMask = 0x1000;     ///< Any/no-motion reference update.
    static const uint16_t kMotionHysteresisMask = 0x03FF;     ///< Any/no-motion hysteresis.
    static const uint16_t kMotionDurationMask = 0x1FFF;       ///< Any/no-motion duration.
    static const uint16_t kMotionWaitTimeMask = 0xE000;       ///< Any/no-motion wait time.
    static const uint8_t kMotionWaitTimeShift = 13;           ///< Wait time field position.

    static const uint16_t kSigP2pMinMask = 0x03FF; ///< Significant motion minimum peak-to-peak.
    static const uint16_t kSigMcrMinMask = 0xFC00; ///< Significant motion minimum crossing rate.
    static const uint8_t kSigMcrMinShift = 10;     ///< Minimum crossing rate field position.
    static const uint16_t kSigP2pMaxMask = 0x03FF; ///< Significant motion maximum peak-to-peak.
    static const uint16_t kSigMcrMaxMask = 0xFC00; ///< Significant motion maximum crossing rate.
    static const uint8_t kSigMcrMaxShift = 10;     ///< Maximum crossing rate field position.

    static const uint16_t kTiltSegmentSizeMask = 0x00FF;  ///< Tilt reference segment size.
    static const uint16_t kTiltMinAngleMask = 0xFF00;     ///< Tilt minimum angle.
    static const uint8_t kTiltMinAngleShift = 8;          ///< Minimum angle field position.

    static const uint16_t kOrientUdEnMask = 0x0001;    ///< Orientation face up / down enable.
    static const uint16_t kOrientModeMask = 0x0006;    ///< Orientation mode.
    static const uint8_t kOrientModeShift = 1;         ///< Orientation mode field position.
    static const uint16_t kOrientBlockingMask = 0x0018;///< Orientation blocking.
    static const uint8_t kOrientBlockingShift = 3;     ///< Blocking field position.
    static const uint16_t kOrientThetaMask = 0x07E0;   ///< Orientation theta.
    static const uint8_t kOrientThetaShift = 5;        ///< Theta field position.
    static const uint16_t kOrientHoldTimeMask = 0xF800;///< Orientation hold time.
    static const uint8_t kOrientHoldTimeShift = 11;    ///< Hold time field position.
    static const uint16_t kOrientSlopeThresMask = 0x00FF; ///< Orientation slope threshold.
    static const uint16_t kOrientHystMask = 0xFF00;    ///< Orientation hysteresis.
    static const uint8_t kOrientHystShift = 8;         ///< Hysteresis field position.

    static const uint16_t kFlatThetaMask = 0x003F;     ///< Flat theta.
    static const uint16_t kFlatBlockingMask = 0x00C0;  ///< Flat blocking.
    static const uint8_t kFlatBlockingShift = 6;       ///< Blocking field position.
    static const uint16_t kFlatHoldTimeMask = 0xFF00;  ///< Flat hold time.
    static const uint8_t kFlatHoldTimeShift = 8;       ///< Hold time field position.
    static const uint16_t kFlatSlopeThresMask = 0x00FF;///< Flat slope threshold.
    static const uint16_t kFlatHystMask = 0xFF00;      ///< Flat hysteresis.
    static const uint8_t kFlatHystShift = 8;           ///< Hysteresis field position.

    static const uint16_t kTapAxisMask = 0x0003;        ///< Tap axis selection.
    static const uint16_t kTapWaitTimeoutMask = 0x0004; ///< Tap wait-for-timeout.
    static const uint16_t kTapMaxPeaksMask = 0x0038;    ///< Tap maximum peaks.
    static const uint8_t kTapMaxPeaksShift = 3;         ///< Maximum peaks field position.
    static const uint16_t kTapModeMask = 0x00C0;        ///< Tap mode.
    static const uint8_t kTapModeShift = 6;             ///< Mode field position.
    static const uint16_t kTapPeakThresMask = 0x03FF;   ///< Tap peak threshold.
    static const uint16_t kTapMaxGestDurMask = 0xFC00;  ///< Tap maximum gesture duration.
    static const uint8_t kTapMaxGestDurShift = 10;      ///< Gesture duration field position.
    static const uint16_t kTapMaxDurBwPeaksMask = 0x000F; ///< Tap maximum duration between peaks.
    static const uint16_t kTapShockSettDurMask = 0x00F0;  ///< Tap shock settling duration.
    static const uint8_t kTapShockSettDurShift = 4;       ///< Shock settling field position.
    static const uint16_t kTapMinQuietDurMask = 0x0F00;   ///< Tap minimum quiet duration.
    static const uint8_t kTapMinQuietDurShift = 8;        ///< Minimum quiet field position.
    static const uint16_t kTapQuietAfterGestMask = 0xF000;///< Tap quiet time after gesture.
    static const uint8_t kTapQuietAfterGestShift = 12;    ///< Quiet after gesture field position.

    static const uint16_t kStepWatermarkMask = 0x03FF;    ///< Step counter watermark.
    static const uint16_t kStepResetCounterMask = 0x0400; ///< Step counter reset bit.

    static const uint16_t kRemapAxisMask = 0x0007;  ///< Axis permutation.
    static const uint16_t kRemapXSignMask = 0x0008; ///< Invert X.
    static const uint16_t kRemapYSignMask = 0x0010; ///< Invert Y.
    static const uint16_t kRemapZSignMask = 0x0020; ///< Invert Z.

    ///////////////////////////////////////////////////////////////////////////
    // Offset Register Masks
    ///////////////////////////////////////////////////////////////////////////

    static const uint16_t kAccelOffsetMask14 = 0x3FFF; ///< Accelerometer offset, 14-bit (rev 1+).
    static const uint16_t kAccelOffsetMask13 = 0x1FFF; ///< Accelerometer offset, 13-bit (rev 0).
    static const uint16_t kGyroOffsetMask = 0x03FF;  ///< Gyroscope offset, 10-bit signed.

    ///////////////////////////////////////////////////////////////////////////
    // Timing and Limits
    ///////////////////////////////////////////////////////////////////////////

    /** @brief Dummy bytes prepended to a read on I2C. */
    static const uint8_t kDummyBytesI2C = 2;
    /** @brief Dummy bytes prepended to a read on SPI. */
    static const uint8_t kDummyBytesSPI = 1;
    /**
     * @brief Largest word count a single readRegisters() / writeRegisters() call handles.
     *
     * @details Fifteen, not sixteen, and the reason is the transport. A read asks the bus for
     * count * 2 + dummy bytes; the Toolkit's Arduino I2C layer splits anything longer than its
     * 32-byte chunk size into separate transactions, and the BMI330 re-inserts its dummy bytes at
     * the start of every read phase. A split burst would therefore return two dummy bytes in the
     * middle of the payload, shifting everything after them by one word - and, because the byte
     * COUNT still comes out right, reporting success. Fifteen words plus two dummy bytes is exactly
     * 32, which also fits the 32-byte Wire buffer on AVR.
     *
     * @note This cap is load-bearing, not slack. readFifoData() packs as many whole frames into a
     * burst as will fit, so an accelerometer-only FIFO reads the full fifteen words at a time.
     * Raising this constant without also raising the bus object's chunk size reintroduces the
     * silent corruption described above.
     */
    static const uint8_t kMaxBurstWords = 15;
    /** @brief Milliseconds to wait after a soft reset before touching the device. */
    static const uint8_t kSoftResetDelayMs = 2;
    /** @brief Milliseconds between feature engine start-up polls. */
    static const uint8_t kFeatureEngineStepMs = 10;
    /** @brief Feature engine start-up polls before giving up. */
    static const uint8_t kFeatureEngineRetries = 20;
    /** @brief Milliseconds between self-test completion polls. */
    static const uint8_t kSelfTestStepMs = 35;
    /** @brief Milliseconds between self-calibration completion polls. */
    static const uint8_t kSelfCalibStepMs = 43;
    /** @brief Completion polls before a self-test or self-calibration is declared timed out. */
    static const uint8_t kSelfTestRetries = 10;
    /** @brief Axis-map acknowledgement polls before giving up. */
    static const uint8_t kAxisMapRetries = 10;
    /** @brief Milliseconds between axis-map acknowledgement polls. */
    static const uint8_t kAxisMapStepMs = 10;
    /** @brief Words in the FIFO buffer (2048 bytes). */
    static const uint16_t kFifoSizeWords = 1024;
};
