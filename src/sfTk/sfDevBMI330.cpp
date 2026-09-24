/**
 * @file sfDevBMI330.cpp
 * @brief Implementation file for the SparkFun Bosch BMI330 6-DoF IMU Driver.
 *
 * @details
 * Implements sfDevBMI330: 16-bit little-endian register access with the device's leading dummy
 * bytes handled, the ACC_CONF / GYR_CONF bit packing, the headerless FIFO parser, the feature
 * engine address window, and the self-test and self-calibration sequences.
 *
 * @author SparkFun Electronics
 * @date September 2026
 * @copyright Copyright (c) 2026, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 *
 * @see https://github.com/sparkfun/SparkFun_BMI330_Arduino_Library
 */

#include "sfDevBMI330.h"

///////////////////////////////////////////////////////////////////////////////
// Gyroscope Filter Coefficients
///////////////////////////////////////////////////////////////////////////////
//
// The BMI330's self-test and self-calibration routines run against a set of gyroscope filter
// coefficients held in feature memory at BMI330_FEAT_ADDR_SC_ST_COEFF. The BMI330 uses a DIFFERENT
// set from the BMI323, and a part seeded with the BMI323 values reports self-test failures while
// being perfectly healthy. primeGyroFilterCoefficients() writes these when what is already there
// does not match.
//
// Source: Bosch BMI3xy SensorAPI v2.4.0, BMI330 build (BMI3_SC_ST_VALUE_0 through _8).
static const uint16_t kGyroFilterCoefficients[9] = {0x5A2E, 0x9219, 0x5637, 0xFFE8, 0xFFEF,
                                                    0x000D, 0x07CA, 0xFFCD, 0xEF6C};

// ========================== Setup and Identity ============================

sfTkError_t sfDevBMI330::begin(sfTkIBus *theBus)
{
    // Adopt the supplied bus if one was given; otherwise keep whatever a prior call set.
    if (theBus != nullptr)
        setCommunicationBus(theBus);

    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    // The device only selects its SPI interface after it has seen a rising edge on CSB, so the very
    // first SPI read comes back as garbage by design. Burn one here rather than failing the chip ID
    // check on a perfectly good board.
    if (_theBus->type() == ksfTkBusTypeSPI)
    {
        uint16_t discard = 0;
        readRegister(kRegChipId, discard);
    }

    uint8_t chipId = 0, revId = 0;
    sfTkError_t rc = getChipID(chipId, revId);
    if (rc != ksfTkErrOk)
        return rc;

    if (chipId != SFE_BMI330_CHIP_ID)
        return ksfTkErrBusNoResponse;

    // The accelerometer user offset field narrowed from 14 bits to 13 on earlier silicon. Pick the
    // width once, here, rather than guessing at every offset access.
    _accelOffsetMask = (revId >= 1) ? kAccelOffsetMask14 : kAccelOffsetMask13;

    // Reset to a known state, bring the feature engine up, and resynchronize the range caches -
    // softReset() does all three. Both sensors are left disabled, which is the power-on state.
    return softReset();
}

void sfDevBMI330::setCommunicationBus(sfTkIBus *theBus)
{
    _theBus = theBus;

    // How many dummy bytes precede read data is a property of the interface, not the register, so
    // settle it once here rather than testing the bus type on every read.
    if (_theBus != nullptr && _theBus->type() == ksfTkBusTypeSPI)
        _dummyBytes = kDummyBytesSPI;
    else
        _dummyBytes = kDummyBytesI2C;
}

bool sfDevBMI330::isConnected(void)
{
    uint8_t chipId = 0, revId = 0;

    if (getChipID(chipId, revId) != ksfTkErrOk)
        return false;

    return chipId == SFE_BMI330_CHIP_ID;
}

sfTkError_t sfDevBMI330::getChipID(uint8_t &chipId, uint8_t &revId)
{
    chipId = 0;
    revId = 0;

    uint16_t value = 0;
    sfTkError_t rc = readRegister(kRegChipId, value);
    if (rc != ksfTkErrOk)
        return rc;

    // The word is [rev_id:15:12][reserved:11:8][chip_id:7:0]. Bosch reads the revision as
    // (highByte & 0xF0) >> 4, which is bits 15:12 - not the low nibble of the high byte.
    chipId = (uint8_t)(value & 0x00FF);
    revId = (uint8_t)((value >> 12) & 0x0F);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::softReset(void)
{
    sfTkError_t rc = sendCommand(kCmdSoftReset);
    if (rc != ksfTkErrOk)
        return rc;

    _pendingInt1 = 0;

    // Datasheet: 1.5 ms from reset to the serial interface being operational.
    sftk_delay_ms(kSoftResetDelayMs);

    // A reset returns the interface to its undecided state, so SPI needs the same throwaway read
    // that begin() performs.
    if (_theBus != nullptr && _theBus->type() == ksfTkBusTypeSPI)
    {
        uint16_t discard = 0;
        readRegister(kRegChipId, discard);
    }

    sfTkError_t engineResult = enableFeatureEngine();
    if (engineResult != ksfTkErrOk)
        return engineResult;

    // A reset returned ACC_CONF and GYR_CONF to their power-on values, so anything the caller had
    // configured - including the full-scale ranges - is gone. Read them back rather than assuming
    // what the reset values are. accelRawToG() and gyroRawToDps() divide by the cached sensitivity,
    // so a cache that disagrees with the device does not produce an error; it quietly scales every
    // reading by a constant factor, which is far harder to notice. Two register reads remove the
    // assumption entirely.
    sfe_bmi330_accel_mode_t accelMode;
    sfe_bmi330_gyro_mode_t gyroMode;
    sfe_bmi330_odr_t odr;
    sfe_bmi330_accel_range_t accelRange;
    sfe_bmi330_gyro_range_t gyroRange;
    sfe_bmi330_bandwidth_t bandwidth;
    sfe_bmi330_averaging_t averaging;

    rc = getAccelConfig(accelMode, odr, accelRange, bandwidth, averaging);
    if (rc != ksfTkErrOk)
        return rc;

    return getGyroConfig(gyroMode, odr, gyroRange, bandwidth, averaging);
}

sfTkError_t sfDevBMI330::enableFeatureEngine(void)
{
    // Three steps, in this order: seed the start-up configuration, commit it, then set the enable
    // bit. Skipping the commit leaves the engine sitting at its defaults and nothing reports an
    // error - it simply never comes up.
    sfTkError_t rc = writeRegister(kRegFeatureIO2, kFeatureStartupConfig);
    if (rc != ksfTkErrOk)
        return rc;

    rc = writeRegister(kRegFeatureIOStatus, kFeatureIOStatusCommit);
    if (rc != ksfTkErrOk)
        return rc;

    rc = writeRegister(kRegFeatureCtrl, kFeatureEngineEnable);
    if (rc != ksfTkErrOk)
        return rc;

    // The engine boots from ROM; poll until it says it is running.
    for (uint8_t i = 0; i < kFeatureEngineRetries; i++)
    {
        sftk_delay_ms(kFeatureEngineStepMs);

        uint16_t status = 0;
        if (readRegister(kRegFeatureIO1, status) != ksfTkErrOk)
            continue;

        // The low nibble is a status CODE, not a set of flags: 0x1 (activated) and 0x5 (no error)
        // mean running. 0x3 (config CRC error), 0x9, 0xB and 0xD also have bit 0 set, so testing
        // bit 0 alone would report a failed start as success.
        uint16_t code = (uint16_t)(status & kFeatureErrorMask);
        if (code == kFeatureEngineEnable || code == kFeatureNoError)
            return ksfTkErrOk;
    }

    return ksfTkErrFail;
}

bool sfDevBMI330::getFeatureEngineStatus(void)
{
    uint16_t status = 0;

    if (readRegister(kRegFeatureIO1, status) != ksfTkErrOk)
        return false;

    uint16_t code = (uint16_t)(status & kFeatureErrorMask);
    return code == kFeatureEngineEnable || code == kFeatureNoError;
}

sfTkError_t sfDevBMI330::getFeatureEngineError(uint16_t &status)
{
    return readRegister(kRegFeatureIO1, status);
}

// ====================== Accelerometer Configuration =======================

sfTkError_t sfDevBMI330::setAccelConfig(sfe_bmi330_accel_mode_t mode, sfe_bmi330_odr_t odr,
                                        sfe_bmi330_accel_range_t range, sfe_bmi330_bandwidth_t bandwidth,
                                        sfe_bmi330_averaging_t averaging)
{
    uint16_t config = 0;

    config |= ((uint16_t)odr & kConfOdrMask);
    config |= (((uint16_t)range << kConfRangeShift) & kConfRangeMask);
    config |= (((uint16_t)bandwidth << kConfBwShift) & kConfBwMask);
    config |= (((uint16_t)averaging << kConfAvgShift) & kConfAvgMask);
    config |= (((uint16_t)mode << kConfModeShift) & kConfModeMask);

    sfTkError_t rc = writeRegister(kRegAccelConf, config);
    if (rc != ksfTkErrOk)
        return rc;

    // The conversion math needs the range, and reading it back per sample would double the bus
    // traffic of every measurement. Cache it.
    _accelRange = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getAccelConfig(sfe_bmi330_accel_mode_t &mode, sfe_bmi330_odr_t &odr,
                                        sfe_bmi330_accel_range_t &range, sfe_bmi330_bandwidth_t &bandwidth,
                                        sfe_bmi330_averaging_t &averaging)
{
    uint16_t config = 0;
    sfTkError_t rc = readRegister(kRegAccelConf, config);
    if (rc != ksfTkErrOk)
        return rc;

    odr = (sfe_bmi330_odr_t)(config & kConfOdrMask);
    range = (sfe_bmi330_accel_range_t)((config & kConfRangeMask) >> kConfRangeShift);
    bandwidth = (sfe_bmi330_bandwidth_t)((config & kConfBwMask) >> kConfBwShift);
    averaging = (sfe_bmi330_averaging_t)((config & kConfAvgMask) >> kConfAvgShift);
    mode = (sfe_bmi330_accel_mode_t)((config & kConfModeMask) >> kConfModeShift);

    // Resync the cache from the device - this is how a sketch picks up a configuration it did not
    // set itself, for example after a warm reset of the host but not the sensor.
    _accelRange = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::enableAccel(bool enable)
{
    return setAccelMode(enable ? BMI330_ACCEL_MODE_HIGH_PERF : BMI330_ACCEL_MODE_DISABLE);
}

sfTkError_t sfDevBMI330::setAccelMode(sfe_bmi330_accel_mode_t mode)
{
    return updateRegister(kRegAccelConf, kConfModeMask, (uint16_t)((uint16_t)mode << kConfModeShift));
}

sfTkError_t sfDevBMI330::setAccelODR(sfe_bmi330_odr_t odr)
{
    return updateRegister(kRegAccelConf, kConfOdrMask, (uint16_t)odr);
}

sfTkError_t sfDevBMI330::setAccelRange(sfe_bmi330_accel_range_t range)
{
    sfTkError_t rc =
        updateRegister(kRegAccelConf, kConfRangeMask, (uint16_t)((uint16_t)range << kConfRangeShift));
    if (rc != ksfTkErrOk)
        return rc;

    _accelRange = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setAccelBandwidth(sfe_bmi330_bandwidth_t bandwidth)
{
    return updateRegister(kRegAccelConf, kConfBwMask, (uint16_t)((uint16_t)bandwidth << kConfBwShift));
}

sfTkError_t sfDevBMI330::setAccelAveraging(sfe_bmi330_averaging_t averaging)
{
    return updateRegister(kRegAccelConf, kConfAvgMask, (uint16_t)((uint16_t)averaging << kConfAvgShift));
}

// ======================= Gyroscope Configuration ==========================

sfTkError_t sfDevBMI330::setGyroConfig(sfe_bmi330_gyro_mode_t mode, sfe_bmi330_odr_t odr,
                                       sfe_bmi330_gyro_range_t range, sfe_bmi330_bandwidth_t bandwidth,
                                       sfe_bmi330_averaging_t averaging)
{
    uint16_t config = 0;

    config |= ((uint16_t)odr & kConfOdrMask);
    config |= (((uint16_t)range << kConfRangeShift) & kConfRangeMask);
    config |= (((uint16_t)bandwidth << kConfBwShift) & kConfBwMask);
    config |= (((uint16_t)averaging << kConfAvgShift) & kConfAvgMask);
    config |= (((uint16_t)mode << kConfModeShift) & kConfModeMask);

    sfTkError_t rc = writeRegister(kRegGyroConf, config);
    if (rc != ksfTkErrOk)
        return rc;

    _gyroRange = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getGyroConfig(sfe_bmi330_gyro_mode_t &mode, sfe_bmi330_odr_t &odr,
                                       sfe_bmi330_gyro_range_t &range, sfe_bmi330_bandwidth_t &bandwidth,
                                       sfe_bmi330_averaging_t &averaging)
{
    uint16_t config = 0;
    sfTkError_t rc = readRegister(kRegGyroConf, config);
    if (rc != ksfTkErrOk)
        return rc;

    odr = (sfe_bmi330_odr_t)(config & kConfOdrMask);
    range = (sfe_bmi330_gyro_range_t)((config & kConfRangeMask) >> kConfRangeShift);
    bandwidth = (sfe_bmi330_bandwidth_t)((config & kConfBwMask) >> kConfBwShift);
    averaging = (sfe_bmi330_averaging_t)((config & kConfAvgMask) >> kConfAvgShift);
    mode = (sfe_bmi330_gyro_mode_t)((config & kConfModeMask) >> kConfModeShift);

    _gyroRange = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::enableGyro(bool enable)
{
    return setGyroMode(enable ? BMI330_GYRO_MODE_HIGH_PERF : BMI330_GYRO_MODE_DISABLE);
}

sfTkError_t sfDevBMI330::setGyroMode(sfe_bmi330_gyro_mode_t mode)
{
    return updateRegister(kRegGyroConf, kConfModeMask, (uint16_t)((uint16_t)mode << kConfModeShift));
}

sfTkError_t sfDevBMI330::setGyroODR(sfe_bmi330_odr_t odr)
{
    return updateRegister(kRegGyroConf, kConfOdrMask, (uint16_t)odr);
}

sfTkError_t sfDevBMI330::setGyroRange(sfe_bmi330_gyro_range_t range)
{
    sfTkError_t rc =
        updateRegister(kRegGyroConf, kConfRangeMask, (uint16_t)((uint16_t)range << kConfRangeShift));
    if (rc != ksfTkErrOk)
        return rc;

    _gyroRange = range;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setGyroBandwidth(sfe_bmi330_bandwidth_t bandwidth)
{
    return updateRegister(kRegGyroConf, kConfBwMask, (uint16_t)((uint16_t)bandwidth << kConfBwShift));
}

sfTkError_t sfDevBMI330::setGyroAveraging(sfe_bmi330_averaging_t averaging)
{
    return updateRegister(kRegGyroConf, kConfAvgMask, (uint16_t)((uint16_t)averaging << kConfAvgShift));
}

// ============================= Measurement ================================

sfTkError_t sfDevBMI330::getSensorData(sfe_bmi330_sensor_data_t &data)
{
    // Nine consecutive words: accel XYZ, gyro XYZ, temperature, sensor time low, sensor time high.
    // One transaction means one instant - reading the axes separately would smear a fast rotation
    // across several samples.
    uint16_t raw[9] = {0};
    sfTkError_t rc = readRegisters(kRegAccelDataX, raw, 9);
    if (rc != ksfTkErrOk)
        return rc;

    data.accelRaw.x = (int16_t)raw[0];
    data.accelRaw.y = (int16_t)raw[1];
    data.accelRaw.z = (int16_t)raw[2];

    data.gyroRaw.x = (int16_t)raw[3];
    data.gyroRaw.y = (int16_t)raw[4];
    data.gyroRaw.z = (int16_t)raw[5];

    data.accel.x = accelRawToG(data.accelRaw.x);
    data.accel.y = accelRawToG(data.accelRaw.y);
    data.accel.z = accelRawToG(data.accelRaw.z);

    data.gyro.x = gyroRawToDps(data.gyroRaw.x);
    data.gyro.y = gyroRawToDps(data.gyroRaw.y);
    data.gyro.z = gyroRawToDps(data.gyroRaw.z);

    data.temperature = rawToCelsius((int16_t)raw[6]);
    data.sensorTime = ((uint32_t)raw[8] << 16) | (uint32_t)raw[7];

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getAccelDataRaw(sfe_bmi330_raw_data_t &data)
{
    uint16_t raw[3] = {0};
    sfTkError_t rc = readRegisters(kRegAccelDataX, raw, 3);
    if (rc != ksfTkErrOk)
        return rc;

    data.x = (int16_t)raw[0];
    data.y = (int16_t)raw[1];
    data.z = (int16_t)raw[2];

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getAccelData(sfe_bmi330_data_t &data)
{
    sfe_bmi330_raw_data_t raw;
    sfTkError_t rc = getAccelDataRaw(raw);
    if (rc != ksfTkErrOk)
        return rc;

    data.x = accelRawToG(raw.x);
    data.y = accelRawToG(raw.y);
    data.z = accelRawToG(raw.z);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getGyroDataRaw(sfe_bmi330_raw_data_t &data)
{
    uint16_t raw[3] = {0};
    sfTkError_t rc = readRegisters(kRegGyroDataX, raw, 3);
    if (rc != ksfTkErrOk)
        return rc;

    data.x = (int16_t)raw[0];
    data.y = (int16_t)raw[1];
    data.z = (int16_t)raw[2];

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getGyroData(sfe_bmi330_data_t &data)
{
    sfe_bmi330_raw_data_t raw;
    sfTkError_t rc = getGyroDataRaw(raw);
    if (rc != ksfTkErrOk)
        return rc;

    data.x = gyroRawToDps(raw.x);
    data.y = gyroRawToDps(raw.y);
    data.z = gyroRawToDps(raw.z);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getTemperature(float &celsius)
{
    celsius = 0.0f;

    uint16_t raw = 0;
    sfTkError_t rc = readRegister(kRegTempData, raw);
    if (rc != ksfTkErrOk)
        return rc;

    celsius = rawToCelsius((int16_t)raw);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getSensorTime(uint32_t &sensorTime)
{
    sensorTime = 0;

    uint16_t raw[2] = {0};
    sfTkError_t rc = readRegisters(kRegSensorTime0, raw, 2);
    if (rc != ksfTkErrOk)
        return rc;

    sensorTime = ((uint32_t)raw[1] << 16) | (uint32_t)raw[0];

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getDataReady(bool &accelReady, bool &gyroReady, bool &tempReady)
{
    accelReady = false;
    gyroReady = false;
    tempReady = false;

    // Clear-on-read, so this single read consumes all three flags.
    uint16_t status = 0;
    sfTkError_t rc = readRegister(kRegStatus, status);
    if (rc != ksfTkErrOk)
        return rc;

    accelReady = (status & kStatusAccelDrdy) != 0;
    gyroReady = (status & kStatusGyroDrdy) != 0;
    tempReady = (status & kStatusTempDrdy) != 0;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getErrorStatus(sfe_bmi330_error_status_t &status)
{
    uint16_t raw = 0;
    sfTkError_t rc = readRegister(kRegErrReg, raw);
    if (rc != ksfTkErrOk)
        return rc;

    status.fatalError = (raw & kErrFatal) != 0;
    status.featEngOverload = (raw & kErrFeatEngOverload) != 0;
    status.featEngWatchdog = (raw & kErrFeatEngWatchdog) != 0;
    status.accelConfError = (raw & kErrAccelConf) != 0;
    status.gyroConfError = (raw & kErrGyroConf) != 0;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getSaturationFlags(sfe_bmi330_saturation_t &flags)
{
    uint16_t raw = 0;
    sfTkError_t rc = readRegister(kRegSatFlags, raw);
    if (rc != ksfTkErrOk)
        return rc;

    flags.accelX = (raw & kSatAccelX) != 0;
    flags.accelY = (raw & kSatAccelY) != 0;
    flags.accelZ = (raw & kSatAccelZ) != 0;
    flags.gyroX = (raw & kSatGyroX) != 0;
    flags.gyroY = (raw & kSatGyroY) != 0;
    flags.gyroZ = (raw & kSatGyroZ) != 0;

    return ksfTkErrOk;
}

// =========================== Unit Conversion ==============================

float sfDevBMI330::accelSensitivity(sfe_bmi330_accel_range_t range)
{
    // 16-bit signed output spans the full range, so sensitivity is 32768 / range in g.
    switch (range)
    {
    case BMI330_ACCEL_RANGE_2G:
        return 16384.0f;
    case BMI330_ACCEL_RANGE_4G:
        return 8192.0f;
    case BMI330_ACCEL_RANGE_8G:
        return 4096.0f;
    case BMI330_ACCEL_RANGE_16G:
        return 2048.0f;
    default:
        return 16384.0f;
    }
}

float sfDevBMI330::gyroSensitivity(sfe_bmi330_gyro_range_t range)
{
    // 32768 / range in dps. These are not whole numbers, which is why they are spelled out.
    switch (range)
    {
    case BMI330_GYRO_RANGE_125DPS:
        return 262.144f;
    case BMI330_GYRO_RANGE_250DPS:
        return 131.072f;
    case BMI330_GYRO_RANGE_500DPS:
        return 65.536f;
    case BMI330_GYRO_RANGE_1000DPS:
        return 32.768f;
    case BMI330_GYRO_RANGE_2000DPS:
        return 16.384f;
    default:
        return 16.384f;
    }
}

float sfDevBMI330::accelRawToG(int16_t raw)
{
    return (float)raw / accelSensitivity(_accelRange);
}

float sfDevBMI330::gyroRawToDps(int16_t raw)
{
    return (float)raw / gyroSensitivity(_gyroRange);
}

float sfDevBMI330::rawToCelsius(int16_t raw)
{
    // BMI330: 256 LSB/K, zero at 23 degrees C. The BMI323 uses 512 here - see the header.
    return ((float)raw / 256.0f) + 23.0f;
}

// ============================== Interrupts ================================

sfTkError_t sfDevBMI330::setInterruptPinConfig(sfe_bmi330_int_pin_t pin,
                                               const sfe_bmi330_int_pin_config_t &config)
{
    uint16_t mask = 0;
    uint16_t value = 0;

    if (pin == BMI330_INT1)
    {
        mask = kInt1LevelMask | kInt1OpenDrainMask | kInt1OutputEnMask;

        if (config.activeHigh)
            value |= kInt1LevelMask;
        if (config.openDrain)
            value |= kInt1OpenDrainMask;
        if (config.enabled)
            value |= kInt1OutputEnMask;
    }
    else if (pin == BMI330_INT2)
    {
        mask = kInt2LevelMask | kInt2OpenDrainMask | kInt2OutputEnMask;

        if (config.activeHigh)
            value |= kInt2LevelMask;
        if (config.openDrain)
            value |= kInt2OpenDrainMask;
        if (config.enabled)
            value |= kInt2OutputEnMask;
    }
    else
    {
        return ksfTkErrInvalidParam;
    }

    // Both pins live in one register, so this has to be read-modify-write or configuring INT2 would
    // silently reset INT1.
    return updateRegister(kRegIoIntCtrl, mask, value);
}

sfTkError_t sfDevBMI330::setInterruptLatch(bool latched)
{
    return updateRegister(kRegIntConf, kIntLatchMask, latched ? kIntLatchMask : 0);
}

sfTkError_t sfDevBMI330::mapInterrupt(sfe_bmi330_int_source_t source, sfe_bmi330_int_pin_t pin)
{
    // Each source owns a 2-bit field. The first eight sources live in INT_MAP1 and the rest in
    // INT_MAP2, with the field position running 0, 2, 4 ... within each register.
    uint8_t reg;
    uint8_t shift;

    if (source < BMI330_INT_SRC_TAP)
    {
        reg = kRegIntMap1;
        shift = (uint8_t)(source * 2);
    }
    else if (source < BMI330_INT_SRC_COUNT)
    {
        reg = kRegIntMap2;
        shift = (uint8_t)((source - BMI330_INT_SRC_TAP) * 2);
    }
    else
    {
        return ksfTkErrInvalidParam;
    }

    uint16_t mask = (uint16_t)(0x0003 << shift);
    uint16_t value = (uint16_t)(((uint16_t)pin & 0x0003) << shift);

    return updateRegister(reg, mask, value);
}

sfTkError_t sfDevBMI330::getInterruptStatus(sfe_bmi330_int_pin_t pin, uint16_t &status)
{
    status = 0;

    if (pin == BMI330_INT1)
    {
        sfTkError_t rc = readRegister(kRegIntStatusInt1, status);
        if (rc != ksfTkErrOk)
            return rc;

        // Hand back anything setAxisRemap() had to consume on the caller's behalf.
        status |= _pendingInt1;
        _pendingInt1 = 0;
        return ksfTkErrOk;
    }

    if (pin == BMI330_INT2)
        return readRegister(kRegIntStatusInt2, status);

    return ksfTkErrInvalidParam;
}

// ================================= FIFO ===================================

uint8_t sfDevBMI330::frameLengthWords(uint16_t sources)
{
    uint8_t words = 0;

    if (sources & BMI330_FIFO_ACCEL)
        words += 3;
    if (sources & BMI330_FIFO_GYRO)
        words += 3;
    if (sources & BMI330_FIFO_TEMP)
        words += 1;
    if (sources & BMI330_FIFO_TIME)
        words += 1;

    return words;
}

sfTkError_t sfDevBMI330::setFifoConfig(uint16_t sources, bool stopOnFull)
{
    uint16_t value = (uint16_t)(sources & kFifoSourceMask);

    if (stopOnFull)
        value |= kFifoStopOnFullMask;

    return updateRegister(kRegFifoConf, (uint16_t)(kFifoSourceMask | kFifoStopOnFullMask), value);
}

sfTkError_t sfDevBMI330::getFifoConfig(uint16_t &sources, bool &stopOnFull)
{
    sources = 0;
    stopOnFull = false;

    uint16_t value = 0;
    sfTkError_t rc = readRegister(kRegFifoConf, value);
    if (rc != ksfTkErrOk)
        return rc;

    sources = (uint16_t)(value & kFifoSourceMask);
    stopOnFull = (value & kFifoStopOnFullMask) != 0;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setFifoWatermark(uint16_t words)
{
    return updateRegister(kRegFifoWatermark, kFifoWatermarkMask, (uint16_t)(words & kFifoWatermarkMask));
}

sfTkError_t sfDevBMI330::getFifoWatermark(uint16_t &words)
{
    words = 0;

    uint16_t value = 0;
    sfTkError_t rc = readRegister(kRegFifoWatermark, value);
    if (rc != ksfTkErrOk)
        return rc;

    words = (uint16_t)(value & kFifoWatermarkMask);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getFifoFillLevel(uint16_t &words)
{
    words = 0;

    uint16_t value = 0;
    sfTkError_t rc = readRegister(kRegFifoFillLevel, value);
    if (rc != ksfTkErrOk)
        return rc;

    words = (uint16_t)(value & kFifoFillLevelMask);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::flushFifo(void)
{
    return writeRegister(kRegFifoCtrl, kFifoFlushMask);
}

sfTkError_t sfDevBMI330::readFifoWords(uint16_t *words, uint8_t count)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    if (words == nullptr || count == 0 || count > kMaxBurstWords)
        return ksfTkErrInvalidParam;

    // FIFO_DATA does not auto-increment - every word of the burst streams from the same address -
    // so this is one transfer rather than a loop of register reads. The dummy bytes are prepended
    // once, at the start of the transfer, not per word.
    //
    // The staging buffer is deliberate: reading straight into the caller's array would need room
    // for the dummy bytes at the front and would overrun it by one or two bytes.
    uint8_t reg = kRegFifoData;
    uint8_t buffer[(kMaxBurstWords * 2) + kDummyBytesI2C] = {0};
    size_t total = (size_t)(count * 2) + _dummyBytes;
    size_t readBytes = 0;

    sfTkError_t rc = _theBus->readRegister(&reg, sizeof(reg), buffer, total, readBytes);
    if (rc != ksfTkErrOk)
        return rc;

    if (readBytes != total)
        return ksfTkErrBusUnderRead;

    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t lsb = buffer[_dummyBytes + (i * 2)];
        uint8_t msb = buffer[_dummyBytes + (i * 2) + 1];
        words[i] = (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::readFifoRaw(uint16_t *words, uint16_t count)
{
    if (words == nullptr || count == 0)
        return ksfTkErrInvalidParam;

    uint16_t remaining = count;
    uint16_t offset = 0;

    while (remaining > 0)
    {
        uint8_t chunk = (remaining > kMaxBurstWords) ? kMaxBurstWords : (uint8_t)remaining;

        sfTkError_t rc = readFifoWords(&words[offset], chunk);
        if (rc != ksfTkErrOk)
            return rc;

        offset += chunk;
        remaining -= chunk;
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::readFifoData(sfe_bmi330_fifo_frame_t *frames, uint8_t maxFrames, uint8_t &numFrames,
                                      uint16_t sources)
{
    numFrames = 0;

    if (frames == nullptr || maxFrames == 0)
        return ksfTkErrInvalidParam;

    uint8_t frameWords = frameLengthWords(sources);
    if (frameWords == 0)
        return ksfTkErrInvalidParam;

    uint16_t fillLevel = 0;
    sfTkError_t rc = getFifoFillLevel(fillLevel);
    if (rc != ksfTkErrOk)
        return rc;

    // Only take whole frames. A partial frame left in the buffer is retransmitted from its start on
    // the next read, so leaving it is free; tearing it in half is not recoverable.
    uint16_t available = fillLevel / frameWords;
    if (available == 0)
        return ksfTkErrOk;

    uint8_t toRead = (available > maxFrames) ? maxFrames : (uint8_t)available;

    // Pull as many WHOLE frames per transaction as the burst buffer holds. Frame-aligned by
    // construction, which matters: the device retransmits any frame that was only partially read
    // out, so a chunk boundary inside a frame would hand back the same samples twice.
    uint8_t framesPerBurst = (uint8_t)(kMaxBurstWords / frameWords);
    if (framesPerBurst == 0)
        framesPerBurst = 1;

    uint16_t words[kMaxBurstWords] = {0};
    uint8_t burstFrames = 0;
    uint8_t burstIndex = 0;

    for (uint8_t f = 0; f < toRead; f++)
    {
        if (burstIndex >= burstFrames)
        {
            uint8_t want = (uint8_t)(toRead - f);
            burstFrames = (want > framesPerBurst) ? framesPerBurst : want;
            burstIndex = 0;

            rc = readFifoWords(words, (uint8_t)(burstFrames * frameWords));
            if (rc != ksfTkErrOk)
                return rc;
        }

        sfe_bmi330_fifo_frame_t &frame = frames[f];
        frame = sfe_bmi330_fifo_frame_t();

        uint8_t idx = (uint8_t)(burstIndex * frameWords);
        burstIndex++;

        // Frame layout is fixed by hardware: accel, gyro, temperature, time - with disabled sources
        // simply absent. The FIFO carries no headers, so this order is the only thing telling the
        // parser what it is looking at.
        if (sources & BMI330_FIFO_ACCEL)
        {
            frame.accel.x = (int16_t)words[idx];
            frame.accel.y = (int16_t)words[idx + 1];
            frame.accel.z = (int16_t)words[idx + 2];
            // A sensor that was idle at capture time writes a sentinel rather than a sample. Report
            // that as "no data" instead of handing back 0x7F01 as an acceleration.
            frame.hasAccel = (words[idx] != kFifoAccelInvalid);
            idx += 3;
        }

        if (sources & BMI330_FIFO_GYRO)
        {
            frame.gyro.x = (int16_t)words[idx];
            frame.gyro.y = (int16_t)words[idx + 1];
            frame.gyro.z = (int16_t)words[idx + 2];
            frame.hasGyro = (words[idx] != kFifoGyroInvalid);
            idx += 3;
        }

        if (sources & BMI330_FIFO_TEMP)
        {
            frame.temperature = (int16_t)words[idx];
            frame.hasTemp = (words[idx] != kFifoTempInvalid);
            idx += 1;
        }

        if (sources & BMI330_FIFO_TIME)
        {
            frame.sensorTime = words[idx];
            frame.hasTime = true;
            idx += 1;
        }

        numFrames++;
    }

    return ksfTkErrOk;
}

// ========================= Feature Engine Access ==========================

sfTkError_t sfDevBMI330::enableFeature(uint16_t features, bool enable)
{
    uint16_t current = 0;
    sfTkError_t rc = readRegister(kRegFeatureIO0, current);
    if (rc != ksfTkErrOk)
        return rc;

    uint16_t updated = enable ? (uint16_t)(current | features) : (uint16_t)(current & ~features);

    // Clear the register before writing the new value, which is what Bosch's own driver does
    // ("reset the register before updating new values"). FEATURE_IO0 is shared with the feature
    // engine, and going straight from one non-zero value to another can leave the engine acting on
    // the old enable set. Two writes instead of one, at startup only.
    rc = writeRegister(kRegFeatureIO0, 0);
    if (rc != ksfTkErrOk)
        return rc;

    rc = writeRegister(kRegFeatureIO0, updated);
    if (rc != ksfTkErrOk)
        return rc;

    // The enable bits do nothing until the host tells the engine to look at them. Without this
    // write the register reads back correctly and the feature stays off - a quiet failure worth
    // knowing about.
    return writeRegister(kRegFeatureIOStatus, kFeatureIOStatusCommit);
}

sfTkError_t sfDevBMI330::getEnabledFeatures(uint16_t &features)
{
    return readRegister(kRegFeatureIO0, features);
}

sfTkError_t sfDevBMI330::readFeatureWords(uint8_t baseAddr, uint16_t *words, uint8_t count)
{
    if (words == nullptr || count == 0 || count > kMaxBurstWords)
        return ksfTkErrInvalidParam;

    // Point the window at the feature, then read consecutive words from the data port, which
    // auto-increments inside extended memory.
    sfTkError_t rc = writeRegister(kRegFeatureDataAddr, (uint16_t)(baseAddr & kFeatureAddrMask));
    if (rc != ksfTkErrOk)
        return rc;

    return readRegisters(kRegFeatureDataTx, words, count);
}

sfTkError_t sfDevBMI330::writeFeatureWords(uint8_t baseAddr, const uint16_t *words, uint8_t count)
{
    if (words == nullptr || count == 0 || count > kMaxBurstWords)
        return ksfTkErrInvalidParam;

    sfTkError_t rc = writeRegister(kRegFeatureDataAddr, (uint16_t)(baseAddr & kFeatureAddrMask));
    if (rc != ksfTkErrOk)
        return rc;

    return writeRegisters(kRegFeatureDataTx, words, count);
}

sfTkError_t sfDevBMI330::updateFeatureWord(uint8_t baseAddr, uint8_t wordIndex, uint16_t mask, uint16_t value)
{
    // Read the whole block up to and including the word of interest, change one field, write it
    // back. Feature blocks hold settings the library does not model, so rewriting a block wholesale
    // from defaults would quietly undo anything the sketch had tuned by hand.
    uint8_t count = (uint8_t)(wordIndex + 1);
    if (count > kMaxBurstWords)
        return ksfTkErrInvalidParam;

    uint16_t words[kMaxBurstWords] = {0};
    sfTkError_t rc = readFeatureWords(baseAddr, words, count);
    if (rc != ksfTkErrOk)
        return rc;

    words[wordIndex] = (uint16_t)((words[wordIndex] & ~mask) | (value & mask));

    return writeFeatureWords(baseAddr, words, count);
}

// ============================ Motion Features =============================

sfTkError_t sfDevBMI330::setMotionConfig(uint8_t baseAddr, const sfe_bmi330_motion_config_t &config)
{
    // Three words, identical layout for any-motion and no-motion:
    //   word 0: slope threshold  [11:0], accel reference update [12]
    //   word 1: hysteresis       [9:0]
    //   word 2: duration         [12:0], wait time [15:13]
    uint16_t words[3] = {0};

    words[0] = (uint16_t)(config.slopeThreshold & kMotionSlopeThresholdMask);
    if (config.accelRefUpdate)
        words[0] |= kMotionAccelRefUpMask;

    words[1] = (uint16_t)(config.hysteresis & kMotionHysteresisMask);

    words[2] = (uint16_t)(config.duration & kMotionDurationMask);
    words[2] |= (uint16_t)(((uint16_t)config.waitTime << kMotionWaitTimeShift) & kMotionWaitTimeMask);

    return writeFeatureWords(baseAddr, words, 3);
}

sfTkError_t sfDevBMI330::getMotionConfig(uint8_t baseAddr, sfe_bmi330_motion_config_t &config)
{
    uint16_t words[3] = {0};
    sfTkError_t rc = readFeatureWords(baseAddr, words, 3);
    if (rc != ksfTkErrOk)
        return rc;

    config.slopeThreshold = (uint16_t)(words[0] & kMotionSlopeThresholdMask);
    config.accelRefUpdate = (words[0] & kMotionAccelRefUpMask) != 0;
    config.hysteresis = (uint16_t)(words[1] & kMotionHysteresisMask);
    config.duration = (uint16_t)(words[2] & kMotionDurationMask);
    config.waitTime = (uint8_t)((words[2] & kMotionWaitTimeMask) >> kMotionWaitTimeShift);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setAnyMotionConfig(const sfe_bmi330_motion_config_t &config)
{
    return setMotionConfig(BMI330_FEAT_ADDR_ANY_MOTION, config);
}

sfTkError_t sfDevBMI330::getAnyMotionConfig(sfe_bmi330_motion_config_t &config)
{
    return getMotionConfig(BMI330_FEAT_ADDR_ANY_MOTION, config);
}

sfTkError_t sfDevBMI330::setNoMotionConfig(const sfe_bmi330_motion_config_t &config)
{
    return setMotionConfig(BMI330_FEAT_ADDR_NO_MOTION, config);
}

sfTkError_t sfDevBMI330::getNoMotionConfig(sfe_bmi330_motion_config_t &config)
{
    return getMotionConfig(BMI330_FEAT_ADDR_NO_MOTION, config);
}

sfTkError_t sfDevBMI330::setSigMotionConfig(const sfe_bmi330_sig_motion_config_t &config)
{
    // Three words:
    //   word 0: block size      [15:0]
    //   word 1: p2p min [9:0], mean crossing rate min [15:10]
    //   word 2: p2p max [9:0], mean crossing rate max [15:10]
    uint16_t words[3] = {0};

    words[0] = config.blockSize;

    words[1] = (uint16_t)(config.peakToPeakMin & kSigP2pMinMask);
    words[1] |= (uint16_t)(((uint16_t)config.mcrMin << kSigMcrMinShift) & kSigMcrMinMask);

    words[2] = (uint16_t)(config.peakToPeakMax & kSigP2pMaxMask);
    words[2] |= (uint16_t)(((uint16_t)config.mcrMax << kSigMcrMaxShift) & kSigMcrMaxMask);

    return writeFeatureWords(BMI330_FEAT_ADDR_SIG_MOTION, words, 3);
}

sfTkError_t sfDevBMI330::getSigMotionConfig(sfe_bmi330_sig_motion_config_t &config)
{
    uint16_t words[3] = {0};
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_SIG_MOTION, words, 3);
    if (rc != ksfTkErrOk)
        return rc;

    config.blockSize = words[0];
    config.peakToPeakMin = (uint16_t)(words[1] & kSigP2pMinMask);
    config.mcrMin = (uint8_t)((words[1] & kSigMcrMinMask) >> kSigMcrMinShift);
    config.peakToPeakMax = (uint16_t)(words[2] & kSigP2pMaxMask);
    config.mcrMax = (uint8_t)((words[2] & kSigMcrMaxMask) >> kSigMcrMaxShift);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setTiltConfig(const sfe_bmi330_tilt_config_t &config)
{
    // Two words:
    //   word 0: segment size [7:0], minimum tilt angle [15:8]
    //   word 1: beta accel mean [15:0]
    uint16_t words[2] = {0};

    words[0] = (uint16_t)(config.segmentSize & kTiltSegmentSizeMask);
    words[0] |= (uint16_t)(((uint16_t)config.minTiltAngle << kTiltMinAngleShift) & kTiltMinAngleMask);

    words[1] = config.betaAccelMean;

    return writeFeatureWords(BMI330_FEAT_ADDR_TILT, words, 2);
}

sfTkError_t sfDevBMI330::getTiltConfig(sfe_bmi330_tilt_config_t &config)
{
    uint16_t words[2] = {0};
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_TILT, words, 2);
    if (rc != ksfTkErrOk)
        return rc;

    config.segmentSize = (uint8_t)(words[0] & kTiltSegmentSizeMask);
    config.minTiltAngle = (uint8_t)((words[0] & kTiltMinAngleMask) >> kTiltMinAngleShift);
    config.betaAccelMean = words[1];

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setOrientationConfig(const sfe_bmi330_orientation_config_t &config)
{
    // Two words:
    //   word 0: ud_en [0], mode [2:1], blocking [4:3], theta [10:5], hold time [15:11]
    //   word 1: slope threshold [7:0], hysteresis [15:8]
    uint16_t words[2] = {0};

    if (config.faceUpDownEnabled)
        words[0] |= kOrientUdEnMask;
    words[0] |= (uint16_t)(((uint16_t)config.mode << kOrientModeShift) & kOrientModeMask);
    words[0] |= (uint16_t)(((uint16_t)config.blocking << kOrientBlockingShift) & kOrientBlockingMask);
    words[0] |= (uint16_t)(((uint16_t)config.theta << kOrientThetaShift) & kOrientThetaMask);
    words[0] |= (uint16_t)(((uint16_t)config.holdTime << kOrientHoldTimeShift) & kOrientHoldTimeMask);

    words[1] = (uint16_t)(config.slopeThreshold & kOrientSlopeThresMask);
    words[1] |= (uint16_t)(((uint16_t)config.hysteresis << kOrientHystShift) & kOrientHystMask);

    return writeFeatureWords(BMI330_FEAT_ADDR_ORIENTATION, words, 2);
}

sfTkError_t sfDevBMI330::getOrientationConfig(sfe_bmi330_orientation_config_t &config)
{
    uint16_t words[2] = {0};
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_ORIENTATION, words, 2);
    if (rc != ksfTkErrOk)
        return rc;

    config.faceUpDownEnabled = (words[0] & kOrientUdEnMask) != 0;
    config.mode = (uint8_t)((words[0] & kOrientModeMask) >> kOrientModeShift);
    config.blocking = (uint8_t)((words[0] & kOrientBlockingMask) >> kOrientBlockingShift);
    config.theta = (uint8_t)((words[0] & kOrientThetaMask) >> kOrientThetaShift);
    config.holdTime = (uint8_t)((words[0] & kOrientHoldTimeMask) >> kOrientHoldTimeShift);
    config.slopeThreshold = (uint8_t)(words[1] & kOrientSlopeThresMask);
    config.hysteresis = (uint8_t)((words[1] & kOrientHystMask) >> kOrientHystShift);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getOrientation(sfe_bmi330_orientation_t &orientation, bool &faceDown)
{
    orientation = BMI330_ORIENT_PORTRAIT_UPRIGHT;
    faceDown = false;

    uint16_t event = 0;
    sfTkError_t rc = readRegister(kRegFeatureEventExt, event);
    if (rc != ksfTkErrOk)
        return rc;

    orientation = (sfe_bmi330_orientation_t)(event & kEventOrientationMask);
    faceDown = (event & kEventFaceDownMask) != 0;

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setFlatConfig(const sfe_bmi330_flat_config_t &config)
{
    // Two words:
    //   word 0: theta [5:0], blocking [7:6], hold time [15:8]
    //   word 1: slope threshold [7:0], hysteresis [15:8]
    uint16_t words[2] = {0};

    words[0] = (uint16_t)(config.theta & kFlatThetaMask);
    words[0] |= (uint16_t)(((uint16_t)config.blocking << kFlatBlockingShift) & kFlatBlockingMask);
    words[0] |= (uint16_t)(((uint16_t)config.holdTime << kFlatHoldTimeShift) & kFlatHoldTimeMask);

    words[1] = (uint16_t)(config.slopeThreshold & kFlatSlopeThresMask);
    words[1] |= (uint16_t)(((uint16_t)config.hysteresis << kFlatHystShift) & kFlatHystMask);

    return writeFeatureWords(BMI330_FEAT_ADDR_FLAT, words, 2);
}

sfTkError_t sfDevBMI330::getFlatConfig(sfe_bmi330_flat_config_t &config)
{
    uint16_t words[2] = {0};
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_FLAT, words, 2);
    if (rc != ksfTkErrOk)
        return rc;

    config.theta = (uint8_t)(words[0] & kFlatThetaMask);
    config.blocking = (uint8_t)((words[0] & kFlatBlockingMask) >> kFlatBlockingShift);
    config.holdTime = (uint8_t)((words[0] & kFlatHoldTimeMask) >> kFlatHoldTimeShift);
    config.slopeThreshold = (uint8_t)(words[1] & kFlatSlopeThresMask);
    config.hysteresis = (uint8_t)((words[1] & kFlatHystMask) >> kFlatHystShift);

    return ksfTkErrOk;
}

// ============================ Tap Detection ===============================

sfTkError_t sfDevBMI330::setTapConfig(const sfe_bmi330_tap_config_t &config)
{
    // Three words:
    //   word 0: axis [1:0], wait for timeout [2], max peaks [5:3], mode [7:6]
    //   word 1: peak threshold [9:0], max gesture duration [15:10]
    //   word 2: max duration between peaks [3:0], shock settling [7:4],
    //           min quiet between taps [11:8], quiet after gesture [15:12]
    uint16_t words[3] = {0};

    words[0] = (uint16_t)((uint16_t)config.axis & kTapAxisMask);
    if (config.waitForTimeout)
        words[0] |= kTapWaitTimeoutMask;
    words[0] |= (uint16_t)(((uint16_t)config.maxPeaksForTap << kTapMaxPeaksShift) & kTapMaxPeaksMask);
    words[0] |= (uint16_t)(((uint16_t)config.mode << kTapModeShift) & kTapModeMask);

    words[1] = (uint16_t)(config.peakThreshold & kTapPeakThresMask);
    words[1] |= (uint16_t)(((uint16_t)config.maxGestureDuration << kTapMaxGestDurShift) & kTapMaxGestDurMask);

    words[2] = (uint16_t)(config.maxDurationBetweenPeaks & kTapMaxDurBwPeaksMask);
    words[2] |=
        (uint16_t)(((uint16_t)config.shockSettlingDuration << kTapShockSettDurShift) & kTapShockSettDurMask);
    words[2] |= (uint16_t)(((uint16_t)config.minQuietBetweenTaps << kTapMinQuietDurShift) & kTapMinQuietDurMask);
    words[2] |=
        (uint16_t)(((uint16_t)config.quietTimeAfterGesture << kTapQuietAfterGestShift) & kTapQuietAfterGestMask);

    return writeFeatureWords(BMI330_FEAT_ADDR_TAP, words, 3);
}

sfTkError_t sfDevBMI330::getTapConfig(sfe_bmi330_tap_config_t &config)
{
    uint16_t words[3] = {0};
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_TAP, words, 3);
    if (rc != ksfTkErrOk)
        return rc;

    config.axis = (sfe_bmi330_tap_axis_t)(words[0] & kTapAxisMask);
    config.waitForTimeout = (words[0] & kTapWaitTimeoutMask) != 0;
    config.maxPeaksForTap = (uint8_t)((words[0] & kTapMaxPeaksMask) >> kTapMaxPeaksShift);
    config.mode = (uint8_t)((words[0] & kTapModeMask) >> kTapModeShift);

    config.peakThreshold = (uint16_t)(words[1] & kTapPeakThresMask);
    config.maxGestureDuration = (uint8_t)((words[1] & kTapMaxGestDurMask) >> kTapMaxGestDurShift);

    config.maxDurationBetweenPeaks = (uint8_t)(words[2] & kTapMaxDurBwPeaksMask);
    config.shockSettlingDuration = (uint8_t)((words[2] & kTapShockSettDurMask) >> kTapShockSettDurShift);
    config.minQuietBetweenTaps = (uint8_t)((words[2] & kTapMinQuietDurMask) >> kTapMinQuietDurShift);
    config.quietTimeAfterGesture = (uint8_t)((words[2] & kTapQuietAfterGestMask) >> kTapQuietAfterGestShift);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::getTapEvent(sfe_bmi330_tap_event_t &event)
{
    event = sfe_bmi330_tap_event_t();

    uint16_t raw = 0;
    sfTkError_t rc = readRegister(kRegFeatureEventExt, raw);
    if (rc != ksfTkErrOk)
        return rc;

    event.singleTap = (raw & kEventSingleTap) != 0;
    event.doubleTap = (raw & kEventDoubleTap) != 0;
    event.tripleTap = (raw & kEventTripleTap) != 0;

    return ksfTkErrOk;
}

// ============================= Step Counter ===============================

sfTkError_t sfDevBMI330::getStepCount(uint32_t &steps)
{
    steps = 0;

    // FEATURE_IO2 and FEATURE_IO3 are adjacent, so one two-word burst gets both halves of the
    // counter from the same instant. Two separate reads could straddle a carry and report a count
    // that never existed.
    uint16_t words[2] = {0};
    sfTkError_t rc = readRegisters(kRegFeatureIO2, words, 2);
    if (rc != ksfTkErrOk)
        return rc;

    steps = ((uint32_t)words[1] << 16) | (uint32_t)words[0];

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::resetStepCount(void)
{
    // Pulse the reset bit: set it, then clear it. Leaving it set would hold the counter at zero.
    sfTkError_t rc =
        updateFeatureWord(BMI330_FEAT_ADDR_STEP_COUNTER, 0, kStepResetCounterMask, kStepResetCounterMask);
    if (rc != ksfTkErrOk)
        return rc;

    // The feature engine runs at 50 Hz. Hold the bit for longer than one 20 ms cycle so the engine
    // is guaranteed to see it before it is cleared.
    sftk_delay_ms(25);

    return updateFeatureWord(BMI330_FEAT_ADDR_STEP_COUNTER, 0, kStepResetCounterMask, 0);
}

sfTkError_t sfDevBMI330::setStepCounterWatermark(uint16_t watermark)
{
    return updateFeatureWord(BMI330_FEAT_ADDR_STEP_COUNTER, 0, kStepWatermarkMask,
                             (uint16_t)(watermark & kStepWatermarkMask));
}

sfTkError_t sfDevBMI330::getStepCounterWatermark(uint16_t &watermark)
{
    watermark = 0;

    uint16_t word = 0;
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_STEP_COUNTER, &word, 1);
    if (rc != ksfTkErrOk)
        return rc;

    watermark = (uint16_t)(word & kStepWatermarkMask);

    return ksfTkErrOk;
}

// ============================ Axis Remapping ==============================

sfTkError_t sfDevBMI330::setAxisRemap(const sfe_bmi330_axis_remap_t &remap)
{
    uint16_t word = (uint16_t)((uint16_t)remap.map & kRemapAxisMask);

    if (remap.invertX)
        word |= kRemapXSignMask;
    if (remap.invertY)
        word |= kRemapYSignMask;
    if (remap.invertZ)
        word |= kRemapZSignMask;

    sfTkError_t rc = writeFeatureWords(BMI330_FEAT_ADDR_AXIS_REMAP, &word, 1);
    if (rc != ksfTkErrOk)
        return rc;

    // The device will only accept an axis-map update while the ACCELEROMETER IS SUSPENDED. Find out
    // what mode it is in so it can be put back afterwards.
    uint16_t accelConf = 0;
    rc = readRegister(kRegAccelConf, accelConf);
    if (rc != ksfTkErrOk)
        return rc;

    uint16_t savedMode = (uint16_t)(accelConf & kConfModeMask);

    // Accelerometer already off: issue the command and stop. There is no acknowledgement to wait
    // for on this path - the completion flag is only raised by the suspend-and-restore sequence.
    if (savedMode == 0)
        return sendCommand(kCmdAxisMapUpdate);

    // INT_STATUS_INT1 is clear-on-read, and the commit below has to read it to remove the flag the
    // update leaves behind. Capture what is genuinely pending first, so getInterruptStatus() can
    // still report it. Nothing new can arrive while the accelerometer is suspended.
    uint16_t pending = 0;
    if (readRegister(kRegIntStatusInt1, pending) == ksfTkErrOk)
        _pendingInt1 |= pending;

    rc = updateRegister(kRegAccelConf, kConfModeMask, 0);
    if (rc != ksfTkErrOk)
        return rc;

    rc = sendCommand(kCmdAxisMapUpdate);
    if (rc != ksfTkErrOk)
    {
        // Put the sensor back before reporting the failure - leaving a caller's accelerometer
        // suspended because a command write failed would be a nasty surprise.
        updateRegister(kRegAccelConf, kConfModeMask, savedMode);
        return rc;
    }

    bool acknowledged = false;

    for (uint8_t i = 0; i < kAxisMapRetries; i++)
    {
        sftk_delay_ms(kAxisMapStepMs);

        uint16_t status = 0;
        if (readRegister(kRegFeatureIO1, status) != ksfTkErrOk)
            continue;

        if ((status & kAxisMapComplete) && ((status & kFeatureErrorMask) == kFeatureNoError))
        {
            acknowledged = true;
            break;
        }
    }

    // The update leaves a flag behind in the INT1 status register. Clear it by reading, before the
    // accelerometer comes back, so the caller's next getInterruptStatus() is not answering for an
    // event it did not ask about. Real events were captured into _pendingInt1 above.
    uint16_t intStatus = 0;
    readRegister(kRegIntStatusInt1, intStatus);

    rc = updateRegister(kRegAccelConf, kConfModeMask, savedMode);
    if (rc != ksfTkErrOk)
        return rc;

    return acknowledged ? ksfTkErrOk : ksfTkErrFail;
}

sfTkError_t sfDevBMI330::getAxisRemap(sfe_bmi330_axis_remap_t &remap)
{
    uint16_t word = 0;
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_AXIS_REMAP, &word, 1);
    if (rc != ksfTkErrOk)
        return rc;

    remap.map = (sfe_bmi330_axis_map_t)(word & kRemapAxisMask);
    remap.invertX = (word & kRemapXSignMask) != 0;
    remap.invertY = (word & kRemapYSignMask) != 0;
    remap.invertZ = (word & kRemapZSignMask) != 0;

    return ksfTkErrOk;
}

// ==================== Self-Test and Self-Calibration ======================

sfTkError_t sfDevBMI330::primeGyroFilterCoefficients(void)
{
    uint16_t current[9] = {0};
    sfTkError_t rc = readFeatureWords(BMI330_FEAT_ADDR_SC_ST_COEFF, current, 9);
    if (rc != ksfTkErrOk)
        return rc;

    bool matches = true;
    for (uint8_t i = 0; i < 9; i++)
    {
        if (current[i] != kGyroFilterCoefficients[i])
        {
            matches = false;
            break;
        }
    }

    if (matches)
        return ksfTkErrOk;

    return writeFeatureWords(BMI330_FEAT_ADDR_SC_ST_COEFF, kGyroFilterCoefficients, 9);
}

sfTkError_t sfDevBMI330::waitForSelfTestComplete(uint16_t &featureIO1, uint16_t stepDelayMs)
{
    featureIO1 = 0;

    for (uint8_t i = 0; i < kSelfTestRetries; i++)
    {
        sftk_delay_ms(stepDelayMs);

        if (readRegister(kRegFeatureIO1, featureIO1) != ksfTkErrOk)
            continue;

        if (featureIO1 & kScStComplete)
            return ksfTkErrOk;
    }

    return ksfTkErrBusTimeout;
}

sfTkError_t sfDevBMI330::performSelfTest(sfe_bmi330_self_test_t selection,
                                         sfe_bmi330_self_test_result_t &result)
{
    result = sfe_bmi330_self_test_result_t();

    // The BMI330 carries its own gyroscope filter coefficients, distinct from the BMI323's. Without
    // them the routine runs and reports failures on a healthy part.
    sfTkError_t rc = primeGyroFilterCoefficients();
    if (rc != ksfTkErrOk)
        return rc;

    // Tell the engine which sensors to exercise.
    uint16_t selectWord = (uint16_t)selection;
    rc = writeFeatureWords(BMI330_FEAT_ADDR_ST_SELECT, &selectWord, 1);
    if (rc != ksfTkErrOk)
        return rc;

    // Remember the caller's accelerometer configuration - the preconditions overwrite it.
    sfe_bmi330_accel_mode_t savedMode;
    sfe_bmi330_odr_t savedOdr;
    sfe_bmi330_accel_range_t savedRange;
    sfe_bmi330_bandwidth_t savedBw;
    sfe_bmi330_averaging_t savedAvg;

    rc = getAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);
    if (rc != ksfTkErrOk)
        return rc;

    // Testing the gyroscope requires the accelerometer running at 100 Hz in high-performance mode.
    // An accelerometer-only test has no such precondition.
    if (selection != BMI330_SELF_TEST_ACCEL)
    {
        rc = setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, savedRange, savedBw, savedAvg);
        if (rc != ksfTkErrOk)
            return rc;
    }

    // Alternate configurations would fight the self-test for control of the sensors. From here on
    // the caller's accelerometer configuration has been changed, so every exit path restores it.
    rc = updateRegister(kRegAltAccelConf, kConfModeMask, 0);
    if (rc == ksfTkErrOk)
        rc = updateRegister(kRegAltGyroConf, kConfModeMask, 0);

    if (rc == ksfTkErrOk)
        rc = sendCommand(kCmdSelfTestTrigger);

    if (rc != ksfTkErrOk)
    {
        setAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);
        return rc;
    }

    uint16_t featureIO1 = 0;
    sfTkError_t waitResult = waitForSelfTestComplete(featureIO1, kSelfTestStepMs);

    result.errorStatus = (uint8_t)(featureIO1 & kFeatureErrorMask);
    result.completed = (waitResult == ksfTkErrOk);

    if (result.completed)
    {
        result.passed = (featureIO1 & kStResult) != 0;

        // Per-axis detail lives in feature memory, not in FEATURE_IO1.
        uint16_t stResult = 0;
        if (readFeatureWords(BMI330_FEAT_ADDR_ST_RESULT, &stResult, 1) == ksfTkErrOk)
        {
            // Only report the axes this run actually exercised. The bits belonging to a sensor that
            // was not tested hold whatever the last run left there, so copying them out
            // unconditionally makes a healthy part look like it failed - or, worse, makes a failed
            // one look healthy on the strength of a previous test.
            if (selection & BMI330_SELF_TEST_ACCEL)
            {
                result.accelXOk = (stResult & 0x0001) != 0;
                result.accelYOk = (stResult & 0x0002) != 0;
                result.accelZOk = (stResult & 0x0004) != 0;
            }

            if (selection & BMI330_SELF_TEST_GYRO)
            {
                result.gyroXOk = (stResult & 0x0008) != 0;
                result.gyroYOk = (stResult & 0x0010) != 0;
                result.gyroZOk = (stResult & 0x0020) != 0;
                result.gyroDriveOk = (stResult & 0x0040) != 0;
            }
        }
    }

    // Put the accelerometer back the way the caller had it, whatever the verdict.
    setAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);

    return waitResult;
}

sfTkError_t sfDevBMI330::performGyroSelfCalibration(sfe_bmi330_self_calib_t selection, bool applyCorrection,
                                                    sfe_bmi330_self_calib_result_t &result)
{
    result = sfe_bmi330_self_calib_result_t();

    sfe_bmi330_accel_mode_t savedMode;
    sfe_bmi330_odr_t savedOdr;
    sfe_bmi330_accel_range_t savedRange;
    sfe_bmi330_bandwidth_t savedBw;
    sfe_bmi330_averaging_t savedAvg;

    sfTkError_t rc = getAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);
    if (rc != ksfTkErrOk)
        return rc;

    // Self-calibration needs the BMI330's own gyro filter coefficients in feature memory, exactly
    // as self-test does. After a soft reset they are not guaranteed to be there.
    rc = primeGyroFilterCoefficients();
    if (rc != ksfTkErrOk)
        return rc;

    // Fixed preconditions: high-performance mode, 100 Hz, +/-8 g. The routine's thresholds are
    // calibrated against exactly this configuration.
    rc = setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_8G, savedBw,
                        savedAvg);
    if (rc != ksfTkErrOk)
        return rc;

    rc = updateRegister(kRegAltAccelConf, kConfModeMask, 0);
    if (rc == ksfTkErrOk)
        rc = updateRegister(kRegAltGyroConf, kConfModeMask, 0);
    if (rc != ksfTkErrOk)
    {
        setAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);
        return rc;
    }

    // Bit 2 of the selection word asks the device to apply the correction it finds.
    uint16_t selectWord = (uint16_t)selection;
    if (applyCorrection)
        selectWord |= 0x0004;

    rc = writeFeatureWords(BMI330_FEAT_ADDR_SC_SELECT, &selectWord, 1);
    if (rc != ksfTkErrOk)
    {
        setAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);
        return rc;
    }

    rc = sendCommand(kCmdSelfCalibTrigger);
    if (rc != ksfTkErrOk)
    {
        setAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);
        return rc;
    }

    uint16_t featureIO1 = 0;
    sfTkError_t waitResult = waitForSelfTestComplete(featureIO1, kSelfCalibStepMs);

    result.errorStatus = (uint8_t)(featureIO1 & kFeatureErrorMask);
    result.completed = (waitResult == ksfTkErrOk);
    if (result.completed)
        result.passed = (featureIO1 & kScResult) != 0;

    setAccelConfig(savedMode, savedOdr, savedRange, savedBw, savedAvg);

    return waitResult;
}

// ============================= User Offsets ===============================

sfTkError_t sfDevBMI330::setAccelOffset(int16_t x, int16_t y, int16_t z)
{
    // The three offsets are interleaved with their gain registers - offset X, gain X, offset Y,
    // gain Y, ... - so they cannot be written as one burst without clobbering the gains.
    sfTkError_t rc = writeRegister(kRegAccelDpOffX, (uint16_t)(x & _accelOffsetMask));
    if (rc != ksfTkErrOk)
        return rc;

    rc = writeRegister((uint8_t)(kRegAccelDpOffX + 2), (uint16_t)(y & _accelOffsetMask));
    if (rc != ksfTkErrOk)
        return rc;

    return writeRegister((uint8_t)(kRegAccelDpOffX + 4), (uint16_t)(z & _accelOffsetMask));
}

sfTkError_t sfDevBMI330::getAccelOffset(int16_t &x, int16_t &y, int16_t &z)
{
    // Read offsets and gains together, then pick out the offsets. One transaction instead of three.
    uint16_t words[5] = {0};
    sfTkError_t rc = readRegisters(kRegAccelDpOffX, words, 5);
    if (rc != ksfTkErrOk)
        return rc;

    // 14 bits on revision 1 and later, 13 on earlier silicon - begin() picked the mask.
    uint8_t bits = (_accelOffsetMask == kAccelOffsetMask14) ? 14 : 13;

    x = signExtend((uint16_t)(words[0] & _accelOffsetMask), bits);
    y = signExtend((uint16_t)(words[2] & _accelOffsetMask), bits);
    z = signExtend((uint16_t)(words[4] & _accelOffsetMask), bits);

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::setGyroOffset(int16_t x, int16_t y, int16_t z)
{
    sfTkError_t rc = writeRegister(kRegGyroDpOffX, (uint16_t)(x & kGyroOffsetMask));
    if (rc != ksfTkErrOk)
        return rc;

    rc = writeRegister((uint8_t)(kRegGyroDpOffX + 2), (uint16_t)(y & kGyroOffsetMask));
    if (rc != ksfTkErrOk)
        return rc;

    return writeRegister((uint8_t)(kRegGyroDpOffX + 4), (uint16_t)(z & kGyroOffsetMask));
}

sfTkError_t sfDevBMI330::getGyroOffset(int16_t &x, int16_t &y, int16_t &z)
{
    uint16_t words[5] = {0};
    sfTkError_t rc = readRegisters(kRegGyroDpOffX, words, 5);
    if (rc != ksfTkErrOk)
        return rc;

    x = signExtend((uint16_t)(words[0] & kGyroOffsetMask), 10);
    y = signExtend((uint16_t)(words[2] & kGyroOffsetMask), 10);
    z = signExtend((uint16_t)(words[4] & kGyroOffsetMask), 10);

    return ksfTkErrOk;
}

int16_t sfDevBMI330::signExtend(uint16_t value, uint8_t bits)
{
    // The offsets are narrower than the register that holds them, so the sign bit sits in the
    // middle of the word. Copy it upward rather than letting the value read back as a large
    // positive number.
    uint16_t signBit = (uint16_t)(1u << (bits - 1));

    if (value & signBit)
        return (int16_t)(value | (uint16_t)~(uint16_t)((1u << bits) - 1u));

    return (int16_t)value;
}

// ========================= Low-Level Register Access ======================

sfTkError_t sfDevBMI330::readRegisters(uint8_t reg, uint16_t *values, uint8_t count)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    if (values == nullptr || count == 0 || count > kMaxBurstWords)
        return ksfTkErrInvalidParam;

    // Room for the device's leading dummy bytes plus the payload. Assembling the words by hand
    // keeps the result independent of the bus object's byte-order setting and of the host's
    // endianness.
    uint8_t buffer[(kMaxBurstWords * 2) + kDummyBytesI2C] = {0};
    size_t total = (size_t)(count * 2) + _dummyBytes;
    size_t readBytes = 0;

    sfTkError_t rc = _theBus->readRegister(&reg, sizeof(reg), buffer, total, readBytes);
    if (rc != ksfTkErrOk)
        return rc;

    if (readBytes != total)
        return ksfTkErrBusUnderRead;

    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t lsb = buffer[_dummyBytes + (i * 2)];
        uint8_t msb = buffer[_dummyBytes + (i * 2) + 1];
        values[i] = (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
    }

    return ksfTkErrOk;
}

sfTkError_t sfDevBMI330::writeRegisters(uint8_t reg, const uint16_t *values, uint8_t count)
{
    if (_theBus == nullptr)
        return ksfTkErrBusNotInit;

    if (values == nullptr || count == 0 || count > kMaxBurstWords)
        return ksfTkErrInvalidParam;

    // Writes carry no dummy bytes - only reads do.
    uint8_t buffer[kMaxBurstWords * 2] = {0};

    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i * 2] = (uint8_t)(values[i] & 0x00FF);
        buffer[(i * 2) + 1] = (uint8_t)((values[i] >> 8) & 0x00FF);
    }

    return _theBus->writeRegister(&reg, sizeof(reg), buffer, (size_t)(count * 2));
}

sfTkError_t sfDevBMI330::readRegister(uint8_t reg, uint16_t &value)
{
    return readRegisters(reg, &value, 1);
}

sfTkError_t sfDevBMI330::writeRegister(uint8_t reg, uint16_t value)
{
    return writeRegisters(reg, &value, 1);
}

sfTkError_t sfDevBMI330::updateRegister(uint8_t reg, uint16_t mask, uint16_t value)
{
    uint16_t current = 0;
    sfTkError_t rc = readRegister(reg, current);
    if (rc != ksfTkErrOk)
        return rc;

    uint16_t updated = (uint16_t)((current & ~mask) | (value & mask));

    // Skip the write when nothing changes. Saves a transaction on the many setters that get called
    // repeatedly with the same value.
    if (updated == current)
        return ksfTkErrOk;

    return writeRegister(reg, updated);
}

sfTkError_t sfDevBMI330::sendCommand(uint16_t command)
{
    return writeRegister(kRegCmd, command);
}
