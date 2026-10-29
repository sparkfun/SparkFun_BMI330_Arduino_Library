/**
 * @file SparkFun_BMI330.h
 * @brief Arduino-specific implementation for the SparkFun Qwiic BMI330 6-DoF IMU breakout.
 *
 * @details
 * This file provides the Arduino wrappers for the Bosch BMI330. SfeBMI330ArdI2C and
 * SfeBMI330ArdSPI inherit from the platform-independent sfDevBMI330 driver and supply the bus - one
 * over Arduino's Wire library, the other over SPI - through the SparkFun Toolkit. One of these two
 * classes is all a sketch needs.
 *
 * THE BREAKOUT EXPOSES BOTH BUSES. I2C is the default, and the Qwiic connector makes it the path of
 * least resistance:
 *
 *     SfeBMI330ArdI2C imu;
 *     imu.begin();                     // Qwiic, factory address 0x68
 *     imu.begin(SFE_BMI330_ALT_ADDRESS); // ADR jumper closed, 0x69
 *
 * The SPI pins are broken out as well, and the same driver drives them - nothing above the bus
 * layer changes:
 *
 *     SfeBMI330ArdSPI imu;
 *     imu.begin(10);                   // chip select on D10
 *
 * SPI is worth the extra four wires when the application is sampling fast. At 6400 Hz with
 * accelerometer and gyroscope both running, each sample is 16 bytes of payload; 400 kHz I2C leaves
 * very little headroom for anything else on the bus, where a 4 MHz SPI link barely notices. SPI is
 * also the answer to an I2C address collision that the single ADR jumper cannot resolve.
 *
 * BOTH INTERRUPT PINS ARE EXPOSED. INT1 and INT2 are independent, and any source can be routed to
 * either - see mapInterrupt(). Putting data-ready on one pin and motion events on the other lets an
 * ISR tell them apart without a bus transaction, which is the difference between a clean wake and a
 * read that has to happen before the sketch knows whether it needed to.
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

// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevBMI330.h"
#include <Arduino.h>
// clang-format on

/**
 * @brief Default SPI clock for SfeBMI330ArdSPI, in hertz.
 *
 * @details The BMI330 is specified to 10 MHz. The default here is 4 MHz, which is quick enough to
 * move a full 6-axis sample in about 40 us and forgiving of the jumper wires a breakout usually
 * meets first. Pass a faster clock to begin() once the wiring is short.
 */
#define SFE_BMI330_SPI_CLOCK_DEFAULT 4000000

///////////////////////////////////////////////////////////////////////////////
// SfeBMI330ArdI2C
///////////////////////////////////////////////////////////////////////////////

/**
 * @class SfeBMI330ArdI2C
 * @brief Arduino I2C implementation for the SparkFun Qwiic BMI330 breakout.
 *
 * @details
 * Owns an sfTkArdI2C bus object wrapping Arduino's Wire library and hands it to sfDevBMI330. Every
 * measurement, configuration, interrupt, FIFO and feature-engine method is inherited unchanged from
 * sfDevBMI330 - this class adds only begin() and the address helpers.
 *
 * @see sfDevBMI330
 * @see SfeBMI330ArdSPI
 */
class SfeBMI330ArdI2C : public sfDevBMI330
{
  public:
    SfeBMI330ArdI2C()
    {
    }

    /**
     * @brief Initialize the board over I2C.
     *
     * @details Brings up the Toolkit bus, confirms something is answering at @p address, checks the
     * chip ID, soft resets, and starts the feature engine.
     *
     * @note Both sensors are left DISABLED, matching the device's power-on state. A sketch that
     * calls begin() and then reads will get zeros until enableAccel() and enableGyro() - or
     * setAccelConfig() / setGyroConfig() - have run. This is deliberate: the BMI330's power modes
     * and rates are application choices, and guessing them would cost current on a battery design
     * that wanted low-power mode.
     *
     * @param address 7-bit I2C address. Defaults to ::SFE_BMI330_DEFAULT_ADDRESS (0x68); close the
     * board's ADR jumper for ::SFE_BMI330_ALT_ADDRESS (0x69).
     * @param wirePort TwoWire instance to use (default: Wire).
     * @return true if the device was found and initialized.
     */
    bool begin(uint8_t address = SFE_BMI330_DEFAULT_ADDRESS, TwoWire &wirePort = Wire)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        // Confirm something is at this address before reading registers, so a missing board reports
        // a bus failure rather than a chip ID mismatch.
        if (_theI2CBus.ping() != ksfTkErrOk)
            return false;

        return sfDevBMI330::begin(&_theI2CBus) == ksfTkErrOk;
    }

    /**
     * @brief Initialize the board over I2C on an explicit Wire port at the default address.
     *
     * @details Convenience overload for boards with more than one I2C peripheral - an ESP32 with
     * the IMU on Wire1, for instance.
     *
     * @param wirePort TwoWire instance to use.
     * @return true if the device was found and initialized.
     */
    bool begin(TwoWire &wirePort)
    {
        return begin(SFE_BMI330_DEFAULT_ADDRESS, wirePort);
    }

    /**
     * @brief Get the 7-bit I2C address this object is using.
     * @return The configured I2C address.
     */
    uint8_t getAddress(void)
    {
        return _theI2CBus.address();
    }

    /**
     * @brief Check whether a BMI330 answers at either factory address on a given bus.
     *
     * @details A discovery helper for a sketch that does not know how the board's ADR jumper is
     * set. It pings each address and reads the chip ID, so a device that merely answers at 0x68 -
     * plenty of parts do - is not mistaken for a BMI330.
     *
     * @param address Receives the address where a BMI330 was found.
     * @param wirePort TwoWire instance to search (default: Wire).
     * @return true if a BMI330 was found.
     */
    static bool findDevice(uint8_t &address, TwoWire &wirePort = Wire)
    {
        static const uint8_t kAddresses[2] = {SFE_BMI330_DEFAULT_ADDRESS, SFE_BMI330_ALT_ADDRESS};

        for (uint8_t i = 0; i < 2; i++)
        {
            SfeBMI330ArdI2C candidate;

            if (candidate._theI2CBus.init(wirePort, kAddresses[i]) != ksfTkErrOk)
                continue;

            if (candidate._theI2CBus.ping() != ksfTkErrOk)
                continue;

            candidate.setCommunicationBus(&candidate._theI2CBus);

            if (!candidate.isConnected())
                continue;

            address = kAddresses[i];
            return true;
        }

        return false;
    }

  private:
    /** @brief Arduino I2C bus interface used for all communication with the board. */
    sfTkArdI2C _theI2CBus;
};

///////////////////////////////////////////////////////////////////////////////
// SfeBMI330ArdSPI
///////////////////////////////////////////////////////////////////////////////

/**
 * @class SfeBMI330ArdSPI
 * @brief Arduino SPI implementation for the SparkFun Qwiic BMI330 breakout.
 *
 * @details
 * Owns an sfTkArdSPI bus object and hands it to sfDevBMI330. Everything above the bus is identical
 * to the I2C class - a sketch can be moved from one to the other by changing the object's type and
 * its begin() arguments, and nothing else.
 *
 * @note SPI IS SELECTED BY THE FIRST CS EDGE. The BMI330 powers up undecided about its interface
 * and commits to SPI when it first sees CSB go low and back high. That first transfer therefore
 * returns garbage by design, which is why the driver performs and discards one read before checking
 * the chip ID - both in begin() and after every soft reset. A driver without that throwaway read
 * fails to identify a perfectly healthy part.
 *
 * @see sfDevBMI330
 * @see SfeBMI330ArdI2C
 */
class SfeBMI330ArdSPI : public sfDevBMI330
{
  public:
    SfeBMI330ArdSPI()
    {
    }

    /**
     * @brief Initialize the board over SPI.
     *
     * @details Drives the chip select pin high before switching it to an output - the order
     * matters, since an output that starts low would look like the beginning of a transfer to a
     * device that has not yet decided which interface it speaks.
     *
     * @note As with the I2C class, both sensors are left disabled. See SfeBMI330ArdI2C::begin().
     *
     * @param csPin Chip select pin.
     * @param clockFrequency SPI clock in hertz. Defaults to ::SFE_BMI330_SPI_CLOCK_DEFAULT;
     * the device is specified to 10 MHz.
     * @param spiPort SPIClass instance to use (default: SPI).
     * @return true if the device was found and initialized.
     */
    bool begin(uint8_t csPin, uint32_t clockFrequency = SFE_BMI330_SPI_CLOCK_DEFAULT, SPIClass &spiPort = SPI)
    {
        // High first, then output. Reversing these glitches CS low for a moment on most cores.
        digitalWrite(csPin, HIGH);
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);

        // Mode 0. The BMI330 also accepts mode 3, but mode 0 is what the BMI270 library uses and
        // keeps behavior consistent across SparkFun's Bosch IMU boards.
        SPISettings settings(clockFrequency, MSBFIRST, SPI_MODE0);

        if (_theSPIBus.init(spiPort, settings, csPin, true) != ksfTkErrOk)
            return false;

        return sfDevBMI330::begin(&_theSPIBus) == ksfTkErrOk;
    }

    /**
     * @brief Get the chip select pin this object is using.
     * @return The configured chip select pin.
     */
    uint8_t getCSPin(void)
    {
        return _theSPIBus.cs();
    }

  private:
    /** @brief Arduino SPI bus interface used for all communication with the board. */
    sfTkArdSPI _theSPIBus;
};
