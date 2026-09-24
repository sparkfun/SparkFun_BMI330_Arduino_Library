/*
  Example 02 - Basic Readings (SPI)

  The same readings as Example 01, over SPI instead of I2C. Compare the two sketches: below
  setup(), they are identical. Everything above the bus - configuration, data, interrupts,
  FIFO, the feature engine - is the same code either way.

  Why bother with four more wires? Two reasons:

    1. Throughput. At 6400 Hz with both sensors running, each sample is 16 bytes of payload.
       That is over 100 kB/s, which eats most of a 400 kHz I2C bus. A 4 MHz SPI link moves it
       in a few percent of its capacity and leaves room for everything else.

    2. Addressing. The board has one ADR jumper, so I2C tops out at two BMI330s per bus. SPI
       needs only another chip select line.

  NOTE: the BMI330 powers up undecided about its interface and commits to SPI on the first CS
  edge it sees. That first transfer returns garbage by design. The library performs and
  discards it inside begin(), so nothing here has to care - but it is why a hand-rolled driver
  often fails to read the chip ID on the first try.

  SparkFun Electronics
  Date: 2026
  SparkFun code, firmware, and software is released under the MIT License.
    Please see LICENSE.md for further details.

  Hardware Connections:
  IoT RedBoard --> BMI330 Breakout
  CIPO (MISO)  --> SDO
  COPI (MOSI)  --> SDI
  SCK          --> SCK
  D10          --> CS
  3V3          --> 3V3
  GND          --> GND

  Open the Serial Monitor at 115200 baud.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/
*/

#include <SparkFun_BMI330.h>

// Chip select pin. Any free digital pin works.
const uint8_t kCSPin = 10;

SfeBMI330ArdSPI myIMU;

void setup()
{
    Serial.begin(115200);
    Serial.println("SparkFun BMI330 Example 2 - Basic Readings (SPI)");

    SPI.begin();

    // The default clock is 4 MHz. The part is specified to 10 MHz - raise it once the wiring is
    // short, lower it if long jumper wires are producing read errors.
    while (myIMU.begin(kCSPin) == false)
    {
        Serial.println("BMI330 not found - check your wiring and chip select pin!");
        delay(1000);
    }

    Serial.print("Connected to a BMI330 on CS pin ");
    Serial.println(myIMU.getCSPin());

    myIMU.setAccelConfig(BMI330_ACCEL_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_ACCEL_RANGE_4G);
    myIMU.setGyroConfig(BMI330_GYRO_MODE_HIGH_PERF, BMI330_ODR_100_HZ, BMI330_GYRO_RANGE_500DPS);

    Serial.println();
    Serial.println("AccX(g)\tAccY(g)\tAccZ(g)\tGyrX(dps)\tGyrY(dps)\tGyrZ(dps)\tTemp(C)");
}

void loop()
{
    sfe_bmi330_sensor_data_t data;

    if (myIMU.getSensorData(data) != ksfTkErrOk)
    {
        Serial.println("Read failed!");
        delay(1000);
        return;
    }

    Serial.print(data.accel.x, 3);
    Serial.print("\t");
    Serial.print(data.accel.y, 3);
    Serial.print("\t");
    Serial.print(data.accel.z, 3);
    Serial.print("\t");
    Serial.print(data.gyro.x, 2);
    Serial.print("\t\t");
    Serial.print(data.gyro.y, 2);
    Serial.print("\t\t");
    Serial.print(data.gyro.z, 2);
    Serial.print("\t\t");
    Serial.println(data.temperature, 1);

    delay(100);
}
