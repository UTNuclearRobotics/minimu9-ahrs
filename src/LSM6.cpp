#include "LSM6.h"
#include <stdexcept>

#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010

#define TEST_REG_ERROR -1

#define DS33_WHO_ID    0x69

LSM6::LSM6(const char * i2cDeviceName) : i2c(i2cDeviceName)
{
  detectAddress();
}

void LSM6::detectAddress()
{
    i2c.addressSet(DS33_SA0_LOW_ADDRESS);
    if (i2c.tryReadByte(LSM6::WHO_AM_I) == DS33_WHO_ID)
    {
        // Detected DS33 with the SA0 pin low.
        return;
    }

    i2c.addressSet(DS33_SA0_HIGH_ADDRESS);
    if (i2c.tryReadByte(LSM6::WHO_AM_I) == DS33_WHO_ID)
    {
        // Detected DS33 with the SA0 pin high.
        return;
    }

    throw std::runtime_error("Could not detect accelerometer and gyro.");
}

// Turns on the gyro and places it in normal mode.
void LSM6::enable()
{
    // Gyro init
    writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
    // Accel init
    writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
}

void LSM6::writeReg(uint8_t reg, uint8_t value)
{
    i2c.writeByte(reg, value);
}

uint8_t LSM6::readReg(uint8_t reg)
{
    return i2c.readByte(reg);
}

void LSM6::read()
{
    accelRead();
    gyroRead();
}

void LSM6::gyroRead()
{
    // read gyro
    uint8_t block[6];
    i2c.readBlock(OUTX_L_G, sizeof(block), block);
    g[0] = (int16_t)(block[1] << 8 | block[0]);
    g[1] = (int16_t)(block[3] << 8 | block[2]);
    g[2] = (int16_t)(block[5] << 8 | block[4]);
}

void LSM6::accelRead()
{
    // read acceleration
    uint8_t block[6];
    i2c.readBlock(OUTX_L_XL, sizeof(block), block);

    a[0] = (int16_t)(block[1] << 8 | block[0]);
    a[1] = (int16_t)(block[3] << 8 | block[2]);
    a[2] = (int16_t)(block[5] << 8 | block[4]);
}
