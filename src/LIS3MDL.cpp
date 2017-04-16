#include "LIS3MDL.h"
#include <stdexcept>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define LIS3MDL_WHO_ID  0x3D

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDL::LIS3MDL(const char * i2cDeviceName) : 
  i2c(i2cDeviceName)
{
  detectAddress();
}

void LIS3MDL::detectAddress()
{
    i2c.addressSet(LIS3MDL_SA1_LOW_ADDRESS);
    if (i2c.tryReadByte(LIS3MDL::WHO_AM_I) ==  LIS3MDL_WHO_ID)
    {
        // Detected LIS3MDL with the SA0 pin low.
        return;
    }

    i2c.addressSet(LIS3MDL_SA1_HIGH_ADDRESS);
    if (i2c.tryReadByte(LIS3MDL::WHO_AM_I) ==  LIS3MDL_WHO_ID)
    {
        // Detected LIS3MDL with the SA0 pin high.
        return;
    }

    throw std::runtime_error("Could not detect accelerometer and gyro.");
}


// Public Methods //////////////////////////////////////////////////////////////

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS3MDL::enable(void)
{
  // 0x70 = 0b01110000
  // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
  writeMagReg(CTRL_REG1, 0x70);

  // 0x00 = 0b00000000
  // FS = 00 (+/- 4 gauss full scale)
  writeMagReg(CTRL_REG2, 0x00);

  // 0x00 = 0b00000000
  // MD = 00 (continuous-conversion mode)
  writeMagReg(CTRL_REG3, 0x00);

  // 0x0C = 0b00001100
  // OMZ = 11 (ultra-high-performance mode for Z)
  writeMagReg(CTRL_REG4, 0x0C);
}

// Writes a mag register
void LIS3MDL::writeMagReg(uint8_t reg, uint8_t value)
{
  i2c.writeByte(reg, value);
}

// Reads a mag register
uint8_t LIS3MDL::readMagReg(uint8_t reg)
{
  return i2c.readByte(reg);
}

// Reads the 3 mag channels and stores them in vector m
void LIS3MDL::read()
{
  uint8_t block[6];
  i2c.readBlock(0x80 | OUT_X_L, sizeof(block), block);


  // combine high and low bytes
  m[0] = (int16_t)(block[1] << 8 | block[0]);
  m[1] = (int16_t)(block[3] << 8 | block[2]);
  m[2] = (int16_t)(block[5] << 8 | block[4]);
}

