#ifndef _MINIMU9_H
#define _MINIMU9_H

#include "IMU.h"
#ifdef IMU_V5
#include "LSM6.h"
#include "LIS3MDL.h"
#else
#include "LSM303.h"
#include "L3G.h"
#endif

class MinIMU9 : public IMU {
public:
    MinIMU9(const char * i2cDeviceName);

    #ifdef IMU_V5
    LSM6 gyro_accel;
    LIS3MDL mag;
    #else
    LSM303 compass;
    L3G gyro;
    #endif

    virtual vector readAcc();
    virtual vector readMag();
    virtual vector readGyro();

    virtual void enable();
    virtual void loadCalibration();
    virtual void measureOffsets();
};

#endif
