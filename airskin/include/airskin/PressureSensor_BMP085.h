/*
 * Driver for the BMP085 pressure/temperature sensor. 
 * based on code by: Jim Lindblom, SparkFun Electronics
 */

#ifndef PRESSURE_SENSOR_BMP085_H
#define PRESSURE_SENSOR_BMP085_H

#include <stdint.h>
#include "I2C_Slave.h"

/**
 * I2C address of the BMP085. Note that this is fixed. So you can not have more
 * than one BMP085 on a single I2C bus (which is shit!). Except if you set all
 * except one sensor silent by using the XCLR (master clear) input.
 */
#define BMP085_ADDR 0xEE

class PressureSensor_BMP085 : public I2C_Slave
{
private:
  // calibration data
  int32_t ac1;
  int32_t ac2; 
  int32_t ac3; 
  uint32_t ac4;
  uint32_t ac5;
  uint32_t ac6;
  int32_t b1; 
  int32_t b2;
  int32_t mb;
  int32_t mc;
  int32_t md;

  uint16_t ReadUInt16(unsigned char addr, unsigned char reg);
  int16_t ReadInt16(unsigned char addr, unsigned char reg);
  void ReadCalibration();
  int32_t ReadRawPressure();
  int32_t ReadRawTemperature();

public:
  PressureSensor_BMP085(I2C_Master *_master, unsigned char _addr = BMP085_ADDR);
  float ReadPressure();
  float ReadTemperature();
  void ReadPressureTemperature(float *press, float *temp);
};

#endif

