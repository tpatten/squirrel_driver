/*
 * Driver for the AirSkin prototype, version Sept 2014.
 */

#ifndef AIRSKIN_PROTOTYPE_H
#define AIRSKIN_PROTOTYPE_H

#include <stdint.h>
#include <airskin/I2C_Slave.h>

/**
 * default I2C address: 0x04 (7 Bit) -> 0x08 (8 Bit), with last bit being the read/write bit
 */
#define AIRSKIN_DEFAULT_ADDR 0x08

class AirSkin_Sense : public I2C_Slave
{
private:
  uint32_t ReadUInt32(unsigned char addr, unsigned char reg);

public:
  AirSkin_Sense(I2C_Master *_master, unsigned char _addr = AIRSKIN_DEFAULT_ADDR);
  int ReadRawPressure();
  int ReadFilteredPressure();
};

#endif

