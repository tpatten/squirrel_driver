/*
 * Driver for the AirSkin prototype, version Sept 2014.
 */

#include <cstdio>
#include <unistd.h>
#include <airskin/AirSkin_Sense.h>

AirSkin_Sense::AirSkin_Sense(I2C_Master *_master, unsigned char _addr)
  : I2C_Slave(_master, _addr)
{
}

/**
 * Read the raw atmospheric pressure value.
 */
int AirSkin_Sense::ReadRawPressure()
{
  unsigned char b[4];
  master->ReadRegister(addr, 0x12, 4, b);
  uint32_t p = (((uint32_t)b[3] << 24)) + (((uint32_t)b[2]) << 16) + (((uint32_t)b[1]) << 8) + (uint32_t)b[0];
  return (int)p;
}

/**
 * Read the filtered pressure value, with sliding mean as reference.
 */
int AirSkin_Sense::ReadFilteredPressure()
{
  unsigned char b[4];
  master->ReadRegister(addr, 0x02, 4, b);
  uint32_t p = (((uint32_t)b[3] << 24)) + (((uint32_t)b[2]) << 16) + (((uint32_t)b[1]) << 8) + (uint32_t)b[0];
  return (int)p;
}

