
#include <airskin/I2C_Slave.h>

I2C_Slave::I2C_Slave(I2C_Master *_master, unsigned char _addr)
{
  master = _master;
  addr = _addr;
}

