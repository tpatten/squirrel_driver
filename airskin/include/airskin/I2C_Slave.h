#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <airskin/I2C_Master.h>

class I2C_Slave
{
protected:
  I2C_Master *master;
  unsigned char addr;

public:
  I2C_Slave(I2C_Master *_master, unsigned char _addr);
  unsigned char GetAddress() {return addr;}
};

#endif

