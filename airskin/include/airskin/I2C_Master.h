/**
 * Base class for I2C masters
 *
 * Currentl available derived classes are the overo onboard I2C bus and the
 * Devantech USB-I2C adapter.
 */

#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <sys/types.h>

class I2C_Master
{
protected:
  /**
   * Write to device.
   */
  void CheckedWrite(int fd, const unsigned char buf[], size_t len);

  /**
   * Read from device.
   */
  void CheckedRead(int fd, unsigned char buf[], size_t len);

public:
  virtual ~I2C_Master() {}

  /**
   * Write to an I2C slave
   * @param addr  single byte address of I2C slave
   * @param nbytes  number of bytes to write
   * @param data  actual bytes to write, must be at least nbytes large
   */
  virtual void Write(unsigned char addr, unsigned char nbytes,
      const unsigned char data[]) = 0;

  /**
   * Write to a register of an I2C slave
   * @param addr  single byte address of I2C slave
   * @param reg   register to write to
   * @param nbytes  number of bytes to write
   * @param data  actual bytes to write, must be at least nbytes large
   */
  virtual void WriteRegister(unsigned char addr, unsigned char reg,
      unsigned char nbytes, const unsigned char data[]) = 0;

  /**
   * Read from an I2C slave
   * @param addr  single byte address of I2C slave, read bit will be added
   *              internally
   * @param nbytes  number of bytes to read
   * @param data  where to store the bytes, must be at least nbytes large
   */
  virtual void Read(unsigned char addr, unsigned char nbytes,
      unsigned char data[]) = 0;

  /**
   * Read from a register of an I2C slave
   * @param addr  single byte address of I2C slave, read bit will be added
   *              internally
   * @param reg   register to read from
   * @param nbytes  number of bytes to read
   * @param data  where to store the bytes, must be at least nbytes large
   */
  virtual void ReadRegister(unsigned char addr, unsigned char reg,
      unsigned char nbytes, unsigned char data[]) = 0;
};

#endif

