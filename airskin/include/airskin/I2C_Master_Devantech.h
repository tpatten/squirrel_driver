/**
 * Devantech USB-I2C adapter
 * (with 8 ultrasonic sensors SRF02 attached, addresses: 0xE0 - 0xEE)
 *
 * RS232 parameters:
 * - Baudrate: 19200
 * - 8 data bits
 * - no parity
 * - 2 stop bits
 *
 * datasheets: 
 * USB-I2C Adapter: http://www.robot-electronics.co.uk/htm/usb_i2c_tech.htm
 */

#ifndef I2C_MASTER_DEVANTECH_H
#define I2C_MASTER_DEVANTECH_H

#include <string>
#include <airskin/I2C_Master.h>

class I2C_Master_Devantech : public I2C_Master
{
private:
  int serial_fd;

  /**
   * Read single ACK byte, used after sending a command.
   */
  bool ReadACK();

public:
  I2C_Master_Devantech(const std::string &device_file = "/dev/ttyUSB0");

  virtual ~I2C_Master_Devantech();

  /**
   * Returns I2C adapter firmware version.
   */
  int GetFirmwareVersion();

  virtual void Write(unsigned char addr, unsigned char nbytes,
      const unsigned char data[]); 

  virtual void WriteRegister(unsigned char addr, unsigned char reg,
      unsigned char nbytes, const unsigned char data[]); 

  virtual void Read(unsigned char addr, unsigned char nbytes,
      unsigned char data[]);

  virtual void ReadRegister(unsigned char addr, unsigned char reg,
      unsigned char nbytes, unsigned char data[]);
};

#endif


