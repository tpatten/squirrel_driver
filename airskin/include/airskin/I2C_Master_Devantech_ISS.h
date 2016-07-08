/**
 * Devantech USB-ISS  Multifunction USB Communications Module
 * (http://www.robot-electronics.co.uk/htm/usb_iss_tech.htm)
 *
 * Based on example source code by James Henderson.
 *
 * @author Michael Zillich, zillich@bluedanuberobotics.com
 * @date Jan, 2015
 */

#ifndef I2C_MASTER_DEVANTECH_ISS_H
#define I2C_MASTER_DEVANTECH_ISS_H

#include <termios.h>
#include <string>
#include <airskin/I2C_Master.h>

class I2C_Master_Devantech_ISS : public I2C_Master
{
private:
  struct termios defaults;  // to store initial default port settings
  int fd;

  void SetI2CMode();

  /**
   * Read single ACK byte, used after sending a command.
   */
  // bool ReadACK();

public:
  I2C_Master_Devantech_ISS(const std::string &device_file = "/dev/ttyACM0");

  virtual ~I2C_Master_Devantech_ISS();

  /**
   * Returns I2C adapter firmware version.
   */
  int GetFirmwareVersion();

  /**
   * Returns the unique 8 byte USB serial number.
   */
  std::string GetSerialNumber();

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


