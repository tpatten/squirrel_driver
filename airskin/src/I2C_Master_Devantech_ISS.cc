/**
 * Devantech USB-ISS Multifunction USB Communications Module
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <airskin/Except.h>
#include <airskin/I2C_Master_Devantech_ISS.h>

#define USB_ISS      0x5A
// Returns 3 bytes, the module ID (7), firmware version (currently 2), and the current operating mode.
#define ISS_VERSION  0x01
// Sets operating mode, I2C/SPI/Serial etc.
#define ISS_MODE     0x02
// Returns the modules unique 8 byte USB serial number.
#define GET_SER_NUM  0x03

// microseconds to wait before a read. If this is too short (esp. 0) the read might fail.
// NOTE: this needs further investigation. There should be a cleaner way to do this.
#define WAIT_BEFORE_READ_US   1000

// Read/Write single byte for non-registered devices,
#define I2C_SGL           0x53
// Read multiple bytes without setting new address
#define I2C_MUL           0x54
// Read/Write single or multiple bytes for 1 byte addressed devices (the
// majority of devices will use this one)
#define I2C_AD1           0x55
// Read/Write single or multiple bytes for 2 byte addressed devices, eeproms
// from 32kbit (4kx8) and up
#define I2C_AD2           0x56
// A range of commands to the USB-I2C module, generally to improve selected
// communications or provide analogue/digital I/O
#define I2C_USB           0x5A
// The only of those "special" commands we use: Returns the USB-I2C firmware
// revision number
#define I2C_USB_REVISION  0x01

I2C_Master_Devantech_ISS::I2C_Master_Devantech_ISS(const std::string &device_file)
{
  struct termios config;  // These will be our new settings
  // O_NOCTTY: not the controlling terminal for the process
  // O_DSYNC | O_FSYNC: always do complete reads and writes, i.e. wait until all
  // data is transferred. For our very small data packages (a few bytes) this
  // does make sense.
  fd = open(device_file.c_str(), O_RDWR | O_NOCTTY | O_DSYNC | O_FSYNC);
  if(fd == -1)
    throw Except(__HERE__, "failed to open device '%s'", device_file.c_str());

  // Grab snapshot of current settings for port
  if(tcgetattr(fd, &defaults) < 0)
    perror("tcgetattr");
  // make options for raw data
  cfmakeraw(&config);

  cfsetispeed(&config, 19200);
  cfsetospeed(&config, 19200);

  // ignore modem control lines
  config.c_cflag |= CLOCAL;
  // activate receiver
  config.c_cflag |= CREAD;
  // disable parity
  config.c_cflag &= ~PARENB;
  // 2 stop bits
  config.c_cflag |= CSTOPB; 
  // character size mask
  config.c_cflag &= ~CSIZE;
  // 8 data bits
  config.c_cflag |= CS8;
  // to be sure: disable RTS/CTS (hardware) flow control.
  config.c_cflag &= ~CRTSCTS;

  // ICANON: Enable canonical mode. This enables the special characters EOF,
  // EOL, EOL2, ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by
  // lines.
  // ECHO: Echo input characters.
  // ISIG: When any of the characters INTR, QUIT, SUSP, or DSUSP are received,
  // generate the corresponding signal.
  config.c_lflag &= ~(ICANON | ECHO | ISIG);

  // IXON : Enable XON/XOFF flow control on output.
  // IXANY: Typing any character will restart stopped output. (The default is to
  // allow just the START character to restart output.)
  // IXOFF: Enable XON/XOFF flow control on input
  config.c_iflag &= ~(IXON | IXOFF | IXANY);

  // OPOST Enable implementation-defined output processing. 
  config.c_oflag &= ~OPOST;

  // VMIN: Minimum number of characters for non-canonical read.
  config.c_cc[VMIN] = 0;
  // VTIME: Timeout in deciseconds for non-canonical read.
  config.c_cc[VTIME] = 5;

  if(tcsetattr(fd, TCSANOW, &config) < 0)
    perror("tcsetattr config");

  SetI2CMode();
}

I2C_Master_Devantech_ISS::~I2C_Master_Devantech_ISS()
{
  // Restore port default before closing
  if(tcsetattr(fd, TCSANOW, &defaults) < 0)
    perror("tcsetattr default");
  close(fd);
}

void I2C_Master_Devantech_ISS::SetI2CMode()
{
  unsigned char buf[20];
  buf[0] = USB_ISS;
  buf[1] = ISS_MODE;
  buf[2] = 0x60;  // Set mode to 100KHz I2C using hardware I2C ports
  buf[3] = 0x00;  // Spare pins set to output low
  if(write(fd, buf, 4) < 0)
    throw Except(__HERE__, "write");
  if(tcdrain(fd) < 0)
    throw Except(__HERE__, "tcdrain");
  usleep(WAIT_BEFORE_READ_US);
  if(read(fd, buf, 2) < 0)
    throw Except(__HERE__, "read");
  // If first returned byte is not 0xFF then an error has occured
  if(buf[0] != 0xFF)
    throw Except(__HERE__, "setting I2C mode");
}

/*bool I2C_Master_Devantech::ReadACK()
{
  unsigned char ack = 0;
  CheckedRead(fd, &ack, 1);
  return ack != 0;
}*/

int I2C_Master_Devantech_ISS::GetFirmwareVersion()
{
  unsigned char buf[20];
  buf[0] = USB_ISS;
  buf[1] = ISS_VERSION;
  if(write(fd, buf, 2) < 0)
    throw Except(__HERE__, "write");
  if(tcdrain(fd) < 0)
    throw Except(__HERE__, "tcdrain");
  usleep(WAIT_BEFORE_READ_US);
  // read module ID (always 7), firmware version, operating mode (I2C/SPI/Serial etc.)
  if(read(fd, buf, 3) < 0)
    throw Except(__HERE__, "read");
  return (int)buf[1];
}

std::string I2C_Master_Devantech_ISS::GetSerialNumber()
{
  unsigned char buf[20];
  std::string sernum;
  buf[0] = USB_ISS;
  buf[1] = GET_SER_NUM;
  if(write(fd, buf, 2) < 0)
    throw Except(__HERE__, "write");
  if(tcdrain(fd) < 0)
    throw Except(__HERE__, "tcdrain");
  usleep(WAIT_BEFORE_READ_US);
  // read 8 bytes, each an ASCII number 0-9
  if(read(fd, buf, 8) < 0)
    throw Except(__HERE__, "read");
  buf[8] = '\0';
  sernum = (char*)buf;
  return sernum;
}

void I2C_Master_Devantech_ISS::Write(unsigned char addr, 
    unsigned char nbytes, const unsigned char data[])
{
/*  if(nbytes <= 0)
    throw Except(__HERE__, "number of bytes to send must be > 0");
  unsigned char buf[4 + nbytes];
  // note: Unfortunately there is no command analogous to I2C_MUL for _writing_
  // multiple bytes to a device without registers. So we have to use a
  // workaround using I2C_AD1.
  buf[0] = I2C_AD1;
  buf[1] = addr;
  // This would be the register. Here the receiver does not expect a register
  // but just nbytes of data. However the Devantech USB-I2C device always
  // expects a register. What it does eventually anywaay is to send a stream of
  // bytes starting with a byte containing the register followed by data bytes.
  // So we take our first data byte to be the "register".
  buf[2] = data[0];
  buf[3] = nbytes-1;
  memcpy(&buf[4], &(data[1]), (size_t)nbytes-1);
  CheckedWrite(serial_fd, buf, 4 + nbytes);
  if(!ReadACK())
    throw Except(__HERE__, "command failed");*/
}

void I2C_Master_Devantech_ISS::WriteRegister(unsigned char addr, unsigned char reg,
    unsigned char nbytes, const unsigned char data[])
{
/*  unsigned char buf[4 + nbytes];
  // this is how the Devantech USB-I2C device normally expects communication to
  // happen: specify communication mode, address, register and data
  buf[0] = I2C_AD1;
  buf[1] = addr;
  buf[2] = reg;
  buf[3] = nbytes;
  memcpy(&buf[4], data, (size_t)nbytes);
  CheckedWrite(serial_fd, buf, 4 + nbytes);
  if(!ReadACK())
    throw Except(__HERE__, "command failed");*/
}

void I2C_Master_Devantech_ISS::Read(unsigned char addr,
    unsigned char nbytes, unsigned char data[])
{
/*  unsigned char buf[3];
  // use I2C_MUL to read multiple byte from a device without registers
  buf[0] = I2C_MUL;
  buf[1] = addr + 1;  // add read bit
  buf[2] = nbytes;
  CheckedWrite(serial_fd, buf, 3);
  CheckedRead(serial_fd, data, (size_t)nbytes);*/
}

void I2C_Master_Devantech_ISS::ReadRegister(unsigned char addr, unsigned char reg,
    unsigned char nbytes, unsigned char data[])
{
  unsigned char buf[4];
  // this is how the Devantech USB-I2C device normally expects communication to
  // happen: specify communication mode, address, register and data
  buf[0] = 0x55;
  buf[1] = addr + 1;  // add read bit
  buf[2] = reg;
  buf[3] = nbytes;
  CheckedWrite(fd, buf, 4);
  CheckedRead(fd, data, (size_t)nbytes);
}
