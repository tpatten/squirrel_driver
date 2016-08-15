/**
 * Devantech USB-I2C adapter
 */

#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <airskin/Except.h>
#include <airskin/I2C_Master_Devantech.h>

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

I2C_Master_Devantech::I2C_Master_Devantech(const std::string &device_file)
{
  speed_t baudrate = B19200;

  struct termios termios_param;

  // O_NOCTTY: not the controlling terminal for the process
  // O_DSYNC | O_FSYNC: always do complete reads and writes, i.e. wait until all
  // data is transferred. For our very small data packages (a few bytes) this
  // does make sense.
  serial_fd = open(device_file.c_str(), O_RDWR | O_NOCTTY | O_DSYNC | O_FSYNC);
  if(serial_fd == -1)
    throw Except(__HERE__, "failed to open device '%s'", device_file.c_str());

  tcgetattr(serial_fd, &termios_param);

  cfsetispeed(&termios_param, baudrate);
  cfsetospeed(&termios_param, baudrate);
 
  // ignore modem control lines
  termios_param.c_cflag |= CLOCAL;
  // activate receiver
  termios_param.c_cflag |= CREAD;
  // disable parity
  termios_param.c_cflag &= ~PARENB;
  // 2 stop bits
  termios_param.c_cflag |= CSTOPB; 
  // character size mask
  termios_param.c_cflag &= ~CSIZE;
  // 8 data bits
  termios_param.c_cflag |= CS8;
  // to be sure: disable RTS/CTS (hardware) flow control.
  termios_param.c_cflag &= ~CRTSCTS;

  // ICANON: Enable canonical mode. This enables the special characters EOF,
  // EOL, EOL2, ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by
  // lines.
  // ECHO: Echo input characters.
  // ISIG: When any of the characters INTR, QUIT, SUSP, or DSUSP are received,
  // generate the corresponding signal.
  termios_param.c_lflag &= ~(ICANON | ECHO | ISIG);

  // IXON : Enable XON/XOFF flow control on output.
  // IXANY: Typing any character will restart stopped output. (The default is to
  // allow just the START character to restart output.)
  // IXOFF: Enable XON/XOFF flow control on input
  termios_param.c_iflag &= ~(IXON | IXOFF | IXANY);

  // OPOST Enable implementation-defined output processing. 
  termios_param.c_oflag &= ~OPOST;

  // VMIN: Minimum number of characters for non-canonical read.
  termios_param.c_cc[VMIN] = 0;
  // VTIME: Timeout in deciseconds for non-canonical read.
  termios_param.c_cc[VTIME] = 5;

  tcsetattr(serial_fd, TCSADRAIN, &termios_param);
}

I2C_Master_Devantech::~I2C_Master_Devantech()
{
  close(serial_fd);
}

bool I2C_Master_Devantech::ReadACK()
{
  unsigned char ack = 0;
  CheckedRead(serial_fd, &ack, 1);
  return ack != 0;
}

int I2C_Master_Devantech::GetFirmwareVersion()
{
  unsigned char buf[4];
  buf[0] = I2C_USB;
  buf[1] = I2C_USB_REVISION;
  buf[2] = 0x00;  // unused but must be sent, just set to zero
  buf[3] = 0x00;  // unused but must be sent, just set to zero
  CheckedWrite(serial_fd, buf, 4);
  CheckedRead(serial_fd, buf, 1);
  return (int)buf[0];
}

void I2C_Master_Devantech::Write(unsigned char addr, 
    unsigned char nbytes, const unsigned char data[])
{
  if(nbytes <= 0)
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
    throw Except(__HERE__, "command failed");
}

void I2C_Master_Devantech::WriteRegister(unsigned char addr, unsigned char reg,
    unsigned char nbytes, const unsigned char data[])
{
  unsigned char buf[4 + nbytes];
  // this is how the Devantech USB-I2C device normally expects communication to
  // happen: specify communication mode, address, register and data
  buf[0] = I2C_AD1;
  buf[1] = addr;
  buf[2] = reg;
  buf[3] = nbytes;
  memcpy(&buf[4], data, (size_t)nbytes);
  CheckedWrite(serial_fd, buf, 4 + nbytes);
  if(!ReadACK())
    throw Except(__HERE__, "command failed");
}

void I2C_Master_Devantech::Read(unsigned char addr,
    unsigned char nbytes, unsigned char data[])
{
  unsigned char buf[3];
  // use I2C_MUL to read multiple byte from a device without registers
  buf[0] = I2C_MUL;
  buf[1] = addr + 1;  // add read bit
  buf[2] = nbytes;
  CheckedWrite(serial_fd, buf, 3);
  CheckedRead(serial_fd, data, (size_t)nbytes);
}

void I2C_Master_Devantech::ReadRegister(unsigned char addr, unsigned char reg,
    unsigned char nbytes, unsigned char data[])
{
  unsigned char buf[4];
  // this is how the Devantech USB-I2C device normally expects communication to
  // happen: specify communication mode, address, register and data
  buf[0] = I2C_AD1;
  buf[1] = addr + 1;  // add read bit
  buf[2] = reg;
  buf[3] = nbytes;
  CheckedWrite(serial_fd, buf, 4);
  CheckedRead(serial_fd, data, (size_t)nbytes);
}

