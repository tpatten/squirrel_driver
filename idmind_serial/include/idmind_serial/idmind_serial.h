#ifndef IDMIND_SERIAL_H
#define IDMIND_SERIAL_H

#include <ros/ros.h>
#include <serial/serial.h>

class IdmindSerial
{
public:
  IdmindSerial(const std::string &port, uint32_t baudrate=115200,  uint32_t timeout=10);
  ~IdmindSerial();

  void checkPorts(const ros::TimerEvent& event);

  bool read(uint8_t *data, size_t size, bool block);
  bool read(int &message, bool block);

  bool write(const uint8_t *data, size_t size);
  bool write(uint8_t data);
  bool write(int message);

  int available();
  void flush();
  bool maxTimeBwWritesPassed();

  int readInt(const uint8_t *bytes);
  void addInt(uint8_t *bytes, int data);

  int failed_read_, failed_write_;
  int max_time_bw_writes_;

  bool ready_, reconnected_;
  bool check_sum_;

private:
  bool open();
  void close();

  bool checkSum(const uint8_t *bytes, size_t size);
  void printBytes(const uint8_t *bytes, size_t size);

  serial::Serial my_connection_;
  ros::Time last_written_;
};

#endif
