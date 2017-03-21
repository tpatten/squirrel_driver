#include "idmind_serial/idmind_serial.h"
#include <ros/ros.h>


IdmindSerial::IdmindSerial(const std::string& port, uint32_t baudrate, uint32_t timeout)
    : failed_read_(0), failed_write_(0), max_time_bw_writes_(10),
      ready_(false), reconnected_(false), check_sum_(true)
{
  my_connection_.setPort(port);
  my_connection_.setBaudrate(baudrate);
  serial::Timeout my_timeout = serial::Timeout::simpleTimeout(timeout);
  my_connection_.setTimeout(my_timeout);

  open();
}

IdmindSerial::~IdmindSerial()
{
  close();
}

void IdmindSerial::checkPorts(const ros::TimerEvent& event)
{
  if (!my_connection_.isOpen())
    if (open())
      reconnected_ = true;
}

bool IdmindSerial::read(uint8_t* data, size_t size, bool block)
{
  if (my_connection_.isOpen())
  {
    try
    {
      if (block)
      {
        ros::Time last = ros::Time::now();
        while (my_connection_.available() < size)
        {
          if (ros::Time::now() > last + ros::Duration(0.1))
          {
            ROS_ERROR("SERIAL --> Read desist.");
            break;
          }
        }
//        ROS_INFO("Read blocked %.2fms. Expected size is %d.",
//        static_cast<double>((ros::Time::now() - last).nsec) / 1e6, static_cast<int>(size));
      }

      if (my_connection_.available() >= size)
      {
        if (my_connection_.read(data, size) == size)
        {
          if (checkSum(data, size))
            return true;
          else
          {
            my_connection_.flushInput();
            failed_read_++;
          }
        }

      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("SERIAL --> Could not read from %s: %s", my_connection_.getPort().c_str(), e.what());
      my_connection_.close();
      return false;
    }
  }

  return false;
}

bool IdmindSerial::read(int& message, bool block)
{
  uint8_t buffer[2];

  if (read(buffer, 2, block))
  {
    message = readInt(buffer);
    return true;
  }

  return false;
}

bool IdmindSerial::write (const uint8_t* data, size_t size)
{
  if (my_connection_.isOpen())
  {
    try
    {
      my_connection_.flushInput();

      if (my_connection_.write(data, size) == size)
      {
        last_written_ = ros::Time::now();
        return true;
      }
      else
        failed_write_++;
    }
    catch (serial::SerialException& e)
    {
      ROS_ERROR("SERIAL --> Could not write to %s: %s", my_connection_.getPort().c_str(), e.what());
      my_connection_.close();
      return false;
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("SERIAL --> Could not write to %s: %s", my_connection_.getPort().c_str(), e.what());
      my_connection_.close();
      return false;
    }
  }

  return false;
}

bool IdmindSerial::write (uint8_t data)
{
  return write(&data, 1);
}

bool IdmindSerial::write(int data)
{
  uint8_t message[2];
  message[0] = (uint8_t) ((data >> 8) & 0xFF);
  message[1] = (uint8_t) (data & 0xFF);

  return write(message, 2);
}

int IdmindSerial::available()
{
  return my_connection_.available();
}

void IdmindSerial::flush()
{
  my_connection_.flushInput();
}

bool IdmindSerial::maxTimeBwWritesPassed()
{
  if (my_connection_.isOpen())
    if ((ros::Time::now() - last_written_).toSec() > max_time_bw_writes_)
      return true;

  return false;
}

int IdmindSerial::readInt(const uint8_t* bytes)
{
  int result = (int)((*bytes << 8) | (*(bytes + 1) & 0xFF));
  if (result > 32767)  result = result - 65536;
  return result;
}

void IdmindSerial::addInt(uint8_t* bytes, int data)
{
  *bytes = (uint8_t) ((data >> 8) & 0xFF);
  *(bytes + 1) = (uint8_t) (data & 0xFF);
}

bool IdmindSerial::open()
{
  try
  {
    my_connection_.open();
    my_connection_.flushInput();
    ready_ = true;

    ROS_INFO("SERIAL --> Port %s successfully opened.", my_connection_.getPort().c_str());
    return true;
  }
  catch (serial::SerialException &e)
  {
    ROS_ERROR("SERIAL --> Could not open %s: %s", my_connection_.getPort().c_str(), e.what());
    return false;
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR("SERIAL --> Could not open %s: %s", my_connection_.getPort().c_str(), e.what());
    return false;
  }
}

void IdmindSerial::close()
{
  if (my_connection_.isOpen())
    my_connection_.close();

  ready_ = false;
}

bool IdmindSerial::checkSum(const uint8_t* bytes, size_t size)
{
  if (!check_sum_)
    return true;

  int my_checksum = 0;
  for (int i = 0; i < size-2; i++)
  {
    my_checksum += (int)(*(bytes + i));
  }

  if (my_checksum == readInt(bytes + (size-2)))
    return true;
  else
  {
    ROS_ERROR("SERIAL --> Checksum wrong.");
    return false;
  }
}

void IdmindSerial::printBytes(const uint8_t* bytes, size_t size)
{
  for(size_t i = 0; i < size; i++)
  {
    ROS_INFO_STREAM(std::hex << *(bytes + i));
  }
}
