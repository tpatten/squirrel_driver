//
// Created by lokalmatador on 12.04.17.
//

#ifndef SQUIRREL_CONTROL_MOTOR_UTILITIES_H
#define SQUIRREL_CONTROL_MOTOR_UTILITIES_H

#include <mutex>
#include <error/throwControlError.h>

#include <squirrel_control/squirrel_hardware_interface.h>
#include <dynamixel_sdk/PortHandler.h>
#include <dynamixel_sdk/Protocol2PacketHandler.h>
#include <dynamixel_sdk/dynamixel_tool.h>

namespace motor_control
{

    struct Motor {
        UINT8_T id;
        dynamixel_tool::DynamixelTool *tool;
    };


    class MotorUtilities
    {
    public:
        static constexpr int BAUD_RATE = 3000000;

        MotorUtilities();

        ~MotorUtilities();

        bool initMotors(std::string motor_port, std::vector<int> motors);

        bool startMotors();

        bool stopMotors();

        bool motorsReady();

        bool enableTorque();

        bool disableTorque();

        bool torqueEnabled();

        std::vector<Motor> getMotors();

        void setMode(squirrel_control::SquirrelControlMode mode);

        squirrel_control::SquirrelControlMode getMode();

        bool write(std::vector<UINT16_T>commands);


    private:

        bool motors_ready;

        bool torque_enabled;

        std::mutex motor_lock;

        std::vector<Motor> motors;

        squirrel_control::SquirrelControlMode current_mode;

        ROBOTIS::PortHandler* portHandler;

        ROBOTIS::Protocol2PacketHandler* packetHandler;
    };

}

#endif //SQUIRREL_CONTROL_MOTOR_UTILITIES_H
