//
// Created by lokalmatador on 12.04.17.
//

#ifndef SQUIRREL_CONTROL_MOTOR_UTILITIES_H
#define SQUIRREL_CONTROL_MOTOR_UTILITIES_H

#include <mutex>
#include <error/throwControlError.h>

#include <squirrel_control/squirrel_hw_interface.h>
#include <dynamixel_sdk/PortHandler.h>
#include <dynamixel_sdk/Protocol2PacketHandler.h>
#include <dynamixel_sdk/dynamixel_tool.h>

namespace motor_control
{

#define DYNAMIXEL_POSITION_MODE 3
#define DYNAMIXEL_VELOCITY_MODE 1
#define DYNAMIXEL_TORQUE_MODE 0

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

        /**
            Writes the high level control commands to the motor. this operation supports all three operating modes, i.e.,
                position mode, velocity mode, and torque mode

            @param commands, a vector of either (i) joint position, (ii) joint velocities, or (iii) joint torques
                depending on the current operation mode; the length of this vector is expected to be this.motors.size
            @return true in case of success
        */
        bool write(std::vector<double>commands);


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
