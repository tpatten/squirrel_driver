#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>
#include <vector>
#include <exception>
#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>
#include <utility>
#include <thread>
#include <chrono>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <dynamixel_sdk/DynamixelSDK.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque


namespace motor_controller {

class MotorException : public std::exception {

    private:
        const char* msg;
    public:

        MotorException(const char* msg);

        virtual const char* what() const throw();

};

enum motor_type { DYNAMIXEL_BIG_MOTOR, DYNAMIXEL_SMALL_MOTOR };

class Motor {

    private:
    
		motor_type type;

        bool nextCommandSet;

        double currentState;
        double nextCommand;
        
        double std_stepsize;
        double std_max_vel_limit;
        double ticks_for_180_deg;

        int motorId;
        double lowerLimit;
        double upperLimit;
        int baudRate;

        std::string deviceName;
        float  protocolVersion;

        ROBOTIS::PortHandler* portHandler;
        ROBOTIS::PacketHandler* packetHandler;

        std::mutex stateMutex;
        std::mutex commandMutex;

        void sendNextCommand(double pos);

        void submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, int value);
        void submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, UINT8_T value);

        int receivePacket(ROBOTIS::PortHandler* portHandler, int motorId, int address);

        int receiveState();

    public:

        static auto constexpr STD_FREQUENCY = 80.0;

		static auto constexpr TICKS_FOR_180_DEG_BIG = 250000.0;
        static auto constexpr TICKS_FOR_180_DEG_SMALL = 150000.0;

        Motor(std::string deviceName, int motorId, motor_type type, float protocolVersion, double lowerLimit, double upperLimit, int baudRate);

        double getStepSize();
        double getFrequency();

        void initialize();

        void setTorqueMode(bool mode);

        void startTorqueMode();

        void stopTorqueMode();

        void setBrakes(bool brakesOpen);

        void releaseBrakes();

        void closeBrakes();

        void shutdown();

        void spinOnce();

        void simplePtp(int targetPos);
        void move(int targetPos);

        void flushNextState();

        void goHome();

        double getCurrentState();

        void setNextState(double state);

        double getMaxVelLimit();

};

}
#endif
