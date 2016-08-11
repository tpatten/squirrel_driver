#ifndef UIBK_ARM_CONTROLLER
#define UIBK_ARM_CONTROLLER

#include <string>
#include <vector>
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
#include <uibk_robot_driver/motor.hpp>
#include <boost/thread.hpp>

// Control table address
#define DONTCARE nan("1")

class Arm {

    private:

        bool keepThreadRunning;
        bool firstJointStateRetrieved;

        std::vector<std::shared_ptr<motor_controller::Motor> > motors;
        std::vector<double> currentJointState;

        boost::mutex jointStateMutex;
        boost::mutex moveMutex;

        std::shared_ptr<std::thread> runnerThread;
        std::shared_ptr<std::thread> moveThread;
        std::shared_ptr<std::thread> moveThreadController;
        double move_rate;
        bool moveArmOk;
        bool allowMovement;
        std::vector<double> targetPosMove;
        void armLoop();
        void moveArmThread();
        void moveArmThreadController();
        bool checkDistance(std::vector<double> &current, std::vector<double> &target, double& exceededDist, double& maxDist);
        void move(std::vector<double> nextJointPos);
    public:

        Arm(std::vector<int> ids, std::vector<motor_controller::motor_type> types, std::string portName, std::vector< std::pair<double, double> > jointLimits, double protocolVersion, int baudRate, double move_freq);

        std::vector<double> getCurrentState();

        std::shared_ptr<std::thread> runArm();
        void moveHome();
        void shutdown();
        void initialize();

        void jointPtp(std::vector<double> targetPos);
        void moveArm(std::vector<double> nextJointPos);
        int getDegOfFreedom();

        double getStepSize(int motorId);
        double getFrequency();
        double getCycleTime();
        double getMaxStepPerCycle();

};


#endif
