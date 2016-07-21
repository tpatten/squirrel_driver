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
// Control table address
#define DONTCARE nan("1")
using namespace motor_controller;
class Arm {

    private:

        bool keepThreadRunning;
        bool firstJointStateRetrieved;


        std::vector<std::shared_ptr<Motor> > motors;
        std::vector<double> currentJointState;

        std::mutex jointStateMutex;

        std::shared_ptr<std::thread> runnerThread;

        void armLoop();

        bool checkDistance(std::vector<double> &current, std::vector<double> &target, double& exceededDist, double& maxDist);
        void move(std::vector<double> nextJointPos);
    public:

        Arm(std::vector<int> ids, std::string portName, std::vector< std::pair<double, double> > jointLimits, double protocolVersion, int baudRate);

        std::vector<double> getCurrentState();

        std::shared_ptr<std::thread> runArm();

        void moveHome();
        void shutdown();
        void initialize();

        void jointPtp(std::vector<double> targetPos);
        void moveArm(std::vector<double> nextJointPos);
        int getDegOfFreedom();

        double getStepSize();
        double getFrequency();
        double getCycleTime();
        double getMaxStepPerCycle();

};


#endif
