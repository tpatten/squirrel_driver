#include <uibk_robot_driver/arm_controller.hpp>
#include <math.h>
#include <time.h>

using namespace ROBOTIS;                                    // Uses functions defined in ROBOTIS namespace


    void Arm::armLoop() {
		
        auto frequ = motors.front()->getFrequency();
        int sleepTime = (int) (1.0 / frequ * 1e3);
        while(keepThreadRunning) {
            for(auto motor : motors)
                motor->spinOnce();
			
            jointStateMutex.lock();
			
                 currentJointState.clear();
//                currentJointState.push_back(base->getCurrentState());
                for(auto motor : motors) {

                    auto currState = motor->getCurrentState();
                    currentJointState.push_back(currState);

                }
				
            jointStateMutex.unlock();
            if(!firstJointStateRetrieved)
                firstJointStateRetrieved = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			
        }
		
    }

    Arm::Arm(std::vector<int> ids, std::string portName, std::vector<std::pair<double, double> > jointLimits, double protocolVersion, int baudRate, double move_freq) {
        move_rate=move_freq;
        firstJointStateRetrieved = false;
        for(unsigned int i = 0; i < ids.size(); ++i) {
            auto id = ids.at(i);
            auto limit = jointLimits.at(i);
            motors.push_back(std::make_shared<Motor>(portName, id, protocolVersion, limit.first, limit.second, baudRate));
        }

        keepThreadRunning = false;

        allowMoveArm=false;
    }

    std::vector<double> Arm::getCurrentState() {
        std::vector<double> jointStateBkp;
        jointStateMutex.lock();
            jointStateBkp = currentJointState;
        jointStateMutex.unlock();
        return jointStateBkp;
    }

    void Arm::initialize() {
		
        for(auto motor : motors) {
            motor->initialize();
            motor->startTorqueMode();
            motor->releaseBrakes();
            motor->spinOnce();
        }
        moveThread=std::make_shared<std::thread>(&Arm::moveArmThread, this);

    }

    void Arm::move(std::vector<double> nextJointPos) {

        if(nextJointPos.size() != (motors.size()))
            throw MotorException("number of joints doesn't fit the number of motors");

        auto js = getCurrentState();
        double velLimit = 0.0;
        double exceededDist = 0.0;
        if(checkDistance(js, nextJointPos, exceededDist, velLimit)) {
            for(unsigned int i = 0; i < nextJointPos.size(); ++i){
                if (std::isnan(nextJointPos.at(i)) == 0)
                    motors.at(i)->setNextState(nextJointPos.at(i));
            }
        } else {
            std::cerr << "velocity limit exceeded (maxVel: " << velLimit << ", commandedVel: " << exceededDist << ")" << std::endl;
        }

    }
    void Arm::moveArm(std::vector<double> targetPos) {
        moveMutex.lock();
          targetPosMove=targetPos;
          allowMoveArm=true;
        moveMutex.unlock();
    }

    void Arm::moveArmThread() {
        ros::Rate myRate(move_rate);
        while(1){
            moveMutex.lock();
            std::vector<double> targetpos= targetPosMove;
            moveMutex.unlock();
            if(allowMoveArm){
                auto jointState = getCurrentState();
                auto runnerState = jointState;
                int sleepTime = (int) (1.0 / (getFrequency()*4) * 1e3);
                auto stepSize = getStepSize() * 3;

                auto targetReached = false;

                std::clock_t start = std::clock();

                while(!targetReached) {

                        targetReached = true;
                        for(unsigned int i = 0; i < runnerState.size(); ++i) {

                            if(std::isnan(targetpos.at(i))!= 0)
                                targetpos.at(i)=runnerState.at(i);
                            auto sig = ((runnerState.at(i) - targetpos.at(i)) > 0) ? -1 : 1;
                            if(fabs(runnerState.at(i) - targetpos.at(i)) > (stepSize * 2)) {
                                runnerState.at(i) += sig * stepSize;
                                targetReached = false;
                            }

                        }

                        move(runnerState);

                        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));

                    if((1.0*std::clock()-start)/CLOCKS_PER_SEC > 1.0/ move_rate)
                        break;

                }

                moveMutex.lock();
                allowMoveArm=false;
                moveMutex.unlock();

            }

        }
    }

    bool Arm::checkDistance(std::vector<double>& current, std::vector<double>& target, double& exceededDist, double& maxDist) {

//        if(fabs(current.front() - target.front()) > 0.6) {
//            exceededDist = fabs(current.front() - target.front());
//            maxDist = 0.6;
//            return false;
//        }

        for(int i = 0; i < current.size(); ++i) {

            if(fabs(current.at(i) - target.at(i)) > motors.at(i)->getMaxVelLimit()) {
                exceededDist = fabs(current.at(i) - target.at(i));
                maxDist = motors.at(i )->getMaxVelLimit();
                return false;
            }
        }
        return true;
    }

    double Arm::getMaxStepPerCycle() {
        return motors.front()->getMaxVelLimit();
    }

    std::shared_ptr<std::thread> Arm::runArm() {
		
        keepThreadRunning = true;
        runnerThread = std::make_shared<std::thread>(&Arm::armLoop, this);
        while(!firstJointStateRetrieved)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
        return runnerThread;
		
    }

    void Arm::shutdown() {
		
        keepThreadRunning = false;
        if(runnerThread)
            runnerThread->join();
        for(auto motor : motors) {
            motor->shutdown();
        }
		
    }

    void Arm::jointPtp(std::vector<double> targetPos) {
		
        auto jointState = getCurrentState();
        auto runnerState = jointState;
        int sleepTime = (int) (1.0 / getFrequency() * 1e3);
        auto stepSize = getStepSize() * 3;
		
        auto targetReached = false;
        while(!targetReached) {

            targetReached = true;
            for(unsigned int i = 0; i < runnerState.size(); ++i) {
                if(std::isnan(targetPos.at(i))!= 0)
                    targetPos.at(i)=runnerState.at(i);
                auto sig = ((runnerState.at(i) - targetPos.at(i)) > 0) ? -1 : 1;
                if(fabs(runnerState.at(i) - targetPos.at(i)) > (stepSize * 2)) {
                    runnerState.at(i) += sig * stepSize;
                    targetReached = false;
                }
            }
				
            move(runnerState);
				
            std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			
        }
			

    }

    void Arm::moveHome() {
		
        std::vector<double> homeJoints;
        for(unsigned int i = 0; i < motors.size(); ++i)
            homeJoints.push_back(0.0);
			
        jointPtp(homeJoints);
		
    }

    double Arm::getStepSize() { return motors.front()->getStepSize(); }
    int Arm::getDegOfFreedom() { return motors.size() + 1; }
    double Arm::getFrequency() { return motors.front()->getFrequency(); }
    double Arm::getCycleTime() { return 1.0 / getFrequency(); }

