#include <uibk_robot_driver/arm_controller.hpp>
#include <math.h>
#include <time.h>

using namespace ROBOTIS;                                    // Uses functions defined in ROBOTIS namespace
using namespace std;
using namespace motor_controller;

void Arm::armLoop() {
	
	auto frequ = motors.front()->getFrequency();
	int sleepTime = (int) (1.0 / frequ * 1e3);
	while(keepThreadRunning) {
		for(auto motor : motors)
			motor->spinOnce();
		
		jointStateMutex.lock();

			currentJointState.clear();
			for(int i = 0; i < motors.size(); ++i) {
				
				auto motor = motors.at(i);
				
				if(i == 0 || i == 3)
					currentJointState.push_back(-motor->getCurrentState());
				else
					currentJointState.push_back(motor->getCurrentState());

			}
			
		jointStateMutex.unlock();
		if(!firstJointStateRetrieved)
			firstJointStateRetrieved = true;
		std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
		
	}
	
}

Arm::Arm(std::vector<int> ids, std::vector<motor_type> types, std::string portName, std::vector<std::pair<double, double> > jointLimits, double protocolVersion, int baudRate, double move_freq) {
	move_rate=move_freq;
	firstJointStateRetrieved = false;
	for(unsigned int i = 0; i < ids.size(); ++i) {
		auto id = ids.at(i);
		auto limit = jointLimits.at(i);
		motors.push_back(std::make_shared<Motor>(portName, id, types.at(i), protocolVersion, limit.first, limit.second, baudRate));
	}

	keepThreadRunning = false;
	moveArmOk=false;
	allowMovement=false;
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
	moveThread=std::make_shared<std::thread>(&Arm::gotoThread, this);
	moveThreadController=std::make_shared<std::thread>(&Arm::gotoThreadController, this);

}

void Arm::move(std::vector<double> nextJointPos) {

	if(nextJointPos.size() != (motors.size()))
		throw MotorException("number of joints doesn't fit the number of motors");

	auto js = getCurrentState();
	double velLimit = 0.0;
	double exceededDist = 0.0;
	if(checkDistance(js, nextJointPos, exceededDist, velLimit)) {
		for(unsigned int i = 0; i < nextJointPos.size(); ++i){
			if (std::isnan(nextJointPos.at(i)) == 0) {
				// swap sign
				if(i == 0 || i == 3)
					motors.at(i)->setNextState(-nextJointPos.at(i));
				else
					motors.at(i)->setNextState(nextJointPos.at(i));
			}
		}
	} else {
		std::cerr << "velocity limit exceeded (maxVel: " << velLimit << ", commandedVel: " << exceededDist << ")" << std::endl;
	}

}
void Arm::gotoArm(std::vector<double> targetPos) {
	moveMutex.lock();
	targetPosMove=targetPos;
	moveArmOk=true;
	moveMutex.unlock();

}
void Arm::gotoThreadController() {

	const int numberOfMoveOrdersAllowedBeforeNextCommand= 1;
	ros::Rate myRate(move_rate);
	int cnt=0;
	while(1){
		moveMutex.lock();
		if (cnt >numberOfMoveOrdersAllowedBeforeNextCommand)
			allowMovement=false;
		if (moveArmOk){
			moveArmOk=false;
			allowMovement=true;
			cnt=0;

		}
		moveMutex.unlock();
		cnt++;
		myRate.sleep();
	}

}

void Arm::gotoThread() {
	ros::Rate myRate(move_rate);

	while(1){

		auto jointState = getCurrentState();
		auto runnerState = jointState;
		auto targetReached = false;
		while(!targetReached && allowMovement) {

			targetReached = true;
			for(unsigned int i = 0; i < runnerState.size(); ++i) {
				auto stepSize = getStepSize(i) * 3;
				if(std::isnan(targetPosMove.at(i))!= 0)
					targetPosMove.at(i)=runnerState.at(i);
				auto sig = ((runnerState.at(i) - targetPosMove.at(i)) > 0) ? -1 : 1;
				if(fabs(runnerState.at(i) - targetPosMove.at(i)) > (stepSize * 2)) {
					runnerState.at(i) += sig * stepSize;
					targetReached = false;
				}
			}

			move(runnerState);

			myRate.sleep();

		}
					myRate.sleep();

	}
}

bool Arm::checkDistance(std::vector<double>& current, std::vector<double>& target, double& exceededDist, double& maxDist) {

	for(int i = 0; i < current.size(); ++i) {

		if(fabs(current.at(i) - target.at(i)) > motors.at(i)->getMaxVelLimit()) {
			exceededDist = fabs(current.at(i) - target.at(i));
			maxDist = motors.at(i )->getMaxVelLimit();
			std::cerr << "Motor " << i << " exceeded its limit by " << exceededDist << " (maxDist: " << maxDist << ")" <<  std::endl;
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
	
	auto targetReached = false;
	while(!targetReached) {

		targetReached = true;
		for(unsigned int i = 0; i < runnerState.size(); ++i) {
			auto stepSize = getStepSize(i) * 3;
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

double Arm::getStepSize(int motorId) { return motors.at(motorId)->getStepSize(); }
int Arm::getDegOfFreedom() { return motors.size() + 1; }
double Arm::getFrequency() { return motors.front()->getFrequency(); }
double Arm::getCycleTime() { return 1.0 / getFrequency(); }

