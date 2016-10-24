#include "uibk_robot_driver/robot_controller.hpp"

using namespace std;
using namespace motor_controller;

RobotController::RobotController(ros::NodeHandle& node, double control_freq) {
    controller_freq = control_freq;
    myNode = node;
    armExists = false;
    baseExists = false;
    receivedFirstSkinPacket = false;
}

void RobotController::initBase() {

   myBase = std::shared_ptr<BaseController>(new BaseController (myNode,controller_freq));
   baseExists = true;
   receivedFirstSkinPacket = false;

}

void RobotController::initArm(std::vector<int> ids, std::vector<motor_type> types, std::vector< std::pair<double, double> > jointLimits) {

   myNode.param("portName", portName, std::string("/dev/ttyArm"));
   myNode.param("protocolVersion", protocolVersion, 2.0);
   myNode.param("baudRate", baudRate, 3000000);
   myArm = std::shared_ptr<Arm> (new Arm(ids, types, portName, jointLimits, protocolVersion, baudRate, controller_freq));
   myArm->initialize();
   myArm->runArm();
   armExists = true;
   receivedFirstSkinPacket = false;

   skinBumper = myNode.subscribe("/airskin/arm_bumper", 2, &RobotController::skinCallback, this);
   
}

// the topic publishes false if nothing is pressed, and switches to true if something is pressed
void RobotController::skinCallback(const std_msgs::Bool& skinReply) {

    receivedFirstSkinPacket = true;
    if(!skinReply.data) {
        skinMutex.lock();
            skinTic.tic("skin");
        skinMutex.unlock();
    }

}

vector<double> RobotController::getCurrentStates() {

    vector<double> allStates,temp,temp2;
    if (baseExists){
    temp=myBase->getCurrentState();
    allStates.insert(allStates.end(),temp.begin(),temp.end());
    }
    else{

        temp={DONTCARE,DONTCARE,DONTCARE};
        allStates.insert(allStates.end(),temp.begin(),temp.end());
    }

    if(armExists){
    temp2=myArm->getCurrentState();
    allStates.insert(allStates.end(),temp2.begin(),temp2.end());
    }

    else{
        temp2={DONTCARE,DONTCARE,DONTCARE,DONTCARE,DONTCARE};
        allStates.insert(allStates.end(),temp2.begin(),temp2.end());
    }

    return allStates;
}

void RobotController::moveAll(vector<double> targetStates) {

    if(receivedFirstSkinPacket) {
        double timeSinceLastOk = skinTic.toc("skin");
        if(timeSinceLastOk < 0.2) {
            if(baseExists)
                myBase->moveBase(targetStates.at(0), targetStates.at(1), targetStates.at(2)) ;
            if(armExists){
                vector<double> temp = vector<double> (targetStates.begin() + 3, targetStates.end());
                myArm->move(temp);
            }
        } else {
            // if the skin is not ok (either timed out or is pressed), ignore the move commands
        }
    }

}

void RobotController::gotoAll(vector<double> targetStates) {
  if (baseExists)
    myBase->gotoBase(targetStates.at(0),targetStates.at(1),targetStates.at(2)) ;
  if (armExists){
      vector<double> temp = vector<double> (targetStates.begin()+3,targetStates.end());
      myArm->gotoArm(temp);
  }
}

void RobotController::ptpAll(vector<double> targetStates) {

	if (baseExists)
		myBase->ptp(targetStates.at(0),targetStates.at(1),targetStates.at(2)) ;
		
	if (armExists) {
		
		vector<double> temp = vector<double> (targetStates.begin()+3,targetStates.end());
		myArm->jointPtp(temp);

	}
}

double RobotController::getArmFrequency() { return myArm->getFrequency();}
double RobotController::getArmCycleTime() { return myArm->getCycleTime();}
double RobotController::getArmMaxStepPerCycle() { return myArm->getMaxStepPerCycle();}
int RobotController::getDegOfFreedom() {return 8;}

void RobotController::shutdown() {

    myArm->shutdown();
    myArmThread->join();

}
