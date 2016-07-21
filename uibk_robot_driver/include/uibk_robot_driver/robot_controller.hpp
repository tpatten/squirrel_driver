#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP
#include <iostream>
#include <uibk_robot_driver/base_controller.hpp>
#include <uibk_robot_driver/arm_controller.hpp>

#define DONTCARE nan("1")
using namespace std;
class RobotController
{
  std::shared_ptr<BaseController> myBase;
  std::shared_ptr<Arm> myArm;
  std::shared_ptr<std::thread> myArmThread;
  bool baseExists,armExists;
  double controller_freq;
  ros::NodeHandle myNode;
//  std::vector<double> currentRobotStates;
std::string portName; double protocolVersion; int baudRate;

public:

  //RobotController(ros::NodeHandle& node, std::shared_ptr<BaseController> base,  std::shared_ptr<Arm> arm);
  RobotController(ros::NodeHandle& node);
  void initBase();
  void initArm( std::vector<int> ids,std::vector< std::pair<double, double> > jointLimits );
  void moveAll(vector<double> targetStates);
  void ptpAll(vector<double> targetStates);
  vector<double> getCurrentStates();


  // For arm

  double getArmStepSize();
  double getArmFrequency();
  double getArmCycleTime();
  double getArmMaxStepPerCycle();
  int getDegOfFreedom();

 void shutdown();

};

#endif // ROBOT_CONTROLLER_HPP
