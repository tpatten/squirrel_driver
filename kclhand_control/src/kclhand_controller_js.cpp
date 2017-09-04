#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <kclhand_control/Definitions.h>
#include <kclhand_control/ActuateHandAction.h>
#include <kclhand_control/kclhand_controller_js.h>

using namespace std;



KCLHandController::	KCLHandController(std::string name)
{

}



bool KCLHandController:: init()
{
	handIsInitialized_ = false;
    hasNewGoal_ = false;
	joint_value_sub_ = nh_.subscribe("joint_value_arduino", 1, &KCLHandController::jointSensorValueCB, this);
	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("active_joint_states", 1);
	return true;
}




void KCLHandController::jointSensorValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  static unsigned int seq = 0;

  if((int)msg->data.size() != NUM_JOINTS)
    throw runtime_error("KCLHandController: wrong number of joint position values received");

  // Whenever we get a position update, we also want a current/effort update.
  // joint_mutex_.lock();

  sensor_msgs::JointState joint_state;

  for(unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    joint_state.position[i] = msg->data[i];
  }

  //joint_mutex_.unlock();
  // ... and publish the updates as joint state.
  joint_state_pub_.publish(joint_state);
}






int main(int argc, char** argv)
{
  ros::init(argc, argv, "kclhand_controller");
  KCLHandController controller("actuate_hand");
  ros::spin();
  return 0;
}
