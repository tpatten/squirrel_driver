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



JointSensor::JointSensor(ros::NodeHandle &nh, unsigned int joint_id)
{
  stringstream param_name;
  
  param_name.clear();
  param_name << "joint_" << joint_id << "/sensor_calibration";
  if(!nh.getParam(param_name.str(), joint_sensor_zero_value_))
    throw runtime_error("missing sensor_calibration");
  setJointValueZero(joint_sensor_zero_value_);


  param_name.str("");
  param_name << "joint_" << joint_id << "/sensor_direction";
  if(!nh.getParam(param_name.str(), sensor_direction_))
    throw runtime_error("missing sensor_direction");

  param_name.str("");
  param_name << "joint_" << joint_id << "/sensor_range_min";
  if(!nh.getParam(param_name.str(), joint_value_min_))
    throw runtime_error("missing sensor_range_min");

  param_name.str("");
  param_name << "joint_" << joint_id << "/sensor_range_max";
  if(!nh.getParam(param_name.str(), joint_value_max_))
    throw runtime_error("missing sensor_range_max");

  joint_value_valid_ = true;

  //ROS_INFO("cali %d", sensor_direction_);
 
}








KCLHandController::	KCLHandController(std::string name)
{
  M_handle_ = 0;
  M_node_id_ = 1;
  M_error_code = 0;
}

KCLHandController:: ~KCLHandController()
{
 delete [] joints_sensor_;
 ROS_INFO("delete array");
}



// initilize the hand, open the epos2 controller 
bool KCLHandController::init()
{
	handIsInitialized_ = false;
  hasNewGoal_ = false;
	
  // ros setting 
  joint_value_sub_ = nh_.subscribe("joint_value_arduino", 1, &KCLHandController::jointSensorValueCB, this);
	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("active_joint_states", 1);
	
  // epos motor setting 

  stringstream param_name;

  param_name.clear();
  param_name << "hand_controller_device_name";
  if(!nh_.getParam(param_name.str(), M_DEVICENAME_NAME_))
    throw runtime_error("missing ROS parameter for hand_controller_device_name");

  param_name.str("");
  param_name << "hand_controller_protocol_stack_name";
  if(!nh_.getParam(param_name.str(), M_PROTOCAL_NAME_))
    throw runtime_error("missing ROS parameter for hand_controller_protocol_stack_name");

  param_name.str("");
  param_name << "hand_controller_interface_name";
  if(!nh_.getParam(param_name.str(), M_INTERFACE_))
    throw runtime_error("missing ROS parameter for hand_controller_interface_name");

  param_name.str("");
  param_name << "hand_controller_port_name";
  if(!nh_.getParam(param_name.str(), M_PORTNAME_))
    throw runtime_error("missing ROS parameter for hand_controller_port_name");

  param_name.str("");
  param_name << "hand_controller_baudrate";
  if(!nh_.getParam(param_name.str(), M_BAUDRATE_))
    throw runtime_error("missing ROS parameter for hand_controller_baudrate");

  if(openDevice())
    ROS_INFO("Deviced opened");

    
  joints_sensor_ = new JointSensor[NUM_JOINTS] { JointSensor(nh_,0),
                                                 JointSensor(nh_,1),
                                                 JointSensor(nh_,2),
                                                 JointSensor(nh_,3),
                                                 JointSensor(nh_,4)};
  /*
  JointSensor joints_sensor_[5]={ JointSensor(nh_,0),
                                  JointSensor(nh_,1),
                                  JointSensor(nh_,2),
                                  JointSensor(nh_,3),
                                  JointSensor(nh_,4)};
                                  
  
  JointMotor joints_motor_[5] = { JointMotor(nh_,M_handle_,0), 
                                  JointMotor(nh_,M_handle_,1),
                                  JointMotor(nh_,M_handle_,2),
                                  JointMotor(nh_,M_handle_,3),
                                  JointMotor(nh_,M_handle_,4)};
  
  */
   //ROS_INFO("cali sensor b %f", joints_sensor_[0].test());
 


  //if(closeDevice())
  //  ROS_INFO("Deviced closed");

  return true;

}

// build the controller communication and open controller, 
// return 1 means successful
// return 0 means faild 
bool KCLHandController::openDevice()
{
  bool if_controller_open = false;

  char* device_name = new char[255];
  char* protocol_stack_name = new char[255];
  char* interface_name = new char[255];
  char* port_name = new char[255];

  strcpy(device_name, M_DEVICENAME_NAME_.c_str());
  strcpy(protocol_stack_name, M_PROTOCAL_NAME_.c_str());
  strcpy(interface_name, M_INTERFACE_.c_str());
  strcpy(port_name, M_PORTNAME_.c_str());

  ROS_INFO("Open the Metahand Controller ");

  M_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &M_error_code);

  /* Print errors
  stringstream msg;
  msg << "M_handle" << M_handle_ << "'\n";
  msg << "error code" << M_error_code << "'\n";
  ROS_INFO("%s", msg.str().c_str());
  */

  if(M_handle_!=0 && M_error_code == 0)
  {
    unsigned int baudrate = 0;
    unsigned int timeout = 0;

    if(VCS_GetProtocolStackSettings(M_handle_, &baudrate, &timeout, &M_error_code)!=0)
    {
      if(VCS_SetProtocolStackSettings(M_handle_, M_BAUDRATE_, timeout, &M_error_code)!=0)
      {
        if(VCS_GetProtocolStackSettings(M_handle_, &baudrate, &timeout, &M_error_code)!=0)
        {
          if(M_BAUDRATE_==(int)baudrate)
          {
            if_controller_open = true;
          }
        }
      }
    }
  }
  
  else
  {
    M_handle_ = 0;
  }

  delete []device_name;
  delete []protocol_stack_name;
  delete []interface_name;
  delete []port_name;

  return if_controller_open;
}

// Joint motor 
JointMotor::JointMotor(ros::NodeHandle &nh, void *controller_handle, unsigned int joint_id)
:epos_handle_(controller_handle)
{
  stringstream param_name;
  
  param_name.clear();
  param_name << "joint_" << joint_id << "/node_id";
  if(!nh.getParam(param_name.str(), motor_node_id_))
    {}//throw runtime_error("missing motor node ID for motor ID: %d", motor_id);

  param_name.str("");
  param_name << "joint_" << joint_id << "/motor_direction";
  if(!nh.getParam(param_name.str(), motor_direction_))
    {}//throw runtime_error("missing motor node ID for motor ID: %d", motor_id);

  motor_position_ = 0;
  motor_velocity_ = 0;
  motor_move_current_ = 0;
  motor_holding_current_ = 0;
  motor_target_position_ = 0;

  max_peak_current_ = 0;
  motor_running_ = false;

  

}



// enable motor
bool JointMotor::enableMotor()
{
  int is_motor_enabled = false ;
  unsigned int error_code = 0; 
  int is_motor_fault = 0; // 1: Device is in fault state  0: Device is not in fault stat

  if(VCS_GetFaultState(epos_handle_, motor_node_id_, &is_motor_fault, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d fault state", motor_node_id_);
    return is_motor_enabled;
  }

  if(is_motor_fault) // if motor has fault. then clear motor fault
  {
    stringstream msg;
    msg << "clear fault, node = '" << motor_node_id_ << "'";
    ROS_INFO("%s", msg.str().c_str());

    if(VCS_ClearFault(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to clear motor ID: %d fault", motor_node_id_);
      return is_motor_enabled;
    }
  }
  
  // if no fault, then get the motor enable state
  if(VCS_GetEnableState(epos_handle_, motor_node_id_, &is_motor_enabled, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d enable state", motor_node_id_);
    return false;
  }

  // if motor is not enabled, then enable the motor
  if(!is_motor_enabled)
  {
    if(VCS_SetEnableState(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to set motor ID: %d enable state", motor_node_id_);
      return false;
    }
        
  }
  // if motor is enabled, then return ture
  return is_motor_enabled;
}


// disable motor
bool JointMotor::disableMotor()
{
  int is_motor_disabled = false ;
  unsigned int error_code = 0; 
  int is_motor_fault = 0; // 1: Device is in fault state  0: Device is not in fault stat

  if(VCS_GetFaultState(epos_handle_, motor_node_id_, &is_motor_fault, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d fault state", motor_node_id_);
    return is_motor_disabled;
  }

  if(is_motor_fault) // if motor has fault. then clear motor fault
  {
    stringstream msg;
    msg << "clear fault, node = '" << motor_node_id_ << "'";
    ROS_INFO("%s", msg.str().c_str());

    if(VCS_ClearFault(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to clear motor ID: %d fault", motor_node_id_);
      return is_motor_disabled;
    }
  }
  
  // if no fault, then get the motor disable state
  if(VCS_GetDisableState(epos_handle_, motor_node_id_, &is_motor_disabled, &error_code) == 0)
  {
    ROS_INFO("Faild to get motor ID: %d disable state", motor_node_id_);
    return false;
  }

  // if motor is not disabled, then disable the motor
  if(!is_motor_disabled)
  {
    if(VCS_SetDisableState(epos_handle_, motor_node_id_, &error_code) == 0)
    {
      ROS_INFO("Faild to set motor ID: %d disable state", motor_node_id_);
      return false;
    }
        
  }
  // if motor is disabled, then return ture
  return is_motor_disabled;
}

void JointMotor::reset()
{
  unsigned int error_code = 0;
  if(VCS_ResetDevice(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Hand joint reset node ID: %d faild", motor_node_id_);
    throw runtime_error("reset node faild"); 
  }
}


void JointMotor::activateVelocityMode()
{
  unsigned int error_code = 0;
  if(VCS_ActivateProfileVelocityMode(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Metahand: activate profile velocity mode faild, motor ID: %d ", motor_node_id_);
    throw runtime_error("MetaHand: faild to activate profile velocity mode"); 
  }
}

void JointMotor::moveWithVelocity(double velocity)
{
  unsigned int error_code = 0;
  long velocity_with_direction =  (long)velocity*motor_direction_;
  if(VCS_MoveWithVelocity(epos_handle_, motor_node_id_, velocity_with_direction, &error_code) == 0)
  {
    ROS_INFO("Metahand: move with velocity faild, motor ID: %d, velocity: %ld", motor_node_id_, velocity_with_direction);
    throw runtime_error("Metahand: failed to move with velocity"); 
  }
    
}

void JointMotor::haltVelocityMovement()
{
  unsigned int error_code = 0;
  if(VCS_HaltVelocityMovement(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Metahand: halt velocity movement faild, motor ID: %d", motor_node_id_);
    throw runtime_error("Metahand: failed to halt velocity movement"); 
  }
}

void JointMotor::activateCurrentMode()
{
  unsigned int error_code = 0;
  if(VCS_ActivateCurrentMode(epos_handle_, motor_node_id_, &error_code) == 0)
  {
    ROS_INFO("Metahand: activate current mode faild, motor ID: %d", motor_node_id_);
    throw runtime_error("Metahand: failed to activate current mode"); 
  }  
}

void JointMotor::setCurrent(double current)
{
  unsigned int error_code = 0;
  int actual_current = current*motor_direction_;
  if(VCS_SetCurrentMust(epos_handle_, motor_node_id_, (short)actual_current, &error_code) == 0)
  {
    ROS_INFO("Metahand: set current mode faild, motor ID: %d, current: %d", motor_node_id_, actual_current);
    throw runtime_error("Metahand: failed to set current"); 
  }  
}

double JointMotor::getCurrent()
{
  unsigned int error_code = 0;
  short current = 0.;
  if(VCS_GetCurrentIsAveraged(epos_handle_, motor_node_id_, &current, &error_code) == 0)
  {
    ROS_INFO("Metahand: get current faild, motor ID: %d", motor_node_id_);
    throw runtime_error("Metahand: failed to get motor current"); 
  }  
  return (double)current;
}



bool KCLHandController::closeDevice()
{
  bool is_device_closed = false;

  M_error_code = 0;

  ROS_INFO("Close device ing");

  if(VCS_CloseDevice(M_handle_, &M_error_code)!=0 && M_error_code == 0)
  {
    is_device_closed = true;
  }

  return is_device_closed;
}



void KCLHandController::jointSensorValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  
  static unsigned int seq = 0;

  if((int)msg->data.size() != NUM_JOINTS)
    throw runtime_error("KCLHandController: wrong number of joint position values received");

  // Whenever we get a position update, we also want a current/effort update.
  // joint_mutex_.lock();
  sensor_msgs::JointState joint_state;
  joint_state.position.resize(NUM_JOINTS);
  
  for(unsigned int i = 0; i < NUM_JOINTS; i++)
  {
    
    joint_state.position[i] = joints_sensor_[i].getSensorCalibratedValue(msg->data[i]);

  }

  //joint_mutex_.unlock();
  joint_state_pub_.publish(joint_state);
  //ROS_INFO("joint sensor done %f", joints_sensor_[1].joint_sensor_zero_value_);
  //ROS_INFO("joint sensor direction %d", joints_sensor_[1].sensor_direction_);


}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "kclhand_controller");
  
  KCLHandController controller("actuate_hand");
  
  if(!controller.init())
  	return 0;
  ros::Rate loop_rate(50); 
  
  ros::spin();	
  return 0;
}
