/**
 * Controls the KCL Metahand
 * 
 * Provides action interfaces for opening, closing etc.
 * Follows the ROS tutorial on action servers and the Maxon tutorial on the
 * EPOS library.
 *
 * @author Michael Zillich <michael.zillich@tuwien.ac.at>
 * @date September 2016
 */

//#include <string>
//#include <string.h>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <kclhand_control/Definitions.h>
#include <kclhand_control/ActuateHandAction.h>
#include <kclhand_control/kclhand_controller.h>

using namespace std;

template<class T>
static T clamp(T val, T min, T max)
{
  if(val < min)
    val = min;
  else if(val > max)
    val = max;
  return val;
}

JointController::JointController(ros::NodeHandle &nh, void *epos_handle, int num)
: epos_handle_(epos_handle)
{
  stringstream param_name;

  param_name.clear();
  param_name << "joint_" << num << "/node_id";
  if(!nh.getParam(param_name.str(), node_id_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/node_id");

  param_name.str("");
  param_name << "joint_" << num << "/name";
  if(!nh.getParam(param_name.str(), name_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/name");

  param_name.str("");
  param_name << "joint_" << num << "/sensor_direction";
  if(!nh.getParam(param_name.str(), sensor_direction_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/sensor_direction");

  param_name.str("");
  param_name << "joint_" << num << "/motor_direction";
  if(!nh.getParam(param_name.str(), motor_direction_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/motor_direction");

  double zero = 0.;
  param_name.str("");
  param_name << "joint_" << num << "/sensor_calibration";
  if(!nh.getParam(param_name.str(), zero))
    throw runtime_error("JointController: missing ROS parameter for joint_X/sensor_calibration");
  calibration_.setZero(zero);

  param_name.str("");
  param_name << "joint_" << num << "/max_velocity";
  if(!nh.getParam(param_name.str(), max_velocity_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/max_velocity");

  param_name.str("");
  param_name << "joint_" << num << "/max_current";
  if(!nh.getParam(param_name.str(), max_sustained_current_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/max_current");

  param_name.str("");
  param_name << "joint_" << num << "/max_peak_current";
  if(!nh.getParam(param_name.str(), max_peak_current_))
    throw runtime_error("JointController: missing ROS parameter for joint_X/max_eeak_current");

  position_ = 0.;
  current_ = 0.;
  target_ = 0.;
  velocity_ = 0.;
  motor_running_ = false;
  target_reached_ = false;

  startMotor();
}

JointController::~JointController()
{
  stopMotor();
  reset();
}

void JointController::getErrors()
{
  // number of actual errors
  unsigned char nbOfDeviceError = 0;
  // error code from function
  unsigned int error_code;
  //error code from device
  unsigned int deviceErrorCode = 0;

  // get number of device errors
  if(VCS_GetNbOfDeviceError(epos_handle_, node_id_, &nbOfDeviceError, &error_code))
  {
    //  read device error code
    for(unsigned char errorNumber = 1; errorNumber <= nbOfDeviceError; errorNumber++)
    {
      if(VCS_GetDeviceErrorCode(epos_handle_, node_id_, errorNumber, &deviceErrorCode, &error_code))
      {
        ROS_INFO("Maxon Motor EPOS: joint %30s (node id %d) error # %d",
                  name_.c_str(), (int)node_id_, (int)deviceErrorCode); 
      }
      else
        ROS_INFO("Maxon Motor EPOS: failed to get failures, really sucks");
    }
  }
  else
    ROS_INFO("Maxon Motor EPOS: failed to get failures, really sucks");
}

void JointController::startMotor()
{
  getErrors();

  int is_fault = 0;
  int is_enabled = 0;
  unsigned int error_code;

  if(VCS_GetFaultState(epos_handle_, node_id_, &is_fault, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to get node fault state");
  if(is_fault)
  {
    std::stringstream msg;
    msg << "clear fault, node = '" << node_id_ << "'";
    ROS_INFO("%s", msg.str().c_str());

    if(VCS_ClearFault(epos_handle_, node_id_, &error_code) == 0)
      throw runtime_error("Maxon Motor EPOS: failed to clear node fault state");
  }

  if(VCS_GetEnableState(epos_handle_, node_id_, &is_enabled, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to get node enable state");
  if(!is_enabled)
  {
    if(VCS_SetEnableState(epos_handle_, node_id_, &error_code) == 0)
      throw runtime_error("Maxon Motor EPOS: failed to enable node");
  }
}

void JointController::stopMotor()
{
  unsigned int error_code;
  if(VCS_SetDisableState(epos_handle_, node_id_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to disable node");
}

void JointController::reset()
{
  unsigned int error_code;
  if(VCS_ResetDevice(epos_handle_, node_id_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to reset node");
}

void JointController::activateVelocityMode()
{
  unsigned int error_code = 0;
  // apparently I have to call this every time to clear errors
  startMotor();
  // TODO: why profile mode? -> check
  if(VCS_ActivateProfileVelocityMode(epos_handle_, node_id_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set profile velocity mode");
}

void JointController::moveWithVelocity(double velocity)
{
  unsigned int error_code = 0;
  if(VCS_MoveWithVelocity(epos_handle_, node_id_, (long)velocity*motor_direction_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to move with velocity");
}

void JointController::haltVelocityMovement()
{
  unsigned int error_code = 0;
  if(VCS_HaltVelocityMovement(epos_handle_, node_id_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to halt velocity movement");
}

void JointController::activateCurrentMode()
{
  unsigned int error_code = 0;
  if(VCS_ActivateCurrentMode(epos_handle_, node_id_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to activate current mode");
}

void JointController::setCurrent(double current)
{
  unsigned int error_code = 0;
  if(VCS_SetCurrentMust(epos_handle_, node_id_, (short)current*motor_direction_, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set current");
}

double JointController::getCurrent()
{
  unsigned int error_code = 0;
  short current = 0.;
  if(VCS_GetCurrentIsAveraged(epos_handle_, node_id_, &current, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to get current");
  return (double)current;
}

bool JointController::startMoveToTarget(double target_position)
{
  static bool first_call = true;  // HACK: only move the palm ONCE with the first move

  target_ = target_position;
  if(!isAtPosition(target_, 0))
  {
    activateVelocityMode();
    // set motion direction depending on where target is relative to current position
    int direction = 0;
    if(position_ <  target_)
      direction = 1;
    else if(position_ > target_)
      direction = -1;
    // else: leave 0
    velocity_ = direction*max_velocity_;
    moveWithVelocity(velocity_);
    motor_running_ = true;
    target_reached_ = false;

    ROS_INFO("START      %30s: pos: %5.3lf  target: %5.3lf  vel: %5.0lf",
             name_.c_str(), position_, target_, velocity_);
  }
  else
  {
    velocity_ = 0;
    motor_running_ = false;
    target_reached_ = true;
  }
  return motor_running_;
}

bool JointController::keepMovingToTarget()
{
  if(motor_running_)
  {
    // if target reached, current limit exceeded or timeout occured then stop this finger
    target_reached_ = isAtPosition(target_, velocity_ > 0 ? 1 : -1);
    bool peak_current_exceeded = fabs(current_) >= max_peak_current_;
    bool avg_current_exceeded = avg_current_.get() >= max_sustained_current_;

    if(peak_current_exceeded || avg_current_exceeded || target_reached_)
    {
      if(target_reached_)
        ROS_INFO("REACH POS  %30s: target: %5.3lf, pos: %5.3lf",
                  name_.c_str(), target_, position_);
      if(peak_current_exceeded)
        ROS_INFO("REACH PEAK %30s: current: %5.0lf mA, pos: %5.3lf",
                  name_.c_str(), current_, position_);
      if(avg_current_exceeded)
        ROS_INFO("REACH CUR  %30s: current: %5.0lf mA, pos: %5.3lf",
                  name_.c_str(), avg_current_.get(), position_);
      motor_running_ = false;
    }
  }
  return motor_running_;
}

void JointController::stopMoveToTarget()
{
  velocity_ = 0;
  haltVelocityMovement();
  ROS_INFO("STOP       %30s: pos: %5.3lf", name_.c_str(), position_);
}

KCLHandController::KCLHandController(std::string name)
: as_(nh_, name, boost::bind(&KCLHandController::actuateHandCB, this, _1), false)
{
  key_handle_ = 0;
  
  // NOTE: Maybe we should get these from parameters. But these seem pretty
  // fixed. So for the time being this seems OK.
  device_name_ = "EPOS2";
  protocol_stack_name_ = "MAXON SERIAL V2";
  interface_name_ = "USB";
  port_name_ = "USB0";
  baudrate_ = 1000000;

  openDevice();

  for(size_t i = 0; i < NUM_JOINTS; i++)
    joints_.push_back(JointController(nh_, key_handle_, i));

  // TODO: wait for finger sensor topic and openFingers();

  joint_value_sub_= new ros::Subscriber(nh_.subscribe("joint_value_arduino", 1, &KCLHandController::jointValueCB, this));
  joint_state_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::JointState>("active_joint_states", 1));

  as_.start();
}

KCLHandController::~KCLHandController()
{
  delete joint_value_sub_;
  delete joint_state_pub_;
  delete move_finger_srv_;
  closeDevice();
}

/**
 * TODO: fill in the action feedback with sensible information
 * TODO: handle action preempt
 */
void KCLHandController::actuateHandCB(const kclhand_control::ActuateHandGoalConstPtr &goal)
{
  ROS_INFO("Starting hand actuation action");

  kclhand_control::ActuateHandFeedback feedback;
  kclhand_control::ActuateHandResult result;
  bool succeeded = false;

  if(goal->command == kclhand_control::ActuateHandGoal::CMD_OPEN)
  {
    succeeded = openFingers(goal->force_limit);
  }
  else if(goal->command == kclhand_control::ActuateHandGoal::CMD_CLOSE)
  {
    succeeded = closeFingers(goal->force_limit);
  }
  else if(goal->command == kclhand_control::ActuateHandGoal::CMD_MOVE_FINGER)
  {
    if(goal->finger >= 0 && goal->finger < NUM_JOINTS)
    {
      bool reached = false;
      succeeded = moveFinger(goal->finger, 1., goal->position, reached);
    }
  }
  else if(goal->command == 3)  // HACK: make a constant
  {
    for(size_t i = 0; i < joints_.size(); i++)
      joints_[i].reset();
  }
  else
  {
    ROS_ERROR("received invalid hand command");
  }

  if(succeeded)
    as_.setSucceeded(result);
  else
    as_.setAborted(result);
}

void KCLHandController::jointValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  static unsigned int seq = 0;

  if((int)msg->data.size() != NUM_JOINTS)
    throw runtime_error("KCLHandController: wrong number of joint position values received");

  sensor_msgs::JointState state;
  state.header.seq = seq++;
  state.header.stamp = ros::Time::now();
  state.header.frame_id = "hand_base_link";
  state.position.assign(NUM_JOINTS, 0.);
  state.velocity.assign(NUM_JOINTS, 0.);
  state.effort.assign(NUM_JOINTS, 0.);
  state.name.resize(NUM_JOINTS);

  // Whenever we get a position update, we also want a current/effort update.
  joint_mutex_.lock();
  for(size_t i = 0; i < NUM_JOINTS; i++)
  {
    joints_[i].updateCurrent();
    joints_[i].updatePositionFromRawSensor(msg->data[i]);

    state.name[i] = joints_[i].name();
    state.position[i] = joints_[i].position();
    state.velocity[i] = joints_[i].velocity();
    state.effort[i] = joints_[i].current();
  }
  joint_mutex_.unlock();
  // ... and publish the updates as joint state.
  joint_state_pub_->publish(state);
}

void KCLHandController::openDevice()
{
  stringstream msg;
  msg << "Opening Maxon Motor EPOS device: :\n";
  msg << "device name         = '" << device_name_ << "'\n";
  msg << "protocal stack name = '" << protocol_stack_name_ << "'\n";
  msg << "interface name      = '" << interface_name_ << "'\n";
  msg << "port name           = '" << port_name_ << "'\n";
  msg << "baudrate            = " << baudrate_;
  ROS_INFO("%s", msg.str().c_str());

  // NOTE: these temporary strings are necessary as VCS_OpenDevice() takes char*
  // but .c_str() returns const char*
  char device_name[256];
  char protocol_stack_name[256];
  char interface_name[256];
  char port_name[256];
  strncpy(device_name, device_name_.c_str(), 256);
  strncpy(protocol_stack_name, protocol_stack_name_.c_str(), 256);
  strncpy(interface_name, interface_name_.c_str(), 256);
  strncpy(port_name, port_name_.c_str(), 256);
  unsigned int error_code;
  key_handle_ = VCS_OpenDevice(device_name, protocol_stack_name, interface_name, port_name, &error_code);

  bool ok = false;
  if(key_handle_ != 0 && error_code == 0)
  {
    unsigned int check_baudrate = 0;
    unsigned int timeout = 0;

    // NOTE: no idea why they do this strange dance. There should be a better way
    // using VCS_GetBaudrateSelection().
    // But this seems to work fine, so leave it for now.
    if(VCS_GetProtocolStackSettings(key_handle_, &check_baudrate, &timeout, &error_code) != 0)
    {
      if(VCS_SetProtocolStackSettings(key_handle_, baudrate_, timeout, &error_code) != 0)
      {
        if(VCS_GetProtocolStackSettings(key_handle_, &check_baudrate, &timeout, &error_code) != 0)
        {
          if(baudrate_ == check_baudrate)
          {
            ok = true;
          }
        }
      }
    }
  }
  if(!ok)
    throw runtime_error("Maxon Motor EPOS: failed to open device");
}

void KCLHandController::closeDevice()
{
  unsigned int error_code;
  ROS_INFO("Closing Maxon Motor EPOS device");
  if(VCS_CloseDevice(key_handle_, &error_code) == 0 || error_code != 0)
    throw runtime_error("Maxon Motor EPOS: failed to close device");
}

bool KCLHandController::openFingers(float rel_current_limit)
{
  std::vector<double> target_positions(NUM_JOINTS);
  bool all_target_positions_reached = false;
  target_positions[RIGHT_PALM] = deg_to_rad(30.);
  target_positions[LEFT_PALM] = deg_to_rad(30.);
  target_positions[RIGHT_FINGER] = deg_to_rad(-70.);
  target_positions[MIDDLE_FINGER] = deg_to_rad(-70.);
  target_positions[LEFT_FINGER] = deg_to_rad(-70.);
  bool succeeded = moveFingers(rel_current_limit, target_positions, all_target_positions_reached);
  // Now slightly close the fingers to pre-tension the tendons
  if(succeeded)
  {
    target_positions[RIGHT_FINGER] = deg_to_rad(-60.);
    target_positions[MIDDLE_FINGER] = deg_to_rad(-60.);
    target_positions[LEFT_FINGER] = deg_to_rad(-60.);
    succeeded = moveFingers(rel_current_limit, target_positions, all_target_positions_reached);
  }
  return succeeded;
}

bool KCLHandController::closeFingers(float rel_current_limit)
{
  std::vector<double> target_positions(NUM_JOINTS);
  bool all_target_positions_reached = false;
  target_positions[RIGHT_PALM] = deg_to_rad(30.);
  target_positions[LEFT_PALM] = deg_to_rad(30.);
  target_positions[RIGHT_FINGER] = deg_to_rad(-15.);
  target_positions[MIDDLE_FINGER] = deg_to_rad(20.);
  target_positions[LEFT_FINGER] = deg_to_rad(-15.);
  bool succeeded = moveFingers(rel_current_limit, target_positions, all_target_positions_reached);
  // if targets not reached, then some joints ran into their current limit, i.e. we are probably
  // holding an object.
  /* HACK: leave out for now:
  if(!all_target_positions_reached)
    keepTightGrip();*/
  return succeeded;
}

/**
 * TODO: coordinate finger movment to arrive at same psition simultaneously
 *       (essentially: the slowest finger pauses the others)
 */
bool KCLHandController::moveFingers(float rel_current_limit, std::vector<double> &target_positions,
                                    bool &all_target_positions_reached)
{
  ROS_INFO("start moving fingers");

  // how many motors are still running
  int num_motors_running = 0;
  // count how many joints reached the target (vs. reaching their current limits)
  int num_targets_reached = 0;

  joint_mutex_.lock();
  for(int i = 0; i < NUM_JOINTS; i++)
    if(joints_[i].startMoveToTarget(target_positions[i]))
      num_motors_running++;
  joint_mutex_.unlock();

  ros::Rate rate(CONTROL_LOOP_FREQ_HZ);
  int timeout_cnt = CONTROL_LOOP_TIMEOUT_S*CONTROL_LOOP_FREQ_HZ;
  while(num_motors_running > 0 && timeout_cnt > 0)
  {
    joint_mutex_.lock();
    for(int i = 0; i < NUM_JOINTS; i++)
    {
      if(joints_[i].isRunning())
      {
        if(!joints_[i].keepMovingToTarget())
        {
          joints_[i].stopMoveToTarget();
          num_motors_running--;
          if(joints_[i].targetReached())
            num_targets_reached++;
        }
      }
    }
    joint_mutex_.unlock();
    timeout_cnt--;
    rate.sleep();
  }
  if(timeout_cnt <= 0)
  {
    for(int i = 0; i < NUM_JOINTS; i++)
      if(joints_[i].isRunning())
        joints_[i].stopMoveToTarget();
    ROS_INFO("timeout");
  }
  all_target_positions_reached = (num_targets_reached == (int)target_positions.size());

  ROS_INFO("done moving fingers");

  return timeout_cnt > 0;
}

bool KCLHandController::moveFinger(int joint_idx, float rel_current_limit, double target_position,
                                   bool &target_position_reached)
{
  ROS_INFO("start moving single finger");

  bool running = false;
  joint_mutex_.lock();
  running  = joints_[joint_idx].startMoveToTarget(target_position);
  joint_mutex_.unlock();

  ros::Rate rate(CONTROL_LOOP_FREQ_HZ);
  int timeout_cnt = CONTROL_LOOP_TIMEOUT_S*CONTROL_LOOP_FREQ_HZ;
  while(running && timeout_cnt > 0)
  {
    joint_mutex_.lock();
    if(!joints_[joint_idx].keepMovingToTarget())
    {
      joints_[joint_idx].stopMoveToTarget();
      running = false;
    }
    joint_mutex_.unlock();
    timeout_cnt--;
    rate.sleep();
  }
  if(timeout_cnt <= 0)
  {
    joints_[joint_idx].stopMoveToTarget();
    ROS_INFO("timeout");
  }

  ROS_INFO("done moving single finger");

  return timeout_cnt > 0;
}

#if 0
void KCLHandController::keepTightGrip()
{
  ROS_INFO("tightening grip");

  for(int i = 0; i < node_ids_.size(); i++)
  {
    // NOTE: the palm joints always stay where they are, only the fingers squeeze
    if(i != RIGHT_PALM && i != LEFT_PALM)
    {
      activateCurrentMode(i);
      setCurrent(i, MAX_SUSTAINED_CURRENT);
    }
  }
}

void KCLHandController::loosenGrip()
{
  ROS_INFO("loosening grip");

  for(int i = 0; i < node_ids_.size(); i++)
  {
    // NOTE: the palm joints always stay where they are, only the fingers squeeze
    if(i != RIGHT_PALM && i != LEFT_PALM)
    {
      activateCurrentMode(i);
      setCurrent(i, 0.);
    }
  }
}

/**
 * TODO: Don't know if this works.
 */
void KCLHandController::holdPosition(int joint)
{
  joint_mutex_.lock();

  int current_motor_encoder_pos = 0;
  unsigned int error_code = 0;

  ROS_INFO("holding posotion for motor %d", joint);

  if(VCS_ActivatePositionMode(key_handle_, node_ids_[joint], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set position mode");
  if(VCS_GetPositionIs(key_handle_, node_ids_[joint], &current_motor_encoder_pos, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to get position");
  if(VCS_SetPositionMust(key_handle_, node_ids_[joint], (long)current_motor_encoder_pos, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set position");

  joint_mutex_.unlock();
}

/**
 * TODO: No idea how to do this,
 */
void KCLHandController::releasePosition(int joint)
{

}
#endif

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kclhand_controller");

  KCLHandController controller("actuate_hand");
  ros::spin();

  return 0;
}
