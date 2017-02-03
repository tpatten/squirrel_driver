/**
 * Controls the KCL Metahand
 * 
 * Provides action interfaces for opening, closing etc.
 * Follows the ROS tutorial on action servers and the Maxon tutorial on the
 * EPOS library.
 *
 * Joint angles are counted like this:
 * LEFT finger: 0 = perpendicular to palm
 *              - outwards up to to mechanical limit of about -80 deg
 *              + inwards up to mechanical limit of about 60 deg
 * RIGHT finger: same
 * MIDDLE finger: same
 * LEFT_PALM: 0 = flat
 *         - upwards
 *         + downwards
 * RIGHT_PALM: same
 *
 * NOTE: Motors can be mounted differently, i.e. somtimes positive velocity is
 * inwards, sometimes outwards.
 *
 * TODO:
 * - check how threading works with action server: what if called multiple times?
 * - proper exception handling with clean shutdown
 * - todo: add "positions reached" to result, so the caller knows that something was probably grasped
 *
 * @author Michael Zillich <michael.zillich@tuwien.ac.at>
 * @date September 2016
 */

#include <string>
#include <sstream>
#include <stdexcept>
#include <string.h>
#include <map>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/concept_check.hpp>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <kclhand_control/ActuateHandAction.h>
#include <kclhand_control/Definitions.h>

using namespace std;

static bool pre_grasp = true;  // HACK

template<class T>
static T clamp(T val, T min, T max)
{
  if(val < min)
    val = min;
  else if(val > max)
    val = max;
  return val;
}

static double deg_to_rad(double deg)
{
  return deg*M_PI/180.;
}

class RunningAverage
{
private:
  // ring buffer
  std::vector<double> values_;
  // position of oldest value in ring buffer
  size_t oldest_value_idx_;
  // sum of all values in ring buffer
  double sum_values_;
  // adding the first value is treated differently
  bool first_val_;

public:
  RunningAverage(size_t size = 10)
  {
    values_.assign(size, 0);
    sum_values_ = 0.;
    oldest_value_idx_ = 0;
    first_val_ = true;
  }

  void add(double value)
  {
    if(!first_val_)
    {
      sum_values_ -= values_[oldest_value_idx_];
      values_[oldest_value_idx_] = value;
      oldest_value_idx_ = (oldest_value_idx_ + 1) % values_.size();
      sum_values_ += value;
    }
    else
    {
      values_.assign(values_.size(), value);
      sum_values_ = values_.size()*value;
      first_val_ = false;
    }
  }

  double get()
  {
    return sum_values_/(double)values_.size();
  }
};

class MedianFilter
{
private:
  // ring buffer
  std::vector<double> values_;
  // position of oldest value in ring buffer
  size_t oldest_value_idx_;
  // adding the first value is treated differently
  bool first_val_;

public:
  MedianFilter(size_t size)
  {
    values_.assign(size, 0);
    oldest_value_idx_ = 0;
    first_val_ = true;
  }

  void add(double value)
  {
    if(!first_val_)
    {
      values_[oldest_value_idx_] = value;
      oldest_value_idx_ = (oldest_value_idx_ + 1) % values_.size();
    }
    else
    {
      values_.assign(values_.size(), value);
      first_val_ = false;
    }
  }

  double get()
  {
    std::vector<double> tmp = values_;
    sort(tmp.begin(), tmp.end());
    return tmp[tmp.size()/2];
  }
};

/**
 * Map from raw sensor values to degrees via a linear function given as
 * y = k*x + b
 */
class SensorCalibration
{
public:
  double k;
  double b;

  SensorCalibration(double zero = 0.)
  {
    // NOTE: slope k is common to all sensors
    k = 9./28.;
    b = -k*zero;
  }
  void setZero(double zero)
  {
    b = -k*zero;
  }
  double rawValueToDegrees(double raw)
  {
    return k*raw + b;
  }
  double rawValueToRadians(double raw)
  {
    return deg_to_rad(k*raw + b);
  }
};

class KCLHandController
{
private:
  /// 3 fingers and 2 palms
  static const int NUM_MOTORS = 5;
  /// default (safe and nice) velocity in rpm when opening and closing fingers
  static const long FINGER_VELOCITY = 1000;
  /// the palm joints need slower motor rotations, as they are more direct
  static const long PALM_VELOCITY = 300;
  /** absolute maximum current in mA that we allow in order not to break the motors
   * NOTE: According to Maxon datasheet for Motor Part number 216014 the maximum
   * continuous current is 189 mA. During closing we almost always surpass that,
   * but not for a long time. So that should be ok.
   */ 
  static const short MAX_SUSTAINED_CURRENT = 300;
  /// the short time peak current can be higher
  static const short MAX_PEAK_CURRENT = 400;
  /// time in ms over which we averge motor currents for limit check
  static const int CURRENT_AVERAGING_TIME_MS = 500;
  /// threshold (in rad) below which two joint positions are regarded equal
  static const double POSITION_REACHED_THRESHOLD = 0.035;
  /** joint numbers
    * NOTE: the EPOS node id is i*2 + 1
    * NOTE: the nominal directions are:
    *   + is inwards (= up for palm joint) = CLOSE 
    *   - is outwards (= down for palm joint) = OPEN
   */ 
  static const int RIGHT_PALM    = 0;  // node id 1, sensor + is down, motor + is down
  static const int LEFT_PALM     = 1;  // node id 3, sensor + is up,   motor + is up
  static const int RIGHT_FINGER  = 2;  // node id 5, sensor + is out,  motor + is out
  static const int MIDDLE_FINGER = 3;  // node id 7, sensor + is out,  motor + is out
  static const int LEFT_FINGER   = 4;  // node id 9, sensor + is out,  motor + is in
  /// frequency of the control loop
  static const int CONTROL_LOOP_FREQ_HZ = 100;
  /// timeout in case something weird goes wrong with the motors
  static const int CONTROL_LOOP_TIMEOUT_S = 5;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<kclhand_control::ActuateHandAction> as_; 
  ros::Subscriber *joint_value_sub_;
  ros::Publisher *joint_state_pub_;
  ros::ServiceServer *move_finger_srv_;

  /// Stuff below is for connecting to EPOS device
  std::string device_name_;
  std::string protocol_stack_name_;
  std::string interface_name_;
  std::string port_name_;
  unsigned int baudrate_;
  void *key_handle_;

  /// Motor IDs are not necessarily from 0 to NUM_MOTORS. In our case they are
  /// 2*i + 1
  std::vector<unsigned short> node_ids_;
  /// calibration paramerers for joint angle sensors
  std::vector<SensorCalibration> sensor_calibrations_;
  /// current joint state
  sensor_msgs::JointState joint_state_;
  /// -1 or 1, depending on how the sensors are mounted
  std::vector<long> sensor_directions_;
  /// -1 or 1, depending on how the motors are mounted
  std::vector<long> motor_directions_;
  /// velocities to use in velocity mode
  std::vector<long> joint_velocities_;
  // joint values are extrremely noisy, so need to be filtered
  std::vector<MedianFilter> joint_value_filters_;
  /// mutex to protect joint positions (written to in subscriber callback, read
  /// from in control loop)
  boost::mutex joint_value_mutex_;
  /// mutex to protect accessing motors
  boost::mutex motor_mutex_;

public:
  KCLHandController(std::string name);
  ~KCLHandController();

  /**
   * Callback for the action.
   */
  void actuateHandCB(const kclhand_control::ActuateHandGoalConstPtr &goal);
  /**
   * Callback for the joint state topic.
   */
  void jointValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg);

private:
  /**
   * Open Maxon Motor EPOS device. In our case the CAN controller attached via
   * USB.
   */
  void openDevice();
  /**
   * Close the Maxon Motor EPOS device.
   */
  void closeDevice();
  /**
   * Get motor errors.
   */
  void getErrors();
  /**
   * Start motors: clear failure flags and enable.
   */
  void startMotors();
  /**
   * Stop motors: disable nodes.
   */
  void stopMotors();
  /**
   * Activate velocity mode for the given joint.
   */
  void activateVelocityMode(int joint_idx);
  /**
   * Move given joint with given velocity, in [RPM].
   */
  void moveWithVelocity(int joint_idx, double velocity);
  /**
   * Stop moving given joint (in velocity mode).
   */
  void haltVelocityMovement(int joint_idx);
  /**
   * Activate current mode for the given joint.
   */
  void activateCurrentMode(int joint_idx);
  /**
   * Set current for given joint.
   */
  void setCurrent(int joint_idx, double current);
  /**
   * Get current of given joint, in [mA].
   */
  double getCurrent(int joint_idx);
  /**
   * Open fiogers, observing a current limit given as ratio of MAX_CURRENT.
   */
  bool openFingers(float rel_current_limit);
  /**
   * Close fiogers, observing a current limit given as ratio of MAX_CURRENT.
   */
  bool closeFingers(float rel_current_limit);
  /**
   * Move fingers to a given position.
   * Common function called by open and close.
   */
  bool moveFingers(float rel_current_limit, std::vector<double> &target_positions, bool &all_target_positions_reached);
  /**
   * Move a single finger to a given position, observing a current limit.
   */
  bool moveFinger(int joint_idx, float rel_current_limit, double target_position, bool &target_position_reached);
  /**
   * When a close was successfull and we know that we are holding something,
   * then we activate current control mode to keep the object squeezed between
   * the fingers.
   */
  void keepTightGrip();
  /**
   * Loosen grip, i.e. set motor corrents to 0.
   */
  void loosenGrip();
  /**
   * Check if a joint position has reached the target.
   * Direction takes into account the current moving direction (if any) of the
   * joint, to avoid that the target is overshot if the sample interval is too
   * coarse.
   * direction: 0 .. just compare positions
   *            -1 .. check if target reached from above (going down)
   *            1 .. check if target reached from below (gping up)
   */
  bool isAtPosition(double pos, double target, int direction);
  void holdPosition(int joint);
  void releasePosition(int joint);
};

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

  // this is how motors are numbered
  for(int i = 0; i < NUM_MOTORS; i++)
    node_ids_.push_back(2*i + 1);
  // this is how the sensors are mounted
  sensor_directions_.assign(NUM_MOTORS, 0);
  sensor_directions_[RIGHT_PALM] =    -1;
  sensor_directions_[LEFT_PALM] =      1;
  sensor_directions_[RIGHT_FINGER] =  -1;
  sensor_directions_[MIDDLE_FINGER] = -1;
  sensor_directions_[LEFT_FINGER] =   -1;
  // this is how the motors are mounted
  motor_directions_.assign(NUM_MOTORS, 0);
  motor_directions_[RIGHT_PALM] =    -1;
  motor_directions_[LEFT_PALM] =      1;
  motor_directions_[RIGHT_FINGER] =  -1;
  motor_directions_[MIDDLE_FINGER] = -1;
  motor_directions_[LEFT_FINGER] =    1;
  // velocities for moving in velocity mode
  joint_velocities_.assign(NUM_MOTORS, 0);
  joint_velocities_[RIGHT_PALM] = PALM_VELOCITY;
  joint_velocities_[LEFT_PALM] = PALM_VELOCITY;
  joint_velocities_[RIGHT_FINGER] = FINGER_VELOCITY;
  joint_velocities_[MIDDLE_FINGER] = FINGER_VELOCITY;
  joint_velocities_[LEFT_FINGER] = FINGER_VELOCITY;
  // set up joint state to publish
  joint_state_.header.frame_id = "hand_base_link";
  joint_state_.position.assign(NUM_MOTORS, 0.);
  joint_state_.velocity.assign(NUM_MOTORS, 0.);
  joint_state_.effort.assign(NUM_MOTORS, 0.);
  joint_state_.name.resize(NUM_MOTORS);
  joint_state_.name[RIGHT_PALM] =    "hand_right_crank_base_joint";
  joint_state_.name[LEFT_PALM] =     "hand_left_crank_base_joint";
  joint_state_.name[RIGHT_FINGER] =  "hand_right_finger_lower_joint";
  joint_state_.name[MIDDLE_FINGER] = "hand_middle_finger_lower_joint";
  joint_state_.name[LEFT_FINGER] =   "hand_left_finger_lower_joint";
  // use a small median filter for the noisy joint values
  for(int i = 0; i < NUM_MOTORS; i++)
    joint_value_filters_.push_back(MedianFilter(5));
  for(int i = 0; i < NUM_MOTORS; i++)
    sensor_calibrations_.push_back(SensorCalibration());

  double zero = 0;
  // NOTE: I don't know if the association of sensor numbers is correct
  nh_.getParam("joint_sensor_calibration/palm_sensor1", zero);
  sensor_calibrations_[LEFT_PALM] = zero;
  nh_.getParam("joint_sensor_calibration/palm_sensor2", zero);
  sensor_calibrations_[RIGHT_PALM] = zero;
  nh_.getParam("joint_sensor_calibration/finger_sensor3", zero);
  // NOTE: Yes, now left and right seem swapped, but this is actually right. I think.
  sensor_calibrations_[LEFT_FINGER] = zero;
  nh_.getParam("joint_sensor_calibration/finger_sensor4", zero);
  sensor_calibrations_[MIDDLE_FINGER] = zero;
  nh_.getParam("joint_sensor_calibration/finger_sensor5", zero);
  sensor_calibrations_[RIGHT_FINGER] = zero;

  openDevice();
  //startMotors();

  // move to some valid starting position
  // TODO: this requires that the published joint positions are already valid,
  // which right now is not the case. It takes a few seconds.
  //openFingers(MAX_SUSTAINED_CURRENT);

  joint_value_sub_= new ros::Subscriber(nh_.subscribe("joint_value_arduino", 1, &KCLHandController::jointValueCB, this));
  joint_state_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::JointState>("active_joint_states", 1));

  as_.start();
}

KCLHandController::~KCLHandController()
{
  delete joint_value_sub_;
  delete joint_state_pub_;
  delete move_finger_srv_;
  stopMotors();
  closeDevice();
}

/**
 * TODO: fill in the action feedback with sensible information
 * TODO: have open/closeHand return a success value and succedd/abort action accordingly
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
    if(goal->finger >= 0 && goal->finger < NUM_MOTORS)
    {
      bool reached = false;
      succeeded = moveFinger(goal->finger, 1., goal->position, reached);
    }
  }
  else
    ROS_ERROR("received invalid hand command");

  if(succeeded)
    as_.setSucceeded(result);
  else
    as_.setAborted(result);
}

void KCLHandController::jointValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  static unsigned int seq = 0;

  if((int)msg->data.size() != NUM_MOTORS)
    throw runtime_error("Maxon Motor EPOS: wrong number of joint position values received");

  // Whenever we get a position update, we also want a current/effort update.
  // NOTE: We do this outside the joint value mutex lock/unlock, because all motor
  // access functions (like getCurrent) use the motor mutex and we don't want to
  // occupy two mutexes at the same time to avoid deadlocks!
  std::vector<double> currents(NUM_MOTORS, 0.);
  for(size_t i = 0; i < currents.size(); i++)
    currents[i] = getCurrent(i);

  joint_value_mutex_.lock();
  for(size_t i = 0; i < msg->data.size(); i++)
  {
    joint_value_filters_[i].add(sensor_directions_[i]*sensor_calibrations_[i].rawValueToRadians(msg->data[i]));
    joint_state_.position[i] = joint_value_filters_[i].get();
    joint_state_.effort[i] = currents[i];
  }
  joint_state_.header.seq = seq++;
  joint_state_.header.stamp = ros::Time::now();
  joint_state_pub_->publish(joint_state_);
  joint_value_mutex_.unlock();
}

void KCLHandController::openDevice()
{
  motor_mutex_.lock();

  std::stringstream msg;

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
          if(baudrate_ != check_baudrate)
          {
            throw runtime_error("Maxon Motor EPOS: failed to set baudrate");
          }
        }
      }
    }
  }
  else
  {
    throw runtime_error("Maxon Motor EPOS: failed to open device");
  }

  motor_mutex_.unlock();
}

void KCLHandController::closeDevice()
{
  motor_mutex_.lock();
  unsigned int error_code;
  ROS_INFO("Closing Maxon Motor EPOS device");
  if(VCS_CloseDevice(key_handle_, &error_code) == 0 || error_code != 0)
    throw runtime_error("Maxon Motor EPOS: failed to close device");
  motor_mutex_.unlock();
}

void KCLHandController::getErrors()
{
  // number of actual errors
  unsigned char nbOfDeviceError = 0;
  // error code from function
  unsigned int error_code;
  //error code from device
  unsigned int deviceErrorCode = 0;

  for(int i = 0; i < node_ids_.size(); i++)
  {
    // get number of device errors
    if(VCS_GetNbOfDeviceError(key_handle_, node_ids_[i], &nbOfDeviceError, &error_code))
    {
      //  read device error code
      for(unsigned char errorNumber = 1; errorNumber <= nbOfDeviceError; errorNumber++)
      {
        if(VCS_GetDeviceErrorCode(key_handle_, node_ids_[i], errorNumber, &deviceErrorCode, &error_code))
        {
          ROS_INFO("Maxon Motor EPOS: joint %13s (node id %d) error # %d",
                   joint_state_.name[i].c_str(), (int)node_ids_[i], (int)deviceErrorCode); 
        }
        else
          ROS_INFO("FUCK C");
      }
    }
    else
      ROS_INFO("FUCK B");
  }
}

void KCLHandController::startMotors()
{
  motor_mutex_.lock();

  getErrors();

  for(int i = 0; i < node_ids_.size(); i++)
  {
    int is_fault = 0;
    int is_enabled = 0;
    unsigned int error_code;

    if(VCS_GetFaultState(key_handle_, node_ids_[i], &is_fault, &error_code) == 0)
      throw runtime_error("Maxon Motor EPOS: failed to get node fault state");
    if(is_fault)
    {
      std::stringstream msg;
      msg << "clear fault, node = '" << node_ids_[i] << "'";
      ROS_INFO("%s", msg.str().c_str());

      if(VCS_ClearFault(key_handle_, node_ids_[i], &error_code) == 0)
        throw runtime_error("Maxon Motor EPOS: failed to clear node fault state");
    }

    if(VCS_GetEnableState(key_handle_, node_ids_[i], &is_enabled, &error_code) == 0)
      throw runtime_error("Maxon Motor EPOS: failed to get node enable state");
    if(!is_enabled)
    {
      if(VCS_SetEnableState(key_handle_, node_ids_[i], &error_code) == 0)
        throw runtime_error("Maxon Motor EPOS: failed to enable node");
    }
  }

  motor_mutex_.unlock();
}

void KCLHandController::stopMotors()
{
  motor_mutex_.lock();
  for(int i = 0; i < node_ids_.size(); i++)
  {
    unsigned int error_code;
    if(VCS_SetDisableState(key_handle_, node_ids_[i], &error_code) == 0)
      throw runtime_error("Maxon Motor EPOS: failed to disable node");
  }
  motor_mutex_.unlock();
}

void KCLHandController::activateVelocityMode(int joint_idx)
{
  motor_mutex_.lock();
  unsigned int error_code = 0;
  if(VCS_ActivateProfileVelocityMode(key_handle_, node_ids_[joint_idx], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set profile velocity mode");
  motor_mutex_.unlock();
}

void KCLHandController::moveWithVelocity(int joint_idx, double velocity)
{
  motor_mutex_.lock();
  unsigned int error_code = 0;
  if(VCS_MoveWithVelocity(key_handle_, node_ids_[joint_idx], (long)velocity*motor_directions_[joint_idx], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to move with velocity");
  motor_mutex_.unlock();
}

void KCLHandController::haltVelocityMovement(int joint_idx)
{
  motor_mutex_.lock();
  unsigned int error_code = 0;
  if(VCS_HaltVelocityMovement(key_handle_, node_ids_[joint_idx], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to halt velocity movement");
  motor_mutex_.unlock();
}

void KCLHandController::activateCurrentMode(int joint_idx)
{
  motor_mutex_.lock();
  unsigned int error_code = 0;
  if(VCS_ActivateCurrentMode(key_handle_, node_ids_[joint_idx], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to activate current mode");
  motor_mutex_.unlock();
}

void KCLHandController::setCurrent(int joint_idx, double current)
{
  motor_mutex_.lock();
  unsigned int error_code = 0;
  if(VCS_SetCurrentMust(key_handle_, node_ids_[joint_idx], (short)current*motor_directions_[joint_idx], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set current");
  motor_mutex_.unlock();
}

double KCLHandController::getCurrent(int joint_idx)
{
  motor_mutex_.lock();
  unsigned int error_code = 0;
  // NOTE: I don't know over which time period the current is averaged. But at least the very
  // sharp peaks should be gone. In any case we should keep our own average too.
  short current = 0.;
  if(VCS_GetCurrentIsAveraged(key_handle_, node_ids_[joint_idx], &current, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to get current");
  motor_mutex_.unlock();
  return (double)current;
}

bool KCLHandController::openFingers(float rel_current_limit)
{
  std::vector<double> target_positions(NUM_MOTORS);
  bool all_target_positions_reached = false;
  target_positions[RIGHT_PALM] = deg_to_rad(15.);
  target_positions[LEFT_PALM] = deg_to_rad(15.);
  target_positions[RIGHT_FINGER] = deg_to_rad(-60.);  // HACK: was -50
  target_positions[MIDDLE_FINGER] = deg_to_rad(-60.);  // HACK: was -50
  target_positions[LEFT_FINGER] = deg_to_rad(-60.);  // HACK: was -50
  bool succeeded = moveFingers(rel_current_limit, target_positions, all_target_positions_reached);
  pre_grasp = true;  // HACK
  return succeeded;
}

bool KCLHandController::closeFingers(float rel_current_limit)
{
  std::vector<double> target_positions(NUM_MOTORS);
  bool all_target_positions_reached = false;
  target_positions[RIGHT_PALM] = deg_to_rad(15.);
  target_positions[LEFT_PALM] = deg_to_rad(15.);
  // HACK
  if(pre_grasp)
  {
    target_positions[RIGHT_FINGER] = deg_to_rad(-50.);
    target_positions[MIDDLE_FINGER] = deg_to_rad(-50.);
    target_positions[LEFT_FINGER] = deg_to_rad(-50.);
    pre_grasp = false;
  }
  else
  {
    target_positions[RIGHT_FINGER] = deg_to_rad(-15.);
    target_positions[MIDDLE_FINGER] = deg_to_rad(10.);
    target_positions[LEFT_FINGER] = deg_to_rad(-15.);
  }
  bool succeeded = moveFingers(rel_current_limit, target_positions, all_target_positions_reached);
  // if targets not reached, then some joints ran into their current limit, i.e. we are probably
  // holding an object.
  /* HACK: leave out for now
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

  static bool first_call = true;  // HACK: only move the palm ONCE with the first move

  startMotors();  // HACK: Do I really need to clear failure states all the time?

  // absolute limit for sustained current in mA
  short current_limit = (short)((float)MAX_SUSTAINED_CURRENT*clamp(rel_current_limit, (float)0., (float)1.));
  // thread safe copy of joint state
  sensor_msgs::JointState state;
  // store which motor is still running
  std::vector<bool> motor_running(NUM_MOTORS, false);
  // .. and how many are still running
  int num_motors_running = 0;
  // count how many joints reached the target (vs. reaching their current limits)
  int num_targets_reached = 0;
  /// motor currents averaged over several cycles, to filter out short peaks
  std::vector<RunningAverage> avg_currents;

  for(int i = 0; i < node_ids_.size(); i++)
    avg_currents.push_back(RunningAverage((CURRENT_AVERAGING_TIME_MS*CONTROL_LOOP_FREQ_HZ)/1000));

  // save current joint state locally
  joint_value_mutex_.lock();
  state = joint_state_;
  joint_value_mutex_.unlock();

  // first set all motors in motion
  for(int i = 0; i < node_ids_.size(); i++)
  {
    // HACK
    if(!first_call && (i == LEFT_PALM || i == RIGHT_PALM))
      continue;

    if(!isAtPosition(state.position[i], target_positions[i], 0))
    {
      activateVelocityMode(i);

      // set motion direction depending on where target is relative to current position
      int direction = 0;
      if(state.position[i] < target_positions[i])
        direction = 1;
      else if(state.position[i] > target_positions[i])
        direction = -1;
      // else: leave 0

      state.velocity[i] = direction*joint_velocities_[i];
      moveWithVelocity(i, state.velocity[i]);

      motor_running[i] = true;
      num_motors_running++;
    }
    else
    {
      num_targets_reached++;
    }

    ROS_INFO("joint %13s:  pos [rad]: %5.3lf  target [rad]: %5.3lf  vel [rpm]: %5.0lf",
             state.name[i].c_str(), state.position[i], target_positions[i], state.velocity[i]);
  }

  // save local joint state to current one with possibly changed velocities
  joint_value_mutex_.lock();
  joint_state_ = state;
  joint_value_mutex_.unlock();

  // Then observe currents and positions and stop when target positions are
  // reached, current limits are exceeded or timeout occurs.
  ros::Rate rate(CONTROL_LOOP_FREQ_HZ);
  int timeout_cnt = CONTROL_LOOP_TIMEOUT_S*CONTROL_LOOP_FREQ_HZ;
  while(num_motors_running > 0 && timeout_cnt > 0)
  {
    // save current joint state locally
    joint_value_mutex_.lock();
    state = joint_state_;
    joint_value_mutex_.unlock();

    for(int i = 0; i < node_ids_.size(); i++)
    {
      // HACK
      if(!first_call && (i == LEFT_PALM || i == RIGHT_PALM))
        continue;

      if(motor_running[i])
      {
        double current = state.effort[i];
        avg_currents[i].add(fabs(current));

        // if target reached, current limit exceeded or timeout occured then stop this finger
        bool target_position_reached = isAtPosition(state.position[i], target_positions[i], state.velocity[i] > 0 ? 1 : -1);
        bool peak_current_exceeded = fabs(current) >= MAX_PEAK_CURRENT;
        bool avg_current_exceeded = avg_currents[i].get() >= current_limit;

        if(peak_current_exceeded || avg_current_exceeded || target_position_reached || timeout_cnt <= 0)
        {
          if(target_position_reached)
            ROS_INFO("joint %13s (node id %d) reached target position pos [rad]: %5.3lf",
                      state.name[i].c_str(), (int)node_ids_[i], state.position[i]);
          else if(peak_current_exceeded)
            ROS_INFO("joint %13s (node id %d) reached peak current limit: %5.0lf mA, pos [rad]: %5.3lf",
                      state.name[i].c_str(), (int)node_ids_[i], current, state.position[i]);
          else if(avg_current_exceeded)
            ROS_INFO("joint %13s (node id %d) reached sustained current limit: %5.0lf mA, pos [rad]: %5.3lf",
                      state.name[i].c_str(), (int)node_ids_[i], avg_currents[i].get(), state.position[i]);
          else
            ROS_INFO("joint %13s (node id %d) reached timeout", state.name[i].c_str(), (int)node_ids_[i]);

          if(target_position_reached)
            num_targets_reached++;

          motor_running[i] = false;
          state.velocity[i] = 0.;
          num_motors_running--;
          haltVelocityMovement(i);
        }
      }
    }

    // save local joint state to current one with possibly changed velocities
    joint_value_mutex_.lock();
    joint_state_ = state;
    joint_value_mutex_.unlock();

    timeout_cnt--;

    rate.sleep();
  }

  all_target_positions_reached = (num_targets_reached == (int)target_positions.size());

  ROS_INFO("done moving fingers");

  first_call = false;

  return timeout_cnt > 0;
}

bool KCLHandController::moveFinger(int joint_idx, float rel_current_limit, double target_position,
                                   bool &target_position_reached)
{
  ROS_INFO("start moving single finger");

  startMotors();  // HACK: Do I really need to clear failure states all the time?

  // absolute limit for sustained current in mA
  short current_limit = (short)((float)MAX_SUSTAINED_CURRENT*clamp(rel_current_limit, (float)0., (float)1.));
  // thread safe copy of joint state
  sensor_msgs::JointState state;
  bool motor_running = false;
  /// motor current averaged over several cycles, to filter out short peaks
  RunningAverage avg_current((CURRENT_AVERAGING_TIME_MS*CONTROL_LOOP_FREQ_HZ)/1000);

  // save current joint state locally
  joint_value_mutex_.lock();
  state = joint_state_;
  joint_value_mutex_.unlock();

  // first set motor in motion
  if(!isAtPosition(state.position[joint_idx], target_position, 0))
  {
    activateVelocityMode(joint_idx);

    // set motion direction depending on where target is relative to current position
    int direction = 0;
    if(state.position[joint_idx] < target_position)
      direction = 1;
    else if(state.position[joint_idx] > target_position)
      direction = -1;
    // else: leave 0

    state.velocity[joint_idx] = direction*joint_velocities_[joint_idx];
    moveWithVelocity(joint_idx, state.velocity[joint_idx]);

    motor_running = true;
    target_position_reached = false;
  }
  else
  {
    target_position_reached = true;
  }

  // save local joint state to current one with possibly changed velocities
  joint_value_mutex_.lock();
  joint_state_ = state;
  joint_value_mutex_.unlock();

  ROS_INFO("joint %13s:  pos [rad]: %5.3lf  target [rad]: %5.3lf  vel [rpm]: %5.0lf",
           state.name[joint_idx].c_str(), state.position[joint_idx],
           target_position, state.velocity[joint_idx]);

  // Then observe currents and positions and stop when target positions are
  // reached, current limits are exceeded or timeout occurs.
  ros::Rate rate(CONTROL_LOOP_FREQ_HZ);
  int timeout_cnt = CONTROL_LOOP_TIMEOUT_S*CONTROL_LOOP_FREQ_HZ;
  while(motor_running > 0 && timeout_cnt > 0)
  {
    // save current joint state locally
    joint_value_mutex_.lock();
    state = joint_state_;
    joint_value_mutex_.unlock();

    double current = state.effort[joint_idx];
    avg_current.add(fabs(current));

    // if target reached, current limit exceeded or timeout occured then stop this finger
    target_position_reached = isAtPosition(state.position[joint_idx], target_position, state.velocity[joint_idx] > 0 ? 1 : -1);
    bool peak_current_exceeded = fabs(current) >= MAX_PEAK_CURRENT;
    bool avg_current_exceeded = avg_current.get() >= current_limit;
    timeout_cnt--;

    if(peak_current_exceeded || avg_current_exceeded || target_position_reached || timeout_cnt <= 0)
    {
      if(target_position_reached)
        ROS_INFO("joint %13s (node id %d) reached target [rad]: %5.3lf, pos [rad]: %5.3lf",
                  state.name[joint_idx].c_str(), (int)node_ids_[joint_idx], target_position, state.position[joint_idx]);
      else if(peak_current_exceeded)
        ROS_INFO("joint %13s (node id %d) reached peak current limit: %5.0lf mA, pos [rad]: %5.3lf",
                  state.name[joint_idx].c_str(), (int)node_ids_[joint_idx], current, state.position[joint_idx]);
      else if(avg_current_exceeded)
        ROS_INFO("joint %13s (node id %d) reached sustained current limit: %5.0lf mA, pos [rad]: %5.3lf",
                  state.name[joint_idx].c_str(), (int)node_ids_[joint_idx], avg_current.get(), state.position[joint_idx]);
      else
        ROS_INFO("joint %13s (node id %d) reached timeout", state.name[joint_idx].c_str(), (int)node_ids_[joint_idx]);

      motor_running = false;
      haltVelocityMovement(joint_idx);
      state.velocity[joint_idx] = 0.;
    }

    // save local joint state to current one with possibly changed velocities
    joint_value_mutex_.lock();
    joint_state_ = state;
    joint_value_mutex_.unlock();

    rate.sleep();
  }

  ROS_INFO("done moving single finger");

  return timeout_cnt > 0;
}

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

bool KCLHandController::isAtPosition(double pos, double target, int direction)
{
  bool ret = false;
  if(direction < 0)
    ret = pos < target;
  else if(direction > 0)
    ret = pos > target;
  else
    ret = fabs(target - pos) <= POSITION_REACHED_THRESHOLD;
  return ret;
}

/**
 * TODO: Don't know if this works.
 */
void KCLHandController::holdPosition(int joint)
{
  motor_mutex_.lock();

  int current_motor_encoder_pos = 0;
  unsigned int error_code = 0;

  ROS_INFO("holding posotion for motor %d", joint);

  if(VCS_ActivatePositionMode(key_handle_, node_ids_[joint], &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set position mode");
  if(VCS_GetPositionIs(key_handle_, node_ids_[joint], &current_motor_encoder_pos, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to get position");
  if(VCS_SetPositionMust(key_handle_, node_ids_[joint], (long)current_motor_encoder_pos, &error_code) == 0)
    throw runtime_error("Maxon Motor EPOS: failed to set position");

  motor_mutex_.unlock();
}

/**
 * TODO: No idea how to do this,
 */
void KCLHandController::releasePosition(int joint)
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kclhand_controller");

  KCLHandController controller("actuate_hand");
  ros::spin();

  return 0;
}
