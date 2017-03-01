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

#ifndef KCLHAND_CONTROLLER_H
#define KCLHAND_CONTROLLER_H

#include <vector>
#include <boost/thread/mutex.hpp>
#include <actionlib/server/simple_action_server.h>

inline double deg_to_rad(double deg)
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

/**
 * A controller for an individual joint (palm or finger)
 */
class JointController
{
private:
  /// time in ms over which we averge motor currents for limit check
  static const int CURRENT_AVERAGING_TIME_MS = 500;
  /// threshold (in rad) below which two joint positions are regarded equal
  static const double POSITION_REACHED_THRESHOLD = 0.035;

  /// Handle to the Maxon EPOS library
  void *epos_handle_;
  /// velocity in rpm
  /// Motor IDs are not necessarily from 0 to NUM_MOTORS. In our case they are
  /// 2*i + 1
  int node_id_;
  /// name, must match respective names in robot description
  std::string name_;
  /// calibration paramerers for joint angle sensors
  SensorCalibration calibration_;
  /// -1 or 1, depending on how the sensors are mounted
  int sensor_direction_;
  /// -1 or 1, depending on how the motors are mounted
  int motor_direction_;
  /// velocities to use in velocity mode, in rpm of the motor, not rad/s of the joint
  double max_velocity_;
  /// maximum allowed sustained current in mA
  double max_sustained_current_;
  /// the short time peak current can be higher
  double max_peak_current_;
  /// current joint position
  double position_;
  /// current motor current
  double current_;
  /// currently set target
  double target_;
  /// current motor velocity in rpm (not rad/s)
  double velocity_;
  /// whether the motor is currently running
  bool motor_running_;
  /// true if current target has been reached
  bool target_reached_;
  /// motor current averaged over several cycles, to filter out short peaks
  RunningAverage avg_current_;
  // joint values are extrremely noisy, so need to be filtered
  //MedianFilter joint_value_filter_;

  /**
   * Get motor errors.
   */
  void getErrors();
  /**
   * Start motors: clear failure flags and enable.
   */
  void startMotor();
  /**
   * Stop motors: disable nodes.
   */
  void stopMotor();
  /**
   * Activate velocity mode for the given joint.
   */
  void activateVelocityMode();
  /**
   * Move given joint with given velocity, in [RPM].
   */
  void moveWithVelocity(double velocity);
  /**
   * Stop moving given joint (in velocity mode).
   */
  void haltVelocityMovement();
  /**
   * Activate current mode for the given joint.
   */
  void activateCurrentMode();
  /**
   * Set current for given joint.
   */
  void setCurrent(double current);
  /**
   * Get current in [mA].
   */
  double getCurrent();
 
  /**
   * Check if a joint position has reached the target.
   * Direction takes into account the current moving direction (if any) of the
   * joint, to avoid that the target is overshot if the sample interval is too
   * coarse.
   * direction: 0 .. just compare positions
   *            -1 .. check if target reached from above (going down)
   *            1 .. check if target reached from below (gping up)
   */
  bool isAtPosition(double target, int direction)
  {
    bool ret = false;
    if(direction < 0)
      ret = position_ < target;
    else if(direction > 0)
      ret = position_ > target;
    else
      ret = fabs(target - position_) <= POSITION_REACHED_THRESHOLD;
    return ret;
  }

public:
  JointController(ros::NodeHandle &nh, void *epos_handle, int num);
  /**
   * Update position in [rad] from raw sensor value.
   */
  void updatePositionFromRawSensor(double p)
  {
    position_ = sensor_direction_*calibration_.rawValueToRadians(p);
  }
  /**
   * Update current in [mA].
   */
  void updateCurrent()
  {
    current_ = getCurrent();
    avg_current_.add(current_);
  }
  /**
   * Return whether target was reached.
   */
  bool targetReached()
  {
    return target_reached_;
  }
  /**
   * Return current position in [rad].
   */
  double position()
  {
    return position_;
  }
  /**
   * Velocity in [rad/s].
   */
  double velocity()
  {
    return velocity_;
  }
  /**
   * Return current current current in [mA].
   */
  double current()
  {
    return current_;
  }
  std::string name()
  {
    return name_;
  }
  bool isRunning()
  {
    return motor_running_;
  }
  /**
   * Start moving to target position.
   */
  bool startMoveToTarget(double target_position);
  /**
   * Keep moving, until target reached or current limit exceeded.
   */
  bool keepMovingToTarget();
  /**
   * And stop, once reached.
   */
  void stopMoveToTarget();
};

class KCLHandController
{
private:
  /// 3 fingers and 2 palms
  static const int NUM_JOINTS = 5;
  /// default (safe and nice) velocity in rpm when opening and closing fingers
  static const long FINGER_VELOCITY = 1000;
  /// the palm joints need slower motor rotations, as they are more direct
  static const long PALM_VELOCITY = 300;
  /// frequency of the control loop
  static const int CONTROL_LOOP_FREQ_HZ = 100;
  /// timeout in case something weird goes wrong with the motors
  static const int CONTROL_LOOP_TIMEOUT_S = 5;
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

  /// controllers for the individual joints
  std::vector<JointController> joints_;
  /// mutex to protect joint values (written to in subscriber callback, read
  /// from in control loop)
  boost::mutex joint_mutex_;

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
  //void keepTightGrip();
  /**
   * Loosen grip, i.e. set motor corrents to 0.
   */
  //void loosenGrip();
  //void holdPosition(int joint);
  //void releasePosition(int joint);
};

#endif
