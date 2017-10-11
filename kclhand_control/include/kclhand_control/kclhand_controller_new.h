/**
 *
 * KCLHand controller
 * auther: Jie Sun
 * email: jie.sun@kcl.ac.uk
 * 
 * Provide following functions for the kclhand control
 * 
 * 1. Joint postition control for finger
 * 2. hand configuration control 
 * 3. grasping
 * 
 * September 2017
*/


#ifndef KCLHAND_CONTROLLER
#define KCLHAND_CONTROLLER

#include <vector>
#include <boost/thread/mutex.hpp>
//#include <actionlib/server/simple_action_server.h>

/**
 * PI controller for hand impedance control 
 *
*/

#include <kclhand_control/MoveFinger.h>
#include <kclhand_control/MoveHand.h>
#include <kclhand_control/HandOperationMode.h>

inline double deg_to_rad(double deg)
{
  return deg*M_PI/180.;
}

class PIController
{
private:
    double SetValue ;        //target 
    double An;              //=kp+ki  for e(n)
    double Bn_1;           // -kp     for e(n-1)
    double Limit;          //limit
    double Errorn_1;       //e(n-1)  e
    double DeltaUn;        // delta_u(n);  
    double Un_output;     //output

public:

  void SetPara(double kp, double ki, double target, double limit)
  {
    An = kp + ki;
    Bn_1 = -kp;
    Limit = limit;
    SetValue = target;
    DeltaUn = 0;
    Errorn_1 = 0;
  }


  double PIControllerCalu(double NewInput)
  {
    

    double Errorn =0;  

    if(NewInput < -20.)
    {
      NewInput = -20.;
    } 

    Errorn = SetValue - NewInput;  // e(n)
    DeltaUn = An * Errorn + Bn_1 * Errorn_1; // delta_u(n)  
    Errorn_1 = Errorn; 
    Un_output = Un_output + DeltaUn;
    
    return DeltaUn;
  }
 
};


class ImpedancePIController
{
private:
    double SetValue ;        //target 
    double An;              //=kp+ki  for e(n)
    double Bn_1;           // -kp     for e(n-1)
    double Limit;          //limit
    double Errorn_1;       //e(n-1)  
    double DeltaUn;        // delta_u(n);  
    double Un_output;     //output

public:

  void SetPara(double kp, double ki, double target, double limit)
  {
    An = kp + ki;
    Bn_1 = -kp;
    Limit = limit;
    SetValue = target;
    DeltaUn = 0;
    Errorn_1 = 0;
  }


  double PIControllerCalu(double NewInput)
  {
    

    double Errorn =0;  

   // if(NewInput < -20.)
   // {
   //   NewInput = -20.;
   // } 
    Errorn = SetValue - NewInput;  // e(n)
    DeltaUn = An * Errorn + Bn_1 * Errorn_1; // delta_u(n)  
    Errorn_1 = Errorn; 
    Un_output = Un_output + DeltaUn;
    
    return DeltaUn;
  }
 
};

/*
 * The JointSensor Class 
 *
 *
 *
 */

class JointSensor
{

private:
	
  ros::NodeHandle nh_;
  std::string joint_name_;
	double joint_sensor_raw_value_;	
  unsigned int joint_id_;

  //get from ros parameter server
  double joint_value_max_;
  double joint_sensor_zero_value_;
  int sensor_direction_;
  double joint_value_min_;

  // map the sensor value from arduino to the hand request
  static const double sensor_k = 9./28.;
  double sensor_b;

  bool joint_value_valid_;
  double joint_calibrated_value_degrees_;

  std::vector<double> temp_joint_raw_value_;
  double sum_temp_joint_sensor_value_;
  static const int TEMP_JOINT_VALUE_COUNT_ = 10;

public:
	JointSensor(const ros::NodeHandle &nh, const unsigned int &joint_id);
  
  // get joint sensor raw value from arduino
  double getJointRawValue()
	{
		return joint_sensor_raw_value_;
	}

  //should make a buffer for 10 values, so it will be more stable
	void checkJointValueRange() 
	{
		if ((joint_calibrated_value_degrees_ >= joint_value_min_) && (joint_calibrated_value_degrees_ <= joint_value_max_))
			joint_value_valid_ = true;
		else joint_value_valid_ = false; // should stop the hand; or go to init fucntion;
	} 


	void jointValueFilter()
	{
		/*
		 This is the filter for sensors

		*/
	}

  bool getIfJointValueValid()
  {
    return joint_value_valid_;
  }

  void setJointValueZero(double zero)
  {
    sensor_b = -sensor_k * zero;
  }

  //get the mean value from the joint sensor raw data
  void sensorCalibratedValueUpdate(double raw)
  {
    if (temp_joint_raw_value_.empty())
    {
      temp_joint_raw_value_.resize(TEMP_JOINT_VALUE_COUNT_, raw);
      sum_temp_joint_sensor_value_ = TEMP_JOINT_VALUE_COUNT_ * raw;
    }

    temp_joint_raw_value_.push_back(raw);
    sum_temp_joint_sensor_value_ += raw;   
    sum_temp_joint_sensor_value_ -= *temp_joint_raw_value_.begin();
    temp_joint_raw_value_.erase(temp_joint_raw_value_.begin());

    double mean_joint_value =  sum_temp_joint_sensor_value_ / TEMP_JOINT_VALUE_COUNT_;
    joint_calibrated_value_degrees_ = (sensor_k*mean_joint_value + sensor_b) * sensor_direction_; 
    checkJointValueRange();
  }

  double getSensorCalibratedValueDeg()
  {
    return joint_calibrated_value_degrees_;
  }

  double getSensorCalibratedValueRad()
  {
    return deg_to_rad(joint_calibrated_value_degrees_);
  }

};

/*
 * The JointMotor class for the epos2 controller (from maxon)
 * 
 *
 *
 */

class JointMotor
{
private:
  
  static const double POSITION_THRESHOLD = 0.07;
  static const short unsigned int NOMINAL_CURRENT_PALM = 300;
  static const short unsigned int NOMINAL_CURRENT_FINGER = 250; //250
  static const short unsigned int MAX_OUTPUT_CURRENT = 500;
  static const short unsigned int THERMAL_TIME_CONSTANT = 95;

  ros::NodeHandle nh_;
  void *epos_handle_;
  
  // get from ros parameters
  int motor_node_id_;

  // motor current state
	double current_joint_position_; 
	double motor_velocity_;
	double motor_current_;
  
  //motor settings 
  double motor_move_current_;
	double motor_holding_current_;
  double motor_moving_velocity_;
  int motor_direction_;
	
  //motor move target
  double target_position_;
	
  double max_peak_current_;
	bool motor_running_;
  bool is_target_reached;


  //motor settings (maxon epos control board)
  short unsigned int nominal_current_;
  short unsigned int max_output_current_;
  short unsigned int thermal_time_constant_;

	void getErrors();


  // reset motor
  void reset();


  void stopMotor();
  
  // activate motor profile velocity mode
  void activateVelocityMode();

  //velocity control of the motor, move with a given velocity
  void moveWithVelocity(double velocity);
 	
  // judge if motor has reached the position
  bool isTargetReached()
  {
    if(fabs(current_joint_position_ - target_position_) <= POSITION_THRESHOLD)
      is_target_reached = true;
    else is_target_reached = false;
    return is_target_reached;
  }



public:

  JointMotor(const ros::NodeHandle &nh, void *epos_handle, const unsigned int &joint_id);
	
  // enable motors 
  bool enableMotor();
  
  // disable motors 
  bool disableMotor();

  // motor move to target position
  bool moveToTarget(double current_position, double target);

  // get motor current
  double getCurrent();

  //halt velocity movement
  void haltVelocityMovement();

  // activate motor current mode
  void activateCurrentMode();

  // set motor current, motor will move with a given current
  void setCurrent(double current);

  //Get Maximal Following Error
  unsigned int getMaxFollowingError();

  //Get Maximal Following Error
  void setMaxFollowingError();

  //Read motor parameters from the maxon board
  void getDCMotorParameter();

  //Set motor parameters from the maxon board
  void setDCMotorParameter();

  // get motor Nominal Current value, default value: 189, we shall set it to 250 to enlarge force
  short unsigned int getNominalCurrent()
  {
    return nominal_current_;
  }

  // get motor Max Output Current value, default value: 376, we shall set it to 500 to enlarge force
  short unsigned int getMaxOutputCurrent()
  {
    return max_output_current_;
  }

  // get motor Thermal Time Constant value, default value: 95
  short unsigned int getThermalTimeConstant()
  {
    return thermal_time_constant_;
  }

  bool getMotorEnableState();
};





class KCLHandController
{
private:
	// Maxon Epos2 controller settings.
  // should get from parameters 

  void* M_handle_;
  unsigned short M_node_id_;
  unsigned int M_error_code;
  
  std::string M_DEVICENAME_NAME_;
  std::string M_PROTOCAL_NAME_;
  std::string M_INTERFACE_;
  std::string M_PORTNAME_;
  int M_BAUDRATE_;
  // motor setting done 

  // ros service, topic  defination 
	ros::Subscriber joint_raw_value_sub_; //subscribe from the arduino board
	ros::Publisher joint_state_pub_;  // publish to palm solver in python
  ros::ServiceServer move_finger_srv_server_;
  ros::ServiceServer move_hand_srv_server_;
  //ros::ServiceServer metahand_init_srv_server_;
  ros::ServiceServer metahand_mode_srv_server_;
  
  // should add a server 
  
  // ros service, topic done 

  // hand related 
	JointSensor *joints_sensor_;
  JointMotor *joints_motor_;
  // hand related done 

  bool hand_is_initialized_;
  bool has_new_goal_;

  // hand hardware setting
  static const unsigned int NUM_JOINTS = 5;
  static const int TIMEOUT_COUNT = 80;
  static const int TIMEOUT_COUNT_FINGER = 300;


  std::vector<double> lower_to_upper_workspace_seq_;
  std::vector<double> upper_to_lower_workspace_seq_;
  std::vector<double> lower_workspace_open_conf_;
  std::vector<double> lower_workspace_close_conf_;
  std::vector<double> upper_workspace_open_conf_;
  std::vector<double> upper_workspace_close_conf_;

  int hand_grasping_current_defalut_;
  
public:
  ros::NodeHandle nh_;
	KCLHandController(std::string name);
  ~KCLHandController();

  // hand joint sensors call back
	void jointSensorValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg);
  
  // hand contorller initialization 
  bool init();
  
  // open device (epos2 controller)
  bool openDevice();
  
  // close device (epos2 controller)
  bool closeDevice();

  bool moveFingerSrvCB(kclhand_control::MoveFinger::Request &req, kclhand_control::MoveFinger::Response &res);
  bool handModeSrvCB(kclhand_control::HandOperationMode::Request &req, kclhand_control::HandOperationMode::Response &res);
  bool moveHandSrvCB(kclhand_control::MoveHand::Request &req, kclhand_control::MoveHand::Response &res);

  bool closingHandForGrasing();
  
  bool resetHandToInitPos();

  bool moveHandToTarget(const std::vector<double> &target);

  bool lowerToUpperWorkspace();
  bool upperToLowerWorkspace();
  
  bool openHand();
};


#endif


/*
1. joint sensor filter: a. for value mean, b from arduino side 
(done, but sensor has deadzone.... if go there, should call recover function)
2. disaster recovery --> init

3. grasping action, force control, velcity is changed 
4. to track a trajectory
5. get key handle function

8. real time joint check, if exceeded, then stop
9. add timeout function to all the service

10. add a service to set parameters -> done
11. singularity crossing

*/

/*
8. default paras 
nominal_current: 189, max_output_current_: 378, thermal_time_constant_: 95
*/