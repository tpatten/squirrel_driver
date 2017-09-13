/**
 *
 * KCLHand controller
 * auther: Jie Sun
 * email: jie.sun@kcl.ac.uk
 * 
 * Provide following functions for the kclhand control
 * 
 * 1. Joint postition control
 * 2. Hand trajectory control 
 * 3. ... 
 * 4. 
 * 5.
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

#define TH

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


class JointSensor
{
private:
	std::string joint_name_;
	double joint_sensor_raw_value_;	

  //get from ros parameter server
  double joint_value_max_;
  double joint_sensor_zero_value_;
  int sensor_direction_;
  double joint_value_min_;


  const double sensor_k = 9./28.;
  double sensor_b;

  bool joint_value_valid_;
  double joint_calibrated_value_degrees_;


public:
	JointSensor(ros::NodeHandle &nh, unsigned int joint_id);
  
  double getJointRawValue()
	{
		return joint_sensor_raw_value_;
	}

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

  void setJointValueZero(double zero)
  {
    sensor_b = -sensor_k * zero;
  }

  void sensorCalibratedValueUpdate(double raw)
  {

    joint_calibrated_value_degrees_ = (sensor_k*raw + sensor_b) * sensor_direction_; 
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



class JointMotor
{
private:
  
  static const double POSITION_THRESHOLD = 0.1;

  void *epos_handle_;
  
  // get from ros parameters
  int motor_node_id_;
	int motor_direction_;

  // motor current state
	double current_joint_position_; 
	double motor_velocity_;
	double motor_current_;
  
  //motor settings 
  double motor_move_current_;
	double motor_holding_current_;
  double motor_moving_velocity_;
	
  //motor move target
  double target_position_;
	
  double max_peak_current_;
	bool motor_running_;

  bool is_target_reached;


	void getErrors();




  // reset motor
  void reset();


  void stopMotor();
  
  // activate motor profile velocity mode
  void activateVelocityMode();

  //velocity control of the motor, move with a given velocity
  void moveWithVelocity(double velocity);
 	
  //halt velocity movement
  void haltVelocityMovement();

  // activate motor current mode
  void activateCurrentMode();

  // set motor current, motor will move with a given current
  void setCurrent(double current);

  // get motor current
  double getCurrent();



  // judge if motor has reached the position
  bool isTargetReached()
  {
    if(fabs(current_joint_position_ - target_position_) <= POSITION_THRESHOLD)
      is_target_reached = true;
    else is_target_reached = false;
    return is_target_reached;
  }



public:

  JointMotor(ros::NodeHandle &nh, void *epos_handle_, unsigned int joint_id);
	
  // enable motors 
  bool enableMotor();
    // disable motors 
  bool disableMotor();

  // motor move to target position
  bool moveToTarget(double current_position, double target);


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
  ros::ServiceServer metahand_init_srv_server_;
  ros::ServiceServer metahand_mode_srv_server_;
  
  
  // ros service, topic done 

  // hand related 
	JointSensor *joints_sensor_;
  JointMotor *joints_motor_;
  // hand related done 

  bool hand_is_initialized_;
  bool has_new_goal_;

  // hand hardware setting
  static const unsigned int NUM_JOINTS = 5;
  // hand hardware setting done 



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





};




#endif