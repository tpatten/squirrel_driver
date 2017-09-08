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

	bool checkJointValueRange()
	{
		if ((joint_calibrated_value_degrees_ >= joint_value_min_) && (joint_calibrated_value_degrees_ <= joint_value_max_))
			return joint_value_valid_;
		else 
		{
			joint_value_valid_ = false; // should stop the hand; or go to init fucntion;
			return joint_value_valid_;
		}
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

  double getSensorCalibratedValue(double raw)
  {
    joint_calibrated_value_degrees_ = (sensor_k*raw + sensor_b) * sensor_direction_;    
    return joint_calibrated_value_degrees_;
  }


  double test()
  {
    return sensor_b;
  }


};



class JointMotor
{
private:
  
  void *epos_handle_;
  
  // get from ros parameters
  int motor_node_id_;
	int motor_direction_;


	double motor_position_;
	double motor_velocity_;
	double motor_move_current_;
	double motor_holding_current_;
	double motor_target_position_;
	
  double max_peak_current_;
	bool motor_running_;


	void getErrors();

  // enable motors 
  bool enableMotor();

  // disable motors 
  bool disableMotor();

  // reset motor
  void reset();


  void stopMotor();
  
  // activate motor profile velocity mode
  void activateVelocityMode();

  //velocity control of the motor, move with a given velocity
  void moveWithVelocity(double velocity);
 	
  //halt velocity movement
  void haltVelocityMovement();


  void activateCurrentMode();

  void setCurrent(double current);

  double getCurrent();

  

public:

  JointMotor(ros::NodeHandle &nh, void *epos_handle_, unsigned int joint_id);
	double getPosition()
	{
		return motor_position_;
	}


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
	ros::Subscriber joint_value_sub_; //subscribe from the arduino board
	ros::Publisher joint_state_pub_;  // publish to palm solver in python
  // ros service, topic done 

  // hand related 
	JointSensor *joints_sensor_;
  JointMotor *joints_motor_;
  // hand related done 

  bool handIsInitialized_;
  bool hasNewGoal_;

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





};




#endif