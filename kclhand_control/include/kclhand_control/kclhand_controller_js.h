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
	double joint_sensor_value_;	
	bool joint_value_valid_ ;
	static double Joint_value_max_;
	static double Joint_value_min_;
	static int sensor_direction_;

public:
	double getJointValue()
	{
		/**
			code for the judgement
		*/
		return joint_sensor_value_;
	}

	bool checkJointValueRange()
	{
		if ((joint_sensor_value_ >= Joint_value_min_) && (joint_sensor_value_ <= Joint_value_max_))
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

};

class JointMotor
{
private:
  	void *epos_handle_;
  	unsigned int node_id_; 
	static int motor_direction_;
	double motor_position_;
	double motor_velocity_;
	double motor_move_current_;
	double motor_holding_current_;
	double motor_target_position_;
	double max_peak_current_;
	bool motor_running_;

	void getErrors();
  	void startMotor();
    void stopMotor();
  	void activateVelocityMode();
    void moveWithVelocity(double velocity);
 	void haltVelocityMovement();
  	void activateCurrentMode();
    void setCurrent(double current);
    double getCurrent();

public:
	double getPosition()
	{
		return motor_position_;
	}

};


class KCLHandController
{
private:
	ros::NodeHandle nh_;
	ros::Subscriber joint_value_sub_;
	ros::Publisher joint_state_pub_;

	std::vector<JointSensor> joints_sensor_;
  std::vector<JointMotor> joints_motor_;

  bool handIsInitialized_;
  bool hasNewGoal_;

  static const unsigned int NUM_JOINTS = 5;




public:
	KCLHandController(std::string name);
  //~KCLHandController();

	void jointSensorValueCB(const std_msgs::Int16MultiArray::ConstPtr &msg);
  bool init();

};




#endif