#include "ros/ros.h"
#include <iostream>
#include "kclhand_control/Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int64.h"
#include <std_srvs/Empty.h>
#include "kclhand_control/graspPreparation.h"
#include "kclhand_control/graspCurrent.h"






typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

const string g_programName = "HandMoveToPosition ";

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif



void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   Demo(unsigned int* p_pErrorCode);
bool  motorRotationToConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send);
int mapHandConfiguration(string hand_configuration);
bool compareConfiguration(float currentOne[], float saveOne[]);
int CurrentContinuousDemo(unsigned int* p_pErrorCode, long current);
bool holdConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send);
bool handConfigurationCheck();
void disableMotor();
void haltPalmJoint(unsigned short motorTotalNumber);
bool handCurrentChecker();
bool holdMotorCurrent();
bool MotorCurrentIndividualChecker();
int PrepareVelocityDemo(unsigned int* p_pErrorCode);
int GetMotorVelocity(int motor_id);
bool palmMotorCurrentControl(int index, int current);



std_msgs::Float64MultiArray tendon_speed;

std_msgs::Float64MultiArray palm_state_data_send;

std_msgs::Float64MultiArray palm_force_control_data;
std_msgs::Float64MultiArray palm_control_signal_data;


sensor_msgs::JointState joint_target_position;
sensor_msgs::JointState real_time_sensor_value;
sensor_msgs::JointState test_send;
sensor_msgs::JointState target_position_temp;

float temp_position[5];
float temp_palm_force_control_data[2];
//float temp_palm_disp_velo_data_send[8];

/*
The motor_direction[5] is the only difference between all the hand produced by KCL
It defines the motor moving direction, just based on the set-up of the motors 
**for the hand in UIBK, motor_direction[5] = {1, -1, -1, 1, 1};
**for the hand in wien, motor_direction[5] = 

*/
//int motor_direction[5] = {1, -1, -1, 1, 1};
int motor_direction[5] = {1, -1, -1, -1, 1};





short int setCurrentValue = 160;
int setHoldingCurrent = 80;


float lower_sphere_parallel_grasp_angle[5] = {25,25,-20,-20,-20};
float lower_sphere_cylindrical_grasp_angle[5] = {60,60,5,-10,5};
float lower_sphere_central_grasp_angle[5] = {40,40,-5,-10,-5};
float upper_sphere_parallel_grasp_angle[5] = {-15,-15,-20,-20,-20};
float upper_sphere_cylindrical_grasp_angle[5] = {-60,-60,-60,-40,-60};

float upper_sphere_central_grasp_angle[5] = {-30,-35,-45,-45,-45};
float upper_sphere_central_grasp_angle_com[5] = {-30,-35,-70,-55,-70};


float temp_hand_configuration[5];
float realtime_control_signal[2];

float upper_sphere_central_grasp_palm[2] = {-40,-40};
float handConfigurationLowerLimit[5] ={-60,-60, -80, -80, -80};
float handConfigurationUpperLimit[5] ={10,10, 25, 15, 25};

float calibration_angle_traget1[5] = {-30,-35,-10,-5,-10};
float calibration_angle_traget2[5] = {-30,-35,-75,-60,-75};
float calibration_angle_traget3[5] = {-30,-35,-60,-45,-60};




float configurationData[5];
const int motor_amount = 5;
const unsigned int maximumCurrent = 150;
const int motorGraspingSpeed = 1000;

float switch_to_upper_sphere_angle[3][5] = {{60,60,-40,-40,-40},
							  			    {-25,67,-40,-40,-40},
								 		    {-40,-30,-40,-40,-40}};

float switch_to_lower_sphere_angle[3][5] = {{-30,-20,-40,-40,-40},
								  			{-40,70,-40,-40,-40},
								  			{60,70,-40,-40,-40}};




unsigned char flag = 0;

void PrintUsage()
{
	cout << "Usage: HelloEposCmd -h -n 1 -d deviceName -s protocolStackName -i interfaceName -p portName -b baudrate" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
}

void SeparatorLine()
{
	const int lineLength = 60;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	//USB
	
	g_deviceName = "EPOS2"; //EPOS
	g_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
	g_interfaceName = "USB"; //RS232
	g_portName = "USB0"; // /dev/ttyS1
	g_baudrate = 1000000; //115200
}

void ChooseMotor(unsigned short motor_number)
{
	g_usNodeId = motor_number;
}

int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	//LogInfo("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int ParseArguments(int argc, char** argv)
{
	int lOption;
	int lResult = MMC_SUCCESS;

	// Shut GetOpt error messages down (return '?'):
	opterr = 0;
	// Retrieve the options:
	while ( (lOption = getopt(argc, argv, ":hd:s:i:p:b:n:")) != -1 )
	{
		switch ( lOption ) {
			case 'h':
				PrintUsage();
				lResult = 1;
				break;
			case 'd':
				g_deviceName = optarg;
				break;
			case 's':
				g_protocolStackName = optarg;
				break;
			case 'i':
				g_interfaceName = optarg;
				break;
			case 'p':
				g_portName = optarg;
				break;
			case 'b':
				g_baudrate = atoi(optarg);
				break;
			case 'n':
				g_usNodeId = (unsigned short)atoi(optarg);
				break;
			case '?':  // unknown option...
				stringstream msg;
				msg << "Unknown option: '" << char(optopt) << "'!";
				LogInfo(msg.str());
				PrintUsage();
				lResult = MMC_FAILED;
				break;
		}
	}

	return lResult;
}

int DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long target_position)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	//LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	

	else
	{
			
			long targetPosition = target_position; 
			stringstream msg;
			msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
			LogInfo(msg.str());

			if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
			{
				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
				//break;
			}

			sleep(1);
		

		if(lResult == MMC_SUCCESS)
		{
			LogInfo("halt position movement");

			if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
			}
		}
	}

	return lResult;
}



bool HaltVelocity(HANDLE p_DeviceHandle, unsigned short p_usNodeId)
{
	int lResult = MMC_SUCCESS;
	unsigned int p_rlErrorCode=0;
	LogInfo("halt velocity movement");

			if(VCS_HaltVelocityMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, p_rlErrorCode);
			}
			return lResult;
}




int DemoCurrentMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long current_limit)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set current mode, node = " << p_usNodeId << "current is " << current_limit;
	LogInfo(msg.str());
	if(VCS_ActivateCurrentMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateCurrentMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	
	
	else
	{
			
			long upperCurrentLimit = current_limit;
			
			if(VCS_SetCurrentMust(p_DeviceHandle, p_usNodeId, upperCurrentLimit, &p_rlErrorCode) == 0)
				{
					LogError("setCurrentFailed", lResult, p_rlErrorCode);
					lResult = MMC_FAILED;
				}


	}

	return lResult;
}


int CurrentDemo(unsigned int* p_pErrorCode, long current)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

	lResult = DemoCurrentMode(g_pKeyHandle, g_usNodeId, lErrorCode, current);

	if(lResult != MMC_SUCCESS)
	{
		LogError("DemoCurrentMode", lResult, lErrorCode);
	}
	//else
	return lResult;
}


int CurrentContinuousDemo(unsigned int* p_pErrorCode, long current)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;
	short int currentValue = 0;

	for (int i =0; i<= current; i++)
	{
		lResult = DemoCurrentMode(g_pKeyHandle, g_usNodeId, lErrorCode, current);
		if (VCS_GetCurrentMust(g_pKeyHandle, g_usNodeId, &currentValue, p_pErrorCode) !=0)
		{
			if (currentValue <= current)
			{
				continue;
			}
			else 
			{
				break;
			}
		}	
	}
	
	if(lResult != MMC_SUCCESS)
	{
		LogError("DemoProfileVelocityMode", lResult, lErrorCode);
	}
	//else
	return lResult;
}


bool DemoProfileVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long motor_speed)
{
	short temp_current = 0;
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile velocity mode, node = " << p_usNodeId;

	LogInfo(msg.str());

	if(VCS_ActivateProfileVelocityMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfileVelocityMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{

		
			long targetvelocity = motor_speed;

			stringstream msg;
			msg << "move with target velocity = " << targetvelocity << " rpm, node = " << p_usNodeId;
			//LogInfo(msg.str());
			if (VCS_GetCurrentIs(p_DeviceHandle, p_usNodeId, &temp_current, &p_rlErrorCode) != 0)
			{

				if(VCS_MoveWithVelocity(p_DeviceHandle, p_usNodeId, targetvelocity, &p_rlErrorCode) == 0)
					{
						lResult = MMC_FAILED;
						LogError("VCS_MoveWithVelocity", lResult, p_rlErrorCode);
					}
			}	
	
	}

	return lResult;
}

int PrepareDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{

						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}


int PrepareVelocityDemo(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0;

	if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(lResult==0)
	{
		if(oIsFault)
		{
			stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult==0)
		{
			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}

			if(lResult==0)
			{
				if(!oIsEnabled)
				{
					if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
					{
						//haltPalmJoint(5);
						int current = holdMotorCurrent();
						
						LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
						lResult = MMC_FAILED;
					}
				}
			}
		}
	}
	return lResult;
}

int Demo(unsigned int* p_pErrorCode, long position)
{
	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

	lResult = DemoProfileVelocityMode(g_pKeyHandle, g_usNodeId, lErrorCode, position);

	if(lResult != MMC_SUCCESS)
	{
		LogError("DemoProfileVelocityMode", lResult, lErrorCode);
	}
	//else
	return lResult;
}


void set_target_position(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_target_position = *msg;
  //the 3rd motor is special, twice rate, inverse direction. But it's not anymore now
//    tendon_speed.data[2] = -2*tendon_speed.data[2];
//   printf("%f", tendon_speed.data[0]);
    for (int i = 0; i<5;i++)
  	{
  		temp_position[i] = (int)joint_target_position.position[i];

  	}
  
    flag = 1;

  //joint_target_position.position[0] = 90.0;
}


void sensor_value_call_back(const sensor_msgs::JointState::ConstPtr& msg)
{
    test_send.position.resize(5);
    real_time_sensor_value = *msg;
    test_send = real_time_sensor_value;
    flag = 1;
    /*
        # signals: [theta1, theta2, phi31, phi41]
        # dot_signals: [dot_theta1, dot_theta2]
        # desired_signals: [desired_theta1, desired_theta2]
    */
    
     //double k = double((360 * 3.1415)/(512*84*180)); 
     
     //double dot_palm_theta1 = GetMotorVelocity(1) * k;
	 //double dot_palm_theta2 = GetMotorVelocity(3) * k;
	 

	 //double displacement_theta1 = test_send.position[0] * 0.01745 + 3.1415;
	 //double displacement_theta2 = -test_send.position[1] * 0.01745;
	
     //float temp_palm_disp_velo_data_send[8] = {displacement_theta1, displacement_theta2, 1.5, 1.5, 0, 0, 3.14, -0.1};
     
    // palm_state_data_send.data.resize(8);

	//for (int i = 0; i<8; i++)
	//{		  	
		//palm_state_data_send.data[i] = temp_palm_disp_velo_data_send[i];
							
	//}
	 			
}



// to control the joints with the input
void palm_control_signal_call_back(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    bool kkk;
    palm_control_signal_data.data.resize(2);
    palm_control_signal_data = *msg;
    flag = 1;
    /*
        # signals: [theta1, theta2, phi31, phi41]
        # dot_signals: [dot_theta1, dot_theta2]
        # desired_signals: [desired_theta1, desired_theta2]
    */

    /*
	for (int i = 0; i<2; i++)
	{
		realtime_control_signal[i] = palm_control_signal_data.data[i];
	}

	ROS_INFO("Palm Fuzzy Control");
    unsigned short index = 0;
    int judge_traget_reach = 0;
    int motor_id;
    unsigned int ulErrorCode = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float graspForce =0;
    short int currentValue = 0;
    float motor_current[2];


	motor_current[0] = int(realtime_control_signal[0]*200);
	motor_current[1] = realtime_control_signal[1]*200;


    if (test_send.position[4]>0)
    {
	stringstream msg1;
	msg1 << "this is the ok one, current is: " << motor_current[0] << "'!";
	LogInfo(msg1.str());
   	kkk = palmMotorCurrentControl(0, motor_current[0]);
   	//palmMotorCurrentControl(1, motor_current[1]);
	}

	else
	{
	stringstream msg1;
	msg1 << "this is the zero one, current is: " << 0 << "'!";
	LogInfo(msg1.str());
	kkk = palmMotorCurrentControl(0,0);
		
		for(int i = 0; i < 1; i++)
		{
			motor_id = 2*i+1;
			ChooseMotor(motor_id);
			if(VCS_SetDisableState(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
			{	
				lResult = MMC_FAILED;
				LogError("VCS_SetDisableState", lResult, ulErrorCode);
				stringstream msg1;
				msg1 << "Disable motor error" << i << "'!";
				LogInfo(msg1.str());
			}	
		} 
	 

	}



   	ros::spinOnce();
    //palmMotorCurrentControl(1,50);

	 
 */


}




void PrintHeader()
{
	SeparatorLine();
	LogInfo("the motor will rotate");
	SeparatorLine();
}


bool swithToUpperSphereCallBack()
{
	
	float switch_angle[5];
	for (int i = 0; i<=2; i++)
	{
		for(int j = 0; j<=4; j++)
		{
			
			switch_angle[j] = switch_to_upper_sphere_angle[i][j];
  		
		}

		while (!motorRotationToConfiguration(motor_amount, switch_angle, test_send)) 		
  		{
  			ros::spinOnce();
  		}

	}

}



bool handCurrentChecker()
{
	bool flag = true;
	int motor_cuurent_amount = 0;
	int lResult = MMC_FAILED;
    short int currentValue = 0;
    int motor_id = 0;
    unsigned int ulErrorCode = 0;


	for (int i = 2; i<=4; i++)
	{
		motor_id = 2*i + 1;
		motor_cuurent_amount = 0;
		ChooseMotor(motor_id);
		if (VCS_GetCurrentMust(g_pKeyHandle, g_usNodeId, &currentValue, &ulErrorCode) !=0)
		{
			if (currentValue > setCurrentValue)
			{
				 motor_cuurent_amount = motor_cuurent_amount+1;
			} 

	    }

		else
		{
			ROS_INFO("cannot read the current of the motor");
			return false;
		}
	}

	if (motor_cuurent_amount >= 2)
	{
		//haltPalmJoint(5);//disableMotor();
		
		LogInfo("current_checker, all motors' current are too large");
		return false;

	} 

	else return true;
}



int GetMotorVelocity(int motor_id)
{

	int lResult = MMC_FAILED;
    int currentVelocity = 0;
    unsigned int ulErrorCode = 0;


		//motor_cuurent_amount = 0;

		ChooseMotor(motor_id);
		if (VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, &currentVelocity, &ulErrorCode) !=0)
		{

				 //stringstream msg1;
				 //msg1 << "the motor velocity is, motor id = " << g_usNodeId << ", velocity " << currentVelocity;
			     //LogInfo(msg1.str());
			     return currentVelocity;
				 //motor_cuurent_amount = motor_cuurent_amount+1;
			

	    }

		else
		{
			//ROS_INFO("cannot read the velocity of the motor");
			return false;
		}
}








bool MotorCurrentIndividualChecker()
{
	bool flag = true;
	int lResult = MMC_FAILED;
    short int currentValue = 0;
    int motor_id = 0;
    unsigned int ulErrorCode = 0;


	for (int i = 2; i<=4; i++)
	{
		motor_id = 2*i + 1;
		//motor_cuurent_amount = 0;

		ChooseMotor(motor_id);
		if (VCS_GetCurrentIs(g_pKeyHandle, g_usNodeId, &currentValue, &ulErrorCode) !=0)
		{
			if (abs(currentValue) <= setCurrentValue)
			{
				 flag = flag * 1;
				 stringstream msg1;
				 msg1 << "the motor current is, motor id = " << g_usNodeId << ", current " << currentValue;
			     LogInfo(msg1.str());

				 //motor_cuurent_amount = motor_cuurent_amount+1;
			} 

			else 
			{

				stringstream msg1;
				msg1 << "the motor current is, motor id = " << g_usNodeId << ", current " << currentValue;
			    LogInfo(msg1.str());
				flag = flag * 0;
				LogInfo("the hand should stop here because of the large current");
			}

	    }

		else
		{
			ROS_INFO("cannot read the current of the motor");
			return false;
		}
	}

	return flag;
}



bool handConfigurationCheck()
{
	ros::spinOnce();
	bool flag = true;
	for (int i = 0; i<=4; i++)
	{

		if ((test_send.position[i] > handConfigurationLowerLimit[i]) && (test_send.position[i] < handConfigurationUpperLimit[i]))
		{
			flag = flag * 1;
		}

		else 
		{
			flag = flag * 0;
			stringstream msg1;
			msg1 << "the joint" << i << " exceeds the limit. The value is"  << test_send.position[i];
			LogInfo(msg1.str());
		}
	}

	if (flag == false)
	{
		LogInfo("configuration checker fails, invalid configuration, stop ");
		disableMotor();
		return false;
	} 

	else return flag;
}



void disableMotor()
{
	unsigned int ulErrorCode = 0;
	int motor_id;
	int lResult = MMC_FAILED;
	for(int i = 0; i < 5; i++)
	{
		motor_id = 2*i+1;
		ChooseMotor(motor_id);
		if(VCS_SetDisableState(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
		{	
			lResult = MMC_FAILED;
			LogError("VCS_SetDisableState", lResult, ulErrorCode);
			stringstream msg1;
			msg1 << "Disable motor error" << i << "'!";
			LogInfo(msg1.str());
		}	
	} 
	
}



bool holdConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send)
{
	PrintHeader();
    unsigned short index = 0;
    float scale = 1.0;
    bool targetReachFlag;
    int judge_traget_reach = 0;
    int motor_id = 1;
    unsigned int ulErrorCode = 0;
    long motor_rotation_value = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float target_position_motor;

    //while(judge_traget_reach != motorTotalNumber)
	//{

	 	for(index=0; index<motorTotalNumber; index++)
	 	{
		 	motor_id = 2*index + 1;
		    if(flag==1) motor_rotation_value = (long)(test_send.position[index]*scale);		    
		    if(motor_state == 1)
			{
				ChooseMotor(motor_id);
				


				if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("OpenDevice", lResult, ulErrorCode);
					return lResult;
				}

				if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("PrepareDemo", lResult, ulErrorCode);
					return lResult;
				}
								
								
				//target_position_motor = (long)configurationData[index];
				target_position_motor = configurationData[index];
				stringstream msg1,msg2;
				msg1 << "the target_posiiton'" << target_position_motor << "'!";
				msg2 << "sensor position'" << motor_rotation_value << "'!";
				LogInfo(msg1.str());
				LogInfo(msg2.str());

				if((abs((long)target_position_motor - motor_rotation_value)) > 12.0 && (target_position_motor !=0))
				{
					if ((target_position_motor - motor_rotation_value) >= 0) //move down, close
					{

						if((lResult = Demo(&ulErrorCode, 500*motor_direction[index]))!=MMC_SUCCESS)
						{

							return lResult;
						}
					}

					else
					{
						if((lResult = Demo(&ulErrorCode, -500*motor_direction[index]))!=MMC_SUCCESS)
						{
												//LogError("Demo", lResult, ulErrorCode);
							return lResult;
						}
					}
									
				}

				else 
				{
					judge_traget_reach = judge_traget_reach + 1;
					if((lResult = Demo(&ulErrorCode, 0))!=MMC_SUCCESS)
					{
						stringstream msg1,msg2;
						msg1 << "the target_posiitonBBB'" << target_position_motor << "'!";
						msg2 << "sensor positionBBBB'" << motor_rotation_value << "'!";
						LogInfo(msg1.str());
						LogInfo(msg2.str());

						lResult = CloseDevice(&ulErrorCode);
						return lResult;
					}

				}
				
												
	 		}

		}

	if (judge_traget_reach == motorTotalNumber)
	{
		LogInfo("halt velocity movement");

		for(int index=0; index<motorTotalNumber; index++)
	 	{
			motor_id = 2*index + 1;
			if(VCS_HaltVelocityMovement(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
			{
					lResult = MMC_FAILED;
					LogError("VCS_HaltVelocityMovement", lResult, ulErrorCode);
					stringstream msg1;
					msg1 << "halt motor'" << index << "'!";
					LogInfo(msg1.str());
						
			}
		
		}

		return true;	
	}

	ros::spinOnce();


}


void haltPalmJoint(unsigned short motorTotalNumber)
{
	LogInfo("halt palm velocity movement");
	int motor_id =0;
	unsigned int ulErrorCode = 0;	        
	int lResult = MMC_FAILED;

	for(int index=2; index<motorTotalNumber; index++)
	{
		motor_id = 2*index + 1;
		if(VCS_HaltVelocityMovement(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
		{
			lResult = MMC_FAILED;
			LogError("VCS_HaltVelocityMovement", lResult, ulErrorCode);
			stringstream msg1;
			msg1 << "halt motor'" << index << "'!";
			LogInfo(msg1.str());
						
		}
		
	}
}



bool motorRotationWithVelocity(unsigned short motorTotalNumber)
{
	PrintHeader();
    unsigned short index = 0;
    float scale = 1.0;
    bool targetReachFlag;
    int judge_traget_reach = 0;
    int motor_id = 1;
    unsigned int ulErrorCode = 0;
    long motor_rotation_value = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float target_position_motor;


    //while((MotorCurrentIndividualChecker() == true) && (handConfigurationCheck() == true))
    while(MotorCurrentIndividualChecker() == true)
    	
    {
	 	
	 	for(index=2; index<motorTotalNumber; index++)
	 	{

		 	motor_id = 2*index + 1;
		    //if(flag==1) motor_rotation_value = (long)(test_send.position[index]*scale);		    
		    
		    if(motor_state == 1)
			{
				ChooseMotor(motor_id);
				

				if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("OpenDevice", lResult, ulErrorCode);
					return lResult;
				}

				//if (MotorCurrentIndividualChecker() == false) goto AAA;

				if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("PrepareDemo", lResult, ulErrorCode);
					return lResult;
				}

				if((lResult = Demo(&ulErrorCode, motorGraspingSpeed*motor_direction[index]))!=MMC_SUCCESS)
				{
					return lResult;
				}
			}

		}

		//LogInfo("force too large");

	}

	//AAA:
	/*
	while (handConfigurationCheck() == true)
	{
		LogInfo("hold grasping force");
		int current = holdMotorCurrent();
	}
	
	LogInfo("hold current mode fails");
	*/

	LogInfo("halt velocity movement");
	for(int index=2; index<motorTotalNumber; index++)
	{
			motor_id = 2*index + 1;
			if(VCS_HaltVelocityMovement(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
			{
				lResult = MMC_FAILED;
				LogError("VCS_HaltVelocityMovement", lResult, ulErrorCode);
				stringstream msg1;
				msg1 << "halt motor'" << index << "'!";
				LogInfo(msg1.str());
						
			}				
	}
	

	return lResult;
}



bool palmMotorCurrentControl(int index, int current)
{
   

    unsigned int ulErrorCode = 0;
    short int currentValue = 0;
    int lResult = MMC_FAILED;
	unsigned int *p_pErrorCode = 0;
    int motor_id;



		motor_id = 2*index+1;
		ChooseMotor(motor_id);
		//lResult = DemoCurrentMode(g_pKeyHandle, g_usNodeId, lErrorCode, current);
		
		if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
		{
			LogError("OpenDevice", lResult, ulErrorCode);
			return lResult;
		}
	
		if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
				{
					LogError("PrepareDemo", lResult, ulErrorCode);
					return lResult;
				}
		//bool l = VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode);
		/*
		if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
		{
			LogError("PrepareDemo", lResult, ulErrorCode);
			return lResult;
		}
		*/
		
		if((lResult = CurrentDemo(&ulErrorCode, current*motor_direction[index]))!=MMC_SUCCESS)
		{
			return lResult;
		}

	
	

}






bool holdMotorCurrent()
{
   

    unsigned short index = 0;
    int motor_id = 1;
    unsigned int ulErrorCode = 0;
    short int currentValue = 0;
    int lResult = MMC_FAILED;
	unsigned int *p_pErrorCode = 0;


    for (int i=0; i<1; i++)
    {
		motor_id = 2*i+1;
		ChooseMotor(motor_id);
		//lResult = DemoCurrentMode(g_pKeyHandle, g_usNodeId, lErrorCode, current);
		
		if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
		{
			LogError("OpenDevice", lResult, ulErrorCode);
			return lResult;
		}
		bool l = VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode);
		/*
		if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
		{
			LogError("PrepareDemo", lResult, ulErrorCode);
			return lResult;
		}
		*/
		
		if((lResult = CurrentDemo(&ulErrorCode, 50*motor_direction[index]))!=MMC_SUCCESS)
		{
			return lResult;
		}

	}
}








bool motorRotationToConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send)
{
	PrintHeader();
    unsigned short index = 0;
    float scale = 1.0;
    bool targetReachFlag;
    int judge_traget_reach = 0;
    int motor_id = 1;
    unsigned int ulErrorCode = 0;
    long motor_rotation_value = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float target_position_motor;

    //while(judge_traget_reach != motorTotalNumber)
	//{

	 	for(index=0; index<motorTotalNumber; index++)
	 	{


		 	motor_id = 2*index + 1;
		    if(flag==1) motor_rotation_value = (long)(test_send.position[index]*scale);		    
		    if(motor_state == 1)
			{
				ChooseMotor(motor_id);
				

				if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("OpenDevice", lResult, ulErrorCode);
					return lResult;
				}

				if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("PrepareDemo", lResult, ulErrorCode);
					return lResult;
				}
								
								
				//target_position_motor = (long)configurationData[index];
				target_position_motor = configurationData[index];
										stringstream msg1,msg2;
						msg1 << "the target_posiiton'" << target_position_motor << "'!";
						msg2 << "sensor position'" << motor_rotation_value << "'!";
						LogInfo(msg1.str());
						LogInfo(msg2.str());

				if((abs((long)target_position_motor - motor_rotation_value)) > 8.0 && (target_position_motor !=0))
				{
					if ((target_position_motor - motor_rotation_value) >= 0) //move down, close
					{



						if((lResult = Demo(&ulErrorCode, 500*motor_direction[index]))!=MMC_SUCCESS)
						{

							return lResult;
						}
					}

					else
					{
						if((lResult = Demo(&ulErrorCode, -500*motor_direction[index]))!=MMC_SUCCESS)
						{
												//LogError("Demo", lResult, ulErrorCode);
							return lResult;
						}
					}

									
				}
				
				else 
				{
					judge_traget_reach = judge_traget_reach + 1;
					if((lResult = Demo(&ulErrorCode, 0))!=MMC_SUCCESS)
					{
						stringstream msg1,msg2;
						msg1 << "the target_posiitonBBB'" << target_position_motor << "'!";
						msg2 << "sensor positionBBBB'" << motor_rotation_value << "'!";
						LogInfo(msg1.str());
						LogInfo(msg2.str());

						lResult = CloseDevice(&ulErrorCode);
						return lResult;
					}

				}
												
	 		}

		}
	//}

	if (judge_traget_reach == motorTotalNumber)
	{
		LogInfo("halt velocity movement");

		for(int index=0; index<motorTotalNumber; index++)
	 	{
			motor_id = 2*index + 1;
			if(VCS_HaltVelocityMovement(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
			{
					lResult = MMC_FAILED;
					LogError("VCS_HaltVelocityMovement", lResult, ulErrorCode);
					stringstream msg1;
					msg1 << "halt motor'" << index << "'!";
					LogInfo(msg1.str());
						
			}
			
			if (motor_id >=5)
			{
				if(VCS_SetDisableState(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
				{
					lResult = MMC_FAILED;
					LogError("VCS_SetDisableState", lResult, ulErrorCode);
					stringstream msg1;
					msg1 << "Disable motor error" << index << "'!";
					LogInfo(msg1.str());
				}
	
			}
						

		}

		return true;	
	}

	else return false;	
	
}


bool closeFingerVelocityCallBack(kclhand_control::graspCurrent::Request &req, kclhand_control::graspCurrent::Response&)
{
    ROS_INFO("Close Finger and Grasp");
    unsigned short index = 0;
    int judge_traget_reach = 0;
    int motor_id;
    unsigned int ulErrorCode = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float graspForce =0;
    short int currentValue = 0;

    for(int i=0;i<=4;i++)
    {
    	temp_hand_configuration[i] = test_send.position[i];	
    }   	
	    //bool k = holdConfiguration(2, upper_sphere_central_grasp_angle, test_send);
	    //haltPalmJoint(2, upper_sphere_central_grasp_angle);

	if (handConfigurationCheck() != false) 
	{
	   	bool k = motorRotationWithVelocity(5);
	   	return true;	
	}

    else return false;
}




int sendMotorCurrent_0(float realtime_control_signal[])
{

    	return int(realtime_control_signal[0] * 200);	

    
}

int sendMotorCurrent_1(float realtime_control_signal[])
{

    	return	int(realtime_control_signal[1] * 200);	
    
    
}


bool palmStabilityCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    ROS_INFO("Palm Fuzzy Control");
    unsigned short index = 0;
    int judge_traget_reach = 0;
    int motor_id;
    unsigned int ulErrorCode = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float graspForce =0;
    short int currentValue = 0;
    float motor_current[2];




//&& abs(test_send.position[0] - 38) > 5 
    //while (palm_control_signal_data.data[0] <- 0.03 )
    while (abs(test_send.position[3]) > 5 )
    {


    motor_current[0] = palm_control_signal_data.data[0] * 500;
	motor_current[1] = palm_control_signal_data.data[1] * 500;
   	//palmMotorCurrentControl(0, long(motor_current[0]));
   	palmMotorCurrentControl(0, int(motor_current[0]));
	stringstream msg1;
	msg1 << "current is: " << int(motor_current[0]) << "'!";
	LogInfo(msg1.str());
   	ros::spinOnce();
    //palmMotorCurrentControl(1,50);

	}    

	//shutdown hand
	palmMotorCurrentControl(0,0);	


	for(int i = 0; i < 1; i++)
	{
		motor_id = 2*i+1;
		ChooseMotor(motor_id);
		if(VCS_SetDisableState(g_pKeyHandle, motor_id, &ulErrorCode) == 0)
		{	
			lResult = MMC_FAILED;
			LogError("VCS_SetDisableState", lResult, ulErrorCode);
			stringstream msg1;
			msg1 << "Disable motor error" << i << "'!";
			LogInfo(msg1.str());
		}	
	} 
    //palmMotorCurrentControl(1,0);

	//disableMotor();
    

    return false;
}








bool closeFingerCallBack(kclhand_control::graspCurrent::Request &req, kclhand_control::graspCurrent::Response&)
{
    ROS_INFO("Close Finger and Grasp");
    unsigned short index = 0;
    int judge_traget_reach = 0;
    int motor_id;
    unsigned int ulErrorCode = 0;
    int motor_state =1;
    int lResult = MMC_FAILED;
    float graspForce =0;
    short int currentValue = 0;
    
    
    for(int i=0;i<=4;i++)
    {
    	temp_hand_configuration[i] = test_send.position[i];	
    }

    for(int j =100; j<= graspForce; j++)
	{   	
	    //bool k = holdConfiguration(2, upper_sphere_central_grasp_angle, test_send);
	    //haltPalmJoint(2, upper_sphere_central_grasp_angle);

	    if (handConfigurationCheck() == false) 
	    {
	    	ROS_INFO("invalid hannd configuration");
	    	break;	
	    }

	    for(index=2; index<5; index++)
	 	{
	 	    motor_id = 2*index + 1;
			if(motor_state == 1)
			{
				ChooseMotor(motor_id);
				
				if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
				{
					LogError("OpenDevice", lResult, ulErrorCode);
					return lResult;
				}

				if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
				{
					LogError("PrepareDemo", lResult, ulErrorCode);
					return lResult;
				}

				if(lResult == MMC_SUCCESS)
				{

					if (VCS_GetCurrentMust(g_pKeyHandle, g_usNodeId, &currentValue, &ulErrorCode) !=0)
					{
						if (currentValue <= j)
						{
							if((lResult = CurrentDemo(&ulErrorCode, j*motor_direction[index]))!=MMC_SUCCESS)
							{
								//LogError("Demo", lResult, ulErrorCode);
								return lResult;
							}
						}
						
						else 
						{
							if((lResult = CurrentDemo(&ulErrorCode, currentValue*motor_direction[index]))!=MMC_SUCCESS)
							{
								//LogError("Demo", lResult, ulErrorCode);
								return lResult;
							}
						}
					}
				}
				
				else 
					{
						if((lResult = CurrentDemo(&ulErrorCode, 0))!=MMC_SUCCESS)
						{
							//LogError("Demo", lResult, ulErrorCode);
							//lResult = CloseDevice(&ulErrorCode);
							return lResult;
						}
		
					}
										
									
		 	}
								
		}
	}
  
	return true;
}

bool shutdownHandCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
    

    disableMotor();
  	return true; 		
}


bool openHandCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		

    unsigned int ulErrorCode = 0;
    int lResult = MMC_FAILED;

    //while(judge_traget_reach != motorTotalNumber)
	//{



				if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("OpenDevice", lResult, ulErrorCode);
					return lResult;
				}

				if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
				{
										//LogError("PrepareDemo", lResult, ulErrorCode);
					return lResult;
				}
	return true;
    		
}




bool openFingerCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
	while (!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle_com, test_send)) 		

  	{
  			ros::spinOnce();
  			//if (handConfigurationCheck(test_send) == false) break;

  	}

  	while (!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) 		

  	{
  			ros::spinOnce();
  			//if (handConfigurationCheck(test_send) == false) break;

  	}



  	return true; 		
}

bool LowerToUpperCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
	float switch_angle[5];
	for (int i = 0; i<=2; i++)
	{
		for(int j = 0; j<=4; j++)
		{
			
			switch_angle[j] = switch_to_upper_sphere_angle[i][j];
  		
		}

		while (!motorRotationToConfiguration(motor_amount, switch_angle, test_send)) 		
  		{
  			ros::spinOnce();
  		}

	}

 	
  	return true; 		
}


bool UpperToLowerCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
	
	float switch_angle[5];
	for (int i = 0; i<=2; i++)
	{
		for(int j = 0; j<=4; j++)
		{
			
			switch_angle[j] = switch_to_lower_sphere_angle[i][j];
  		
		}

		while (!motorRotationToConfiguration(motor_amount, switch_angle, test_send)) 		
  		{
  			ros::spinOnce();
  		}

	}



  	
  	return true; 		
}






bool calibrateHandCallBack(kclhand_control::graspPreparation::Request&, kclhand_control::graspPreparation::Response &res)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
	while (!motorRotationToConfiguration(motor_amount, calibration_angle_traget1, test_send)) 		

  	{
  			ros::spinOnce();
  			//if (handConfigurationCheck(test_send) == false) break;

  	}


	while (!motorRotationToConfiguration(motor_amount, calibration_angle_traget2, test_send)) 		

  	{
  			ros::spinOnce();
  			//if (handConfigurationCheck(test_send) == false) break;

  	}


	while (!motorRotationToConfiguration(motor_amount, calibration_angle_traget3, test_send)) 		

  	{
  			ros::spinOnce();
  			//if (handConfigurationCheck(test_send) == false) break;

  	}

  	res.preparationFlag = 1;
  	return true; 		
}








/*
void metahandReset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{

	if ((test_send.position[0] < 0) && (test_send.position[1] < 0))
	{

	}

	else
	{

	}


  	//res.preparationFlag = 1;
		
}



*/



/*

1. Unity the input, map it to the current model
2. read realtime angular velocity of the palm joints
3. set desired palm joints value

4. send it to py to calculate

5. send back the cpp file for reaction 
6. create data flow and new launch file

unit; rad
*/


void palm_force_control_call_back(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    palm_force_control_data = *msg;
  //the 3rd motor is special, twice rate, inverse direction. But it's not anymore now
//    tendon_speed.data[2] = -2*tendon_speed.data[2];
//   printf("%f", tendon_speed.data[0]);
    
    
  	   
  //joint_target_position.position[0] = 90.0;
}




int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	unsigned short index = 0;
	long motor_velocity = 0;
	long aaa = 0;
	long motor_rotation_value = 0;
	int motor_state =1;
	ros::Time begin, ending, time_pub;
	float diameter = 0.015;
	

	float ratio = 40.0/44.0;
	int duration;
	float test;
	float target_position_motor = 0;
	std_msgs::Int64MultiArray send_rpm;
	std_msgs::Int64 time_nsec;
	
	ros::init(argc,argv, "send_to_motor");
	ros::NodeHandle n;
	ros::Rate loop_rate(500.0);
	


	ros::Subscriber palm_control_signal_sub = n.subscribe("palm_control_signal", 2, palm_control_signal_call_back);	
	

	ros::Subscriber sub = n.subscribe("motor_value_pub", 10, set_target_position);	
	// subscribe the sensor values
	ros::Subscriber sub_sensor_value = n.subscribe("sensorToForwardKinematics", 2, sensor_value_call_back);
	

	//ros::Subscriber sub_palm_control_signal = n.subscribe("palm_control_signal", 10, palm_control_signal_call_back);

	// apply hand configurations


	
	//close fingers
	ros::ServiceServer handCloseFinger = n.advertiseService("closeFinger", closeFingerVelocityCallBack);

	//open fingers
	ros::ServiceServer handOpenFinger = n.advertiseService("openFinger", openFingerCallBack);

	ros::ServiceServer sphereToUpper = n.advertiseService("lower2upper", LowerToUpperCallBack);
	ros::ServiceServer sphereToULower = n.advertiseService("upper2lower", UpperToLowerCallBack);

    ros::ServiceServer handPrepareGrasp = n.advertiseService("prepareGrasp", openFingerCallBack);
	
	ros::ServiceServer shutdownHand = n.advertiseService("shutdownHand", shutdownHandCallBack);
	ros::ServiceServer openHand = n.advertiseService("openHand", openHandCallBack);



	ros::ServiceServer calibrateHand = n.advertiseService("calibrateHand", calibrateHandCallBack);


    //ros::ServiceServer handReset = n.advertiseService("reset", metahandReset);

	ros::Publisher time_publish = n.advertise<std_msgs::Int64>("time_nsec",10);
	ros::Publisher test_data_send = n.advertise<sensor_msgs::JointState>("test",10);
	//ros::ServiceServer plam_fuzzy_control = n.advertiseService("palm_fuzzy_control", palmFuzzyControlCallBack);

	ros::Publisher palm_disp_velo_publisher = n.advertise<std_msgs::Float64MultiArray>("palm_displacement_velocity", 10);

	// palm stability service  
	ros::ServiceServer palm_stability = n.advertiseService("palm_stability", palmStabilityCallBack);




	SetDefaultParameters();
	begin = ros::Time::now();
	
	//PrintHeader();
	int motor_id = 1;
	while(ros::ok())// && (duration<11))
		{
				
	 			//palm_disp_velo_publisher.publish(palm_state_data_send);
	 			ros::spinOnce();	 			

		}

	

  	return 0;

}


