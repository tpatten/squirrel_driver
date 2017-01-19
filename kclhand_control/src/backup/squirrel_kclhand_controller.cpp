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
int checkPalmSphere();
bool compareConfigurationType(float configurationData[], sensor_msgs::JointState test_send);


std_msgs::Float64MultiArray tendon_speed;
std_msgs::Float64MultiArray fingertip_sensor_data;

sensor_msgs::JointState joint_target_position;
sensor_msgs::JointState real_time_sensor_value;
sensor_msgs::JointState test_send;
sensor_msgs::JointState target_position_temp;

float temp_position[5];


/*
The motor_direction[5] is the only difference between all the hand produced by KCL
It defines the motor moving direction, just based on the set-up of the motors 
**for the hand in UIBK, motor_direction[5] = {1, -1, -1, 1, 1};
**for the hand in wien, motor_direction[5] = 

*/
std::vector<int> motor_direction;
int set_current_value;
int set_holding_current;
int motor_velocity_setup = 500;
/*
several types of grasping configurations

float lower_sphere_parallel_grasp_angle[5] = {25,25,-20,-20,-20};
float lower_sphere_cylindrical_grasp_angle[5] = {60,60,5,-10,5};
float lower_sphere_central_grasp_angle[5] = {40,40,-5,-10,-5};
float upper_sphere_parallel_grasp_angle[5] = {-15,-15,-20,-20,-20};
float upper_sphere_cylindrical_grasp_angle[5] = {-60,-60,-60,-40,-60};
float upper_sphere_central_grasp_angle[5] = {-30,-35,-60,-45,-60};
*/


std::vector<float> lower_sphere_parallel_grasp_angle_data;
std::vector<float> lower_sphere_cylindrical_grasp_angle_data;
std::vector<float> lower_sphere_central_grasp_angle_data;
std::vector<float> upper_sphere_parallel_grasp_angle_data;
std::vector<float> upper_sphere_cylindrical_grasp_angle_data;
std::vector<float> upper_sphere_central_grasp_angle_data;
std::vector<float> close_finger_configuration_data;
std::vector<float> open_finger_configuration_data;

float lower_sphere_parallel_grasp_angle[5];
float lower_sphere_cylindrical_grasp_angle[5];
float lower_sphere_central_grasp_angle[5];
float upper_sphere_parallel_grasp_angle[5];
float upper_sphere_cylindrical_grasp_angle[5];
float upper_sphere_central_grasp_angle[5];

float open_finger_configuration[5] = {-30,-30,-60,-45,-60};
float close_finger_configuration[5];

float temp_hand_configuration[5];
float upper_sphere_central_grasp_palm[2] = {-40,-40};
float handConfigurationLowerLimit[5] ={-60,-60, -80, -80, -80};
float handConfigurationUpperLimit[5] ={80,80, 70, 80, 70};

//float calibration_angle_traget1[5] = {-30,-35,-10,-5,-10};
//float calibration_angle_traget2[5] = {-30,-35,-75,-60,-75};
//float calibration_angle_traget3[5] = {-30,-35,-60,-45,-60};

float calibration_angle_traget1[5] = {-30,-30,-50,30,-50};
float calibration_angle_traget2[5] = {-30,-30,-5,-30,-5};


//float calibration_angle_traget3[5] = {-30,-35,-60,-45,-60};



float configurationData[5];
const int motor_amount = 5;
const unsigned int maximumCurrent = 260;
const int motorGraspingSpeed = 1000;

float switch_to_upper_sphere_angle[3][5] = {{60,60,-20,-20,-20},
							  			    {60,-10,-20,-20,-20},
								 		    {-20,-20,-30,-30,-30}};

float switch_to_lower_sphere_angle[3][5] = {{-30,-30,-20,-20,-20},
								  			{70,-10,-20,-20,-20},
								  			{60, 60,-30,-30,-30}};




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

int msleep(unsigned long milisec)   
{   
    struct timespec req={0};   
    time_t sec=(int)(milisec/1000);   
    milisec=milisec-(sec*1000);   
    req.tv_sec=sec;   
    req.tv_nsec=milisec*1000000L;   
    while(nanosleep(&req,&req)==-1)   
        continue;   
    return 1;   
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

	msg << "set current mode, node = " << p_usNodeId;
	//LogInfo(msg.str());
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
					LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
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
		LogError("DemoProfileVelocityMode", lResult, lErrorCode);
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


void setTargetPositionCallBack(const sensor_msgs::JointState::ConstPtr& msg)
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


void sensorValueCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
    test_send.position.resize(5);
    real_time_sensor_value = *msg;
    test_send = real_time_sensor_value;
    flag = 1;
}


void fingertipDataCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    fingertip_sensor_data.data.resize(5);
    fingertip_sensor_data = *msg;
    flag = 1;
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

		if (VCS_GetCurrentMust(g_pKeyHandle, g_usNodeId, &currentValue, &ulErrorCode) !=0)
		{
			if (currentValue > set_current_value)
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


bool MotorCurrentIndividualChecker()
{
	bool flag = false;
	int lResult = MMC_FAILED;
    short int currentValue = 0;
    int motor_id = 0;
    unsigned int ulErrorCode = 0;


	for (int i = 2; i<=4; i++)
	{
		motor_id = 2*i + 1;
		//motor_cuurent_amount = 0;
		flag = true;

		if (VCS_GetCurrentMust(g_pKeyHandle, g_usNodeId, &currentValue, &ulErrorCode) !=0)
		{
			if (currentValue <= set_current_value)
			{
				 flag = flag * 1;
				 //motor_cuurent_amount = motor_cuurent_amount+1;
			} 

			else 
			{
				flag = flag * 0;
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


bool checkIfGraspObject()
{
	for(int i=0;i<=4;i++)
    {
    	temp_hand_configuration[i] = test_send.position[i];	
    }  
    msleep(200);
	ros::spinOnce();
	bool flag = true;
	for (int i = 2; i<=4; i++)
	{

		if (abs(test_send.position[i] - temp_hand_configuration[i]) <= 2)
		{


			flag = flag * 1;
		}

		else 
		{
			flag = flag * 0;
		}
	}

	if (flag == false)
	{
		LogInfo("Grasping in process");
		disableMotor();
		return false;
	} 

	else return flag;
}


int checkPalmSphere()
{
	ros::spinOnce();
	if (test_send.position[0] >= 0 && test_send.position[1] >= 0)
	{
		return 1; //lower hemisphere
	}
	
	else if (test_send.position[0] < 0 && test_send.position[1] < 0)
	{
		return 2; //upper hemisphere
	}

	else
	{
		return 0; // a invalid configuration
	}

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

						if((lResult = Demo(&ulErrorCode, motor_velocity_setup*motor_direction[index]))!=MMC_SUCCESS)
						{

							return lResult;
						}
					}

					else
					{
						if((lResult = Demo(&ulErrorCode, -motor_velocity_setup*motor_direction[index]))!=MMC_SUCCESS)
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


    while((checkIfGraspObject() == false) && (handConfigurationCheck() == true))
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
	

	return lResult;
}





bool holdMotorCurrent()
{
   

    unsigned short index = 0;
    int motor_id = 1;
    unsigned int ulErrorCode = 0;
    short int currentValue = 0;
    int lResult = MMC_FAILED;
	unsigned int *p_pErrorCode = 0;


    for (int i=2; i<5; i++)
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
		
		if((lResult = CurrentDemo(&ulErrorCode, set_holding_current*motor_direction[index]))!=MMC_SUCCESS)
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



						if((lResult = Demo(&ulErrorCode, motor_velocity_setup*motor_direction[index]))!=MMC_SUCCESS)
						{

							return lResult;
						}
					}

					else
					{
						if((lResult = Demo(&ulErrorCode, -motor_velocity_setup*motor_direction[index]))!=MMC_SUCCESS)
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


bool holdPalmConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send)
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



						if((lResult = Demo(&ulErrorCode, motor_velocity_setup*motor_direction[index]))!=MMC_SUCCESS)
						{

							return lResult;
						}
					}

					else
					{
						if((lResult = Demo(&ulErrorCode, -motor_velocity_setup*motor_direction[index]))!=MMC_SUCCESS)
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
	   	return motorRotationWithVelocity(5);
	}

    else return false;
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
	    //bool k = holdConfiguration(2, upper_sphere_central_grasp_angle, test_send);
	    //haltPalmJoint(2, upper_sphere_central_grasp_angle);

	
	if (handConfigurationCheck() != false) 
	{
	   	return motorRotationWithVelocity(5);
	}

    else return false;
}




bool closeFingerSensedCallBack(kclhand_control::graspCurrent::Request &req, kclhand_control::graspCurrent::Response&)
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


bool compareConfigurationType(float configurationData[])
{
	bool compareFlag = 1;
	for (int i = 0 ; i<= 4; i++)
	{
		if(abs(configurationData[i]-test_send.position[i]) <= 5)
		{
			compareFlag = compareFlag * 1;
		}

		else
		{
			compareFlag = compareFlag * 0;	
		}
	}
	return compareFlag;
}


bool shutdownHandCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
    

    disableMotor();
  	return true; 		
}

bool resetHandCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
    int open_finger_flag = 1;

	if (checkPalmSphere() == 1)  //lower hemisphere
	{
		while (!motorRotationToConfiguration(motor_amount, lower_sphere_parallel_grasp_angle, test_send)) 		
  		{
  			ros::spinOnce();
  		}
	}

	if(checkPalmSphere() == 2)
	{
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

		while (!motorRotationToConfiguration(motor_amount, lower_sphere_parallel_grasp_angle, test_send)) 		
  		{
  			ros::spinOnce();
  		}

 	}


 	if (checkPalmSphere() == 0)
 	{
 		// palm in a wierd configuration 
 	}
  	
  	return compareConfigurationType(lower_sphere_parallel_grasp_angle); 

   

    disableMotor();
  	return true; 		
}




bool openFingerCallBack(kclhand_control::graspPreparation::Request&, kclhand_control::graspPreparation::Response &res)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
	
	// check with hemisphere the palm locate
	int open_finger_flag = 1;

	if (checkPalmSphere() == 1)  //lower hemisphere
	{
		while (!motorRotationToConfiguration(motor_amount, lower_sphere_parallel_grasp_angle, test_send)) 		
  		{
  			ros::spinOnce();
  			if (handConfigurationCheck() == false) 
  			{
  				open_finger_flag = 0;
  				res.preparationFlag = open_finger_flag;
  				break;
  			}
  		}
	}
	
	else if (checkPalmSphere() == 2) //upper hemisphere
	{
		while (!motorRotationToConfiguration(motor_amount, upper_sphere_parallel_grasp_angle, test_send)) 		
  		{
  			ros::spinOnce();
  			if (handConfigurationCheck() == false) 
  			{
  				open_finger_flag = 0;
  				res.preparationFlag = open_finger_flag;
  				break;	
  			}

  		}
	}

	else
	{
		while (!motorRotationToConfiguration(motor_amount, upper_sphere_parallel_grasp_angle, test_send)) 		
  		{
  			ros::spinOnce();
  			if (handConfigurationCheck() == false) 
  			{
  				open_finger_flag = 0;
  				res.preparationFlag = open_finger_flag;
  				break;
  			}
  		}
	}
	
	res.preparationFlag = open_finger_flag;
	return open_finger_flag;
	
}



bool upperOpenFingerCallBack(kclhand_control::graspPreparation::Request&, kclhand_control::graspPreparation::Response &res)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
	
	// check with hemisphere the palm locate
	int open_finger_flag = 1;


	while (!motorRotationToConfiguration(motor_amount, open_finger_configuration, test_send)) 		
  		{
  			ros::spinOnce();
  			//if (handConfigurationCheck() == false) 
  			//{
  			//	open_finger_flag = 0;
  				//res.preparationFlag = open_finger_flag;
  				//break;
  			//}
  		}
	
  	res.preparationFlag = open_finger_flag;


	return open_finger_flag;
	
}



bool showHandConfigurationCallBack(kclhand_control::graspPreparation::Request&, kclhand_control::graspPreparation::Response &res)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  	
  	res.preparationFlag = 1;
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
/*

	while (!motorRotationToConfiguration(motor_amount, calibration_angle_traget3, test_send)) 		

  	{
  			ros::spinOnce();
  			//if (handConfigurationCheck(test_send) == false) break;

  	}
*/
  	res.preparationFlag = 1;
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


//grasp types Callback

bool lowerParallelCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  		
	int open_finger_flag = 1;

	if (checkPalmSphere() == 1)  //lower hemisphere
	{
		while (!motorRotationToConfiguration(motor_amount, lower_sphere_parallel_grasp_angle, test_send) && handConfigurationCheck()) 		
  		{
  			ros::spinOnce();
  		}
	}

	if(checkPalmSphere() == 2)
	{
		float switch_angle[5];
		for (int i = 0; i<=2; i++)
		{
			for(int j = 0; j<=4; j++)
			{
				
				switch_angle[j] = switch_to_lower_sphere_angle[i][j];
	  		
			}

			while (!motorRotationToConfiguration(motor_amount, switch_angle, test_send)  && handConfigurationCheck()) 		
	  		{
	  			ros::spinOnce();
			}

		}

		while (!motorRotationToConfiguration(motor_amount, lower_sphere_parallel_grasp_angle, test_send) && handConfigurationCheck()) 		
  		{
  			ros::spinOnce();
  		}

 	}
  	
  	return compareConfigurationType(lower_sphere_parallel_grasp_angle); 		
}


bool upperParallelCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	//while ((!motorRotationToConfiguration(motor_amount, upper_sphere_central_grasp_angle, test_send)) && (handConfigurationCheck(test_send) == true)) 		
  		
	int open_finger_flag = 1;

	if (checkPalmSphere() == 2)  //upper hemisphere
	{
		while (!motorRotationToConfiguration(motor_amount, upper_sphere_parallel_grasp_angle, test_send) && handConfigurationCheck()) 		
  		{
  			ros::spinOnce();
  		}
	}

	if(checkPalmSphere() == 1)
	{
		float switch_angle[5];
		for (int i = 0; i<=2; i++)
		{
			for(int j = 0; j<=4; j++)
			{
				
				switch_angle[j] = switch_to_upper_sphere_angle[i][j];
	  		
			}

			while (!motorRotationToConfiguration(motor_amount, switch_angle, test_send)  && handConfigurationCheck()) 		
	  		{
	  			ros::spinOnce();
			}

		}

		while (!motorRotationToConfiguration(motor_amount, upper_sphere_parallel_grasp_angle, test_send) && handConfigurationCheck()) 		
  		{
  			ros::spinOnce();
  		}

 	}
  	
  	return compareConfigurationType(upper_sphere_parallel_grasp_angle); 		
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


int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;
	unsigned short index = 0;
	long motor_velocity = 0;
	int motor_state =1;

	float target_position_motor = 0;

	
	ros::init(argc,argv, "squirrel_kclhand_controller");
	ros::NodeHandle n;
	
	n.getParam("motor_direction", motor_direction);	

	if(n.getParam("lower_sphere_parallel_grasp", lower_sphere_parallel_grasp_angle_data))
	{
			for (int i = 0; i<= lower_sphere_parallel_grasp_angle_data.size(); i++)
			{
				lower_sphere_parallel_grasp_angle[i] = lower_sphere_parallel_grasp_angle_data[i];
			}

	}

	if(n.getParam("close_finger_configuration", close_finger_configuration_data))
	{
			for (int i = 0; i<= close_finger_configuration_data.size(); i++)
			{
				close_finger_configuration[i] = close_finger_configuration_data[i];
			}

	}

//	if(n.getParam("open_finger_configuration", open_finger_configuration_data))
//	{
//			for (int i = 0; i<= open_finger_configuration_data.size(); i++)
//			{
//				open_finger_configuration[i] = open_finger_configuration_data[i];
//			}
//
//	}


	if(n.getParam("upper_sphere_parallel_grasp", upper_sphere_parallel_grasp_angle_data))
	{
			for (int i = 0; i<= upper_sphere_parallel_grasp_angle_data.size(); i++)
			{
				upper_sphere_parallel_grasp_angle[i] = upper_sphere_parallel_grasp_angle_data[i];
			}
	}



	//n.getParam("lower_sphere_cylindrical_grasp", lower_sphere_cylindrical_grasp_angle_data);
	//n.getParam("lower_sphere_central_grasp", lower_sphere_central_grasp_angle_data);
	
	//n.getParam("upper_sphere_cylindrical_grasp", upper_sphere_cylindrical_grasp_angle_data);
	//n.getParam("upper_sphere_central_grasp", upper_sphere_central_grasp_angle_data);
	
	
	if(!n.getParam("set_motor_current_value", set_current_value))
		{
			n.param<int>("set_motor_current_value", set_current_value, 250);
		}

	if(!n.getParam("set_motor_holding_current", set_holding_current))
		{
			n.param<int>("set_motor_holding_current", set_holding_current, 250);
		}




	ros::Rate loop_rate(10.0); 	
	ros::Subscriber sub = n.subscribe("motor_value_pub", 10, setTargetPositionCallBack);	
	// subscribe the sensor values
	ros::Subscriber sub_sensor_value = n.subscribe("sensorToForwardKinematics", 10, sensorValueCallBack);
	// subscribe the fingertip data 
	ros::Subscriber fingertip_sensor_value = n.subscribe("fingertips", 10, fingertipDataCallBack);


    //ros::ServiceServer handReset = n.advertiseService("reset", metahandReset);
	//ros::ServiceServer handOpenFinger = n.advertiseService("open_finger", openFingerCallBack); //done ; configuration check
    ros::ServiceServer handOpenFinger = n.advertiseService("open_finger", upperOpenFingerCallBack); //done ; configuration check
    
    //ros::ServiceServer handPrepareGrasp = n.advertiseService("prepare_grasp", openFingerCallBack);

	//ros::ServiceServer sphereToUpper = n.advertiseService("lower_to_upper", LowerToUpperCallBack);
	//ros::ServiceServer sphereToULower = n.advertiseService("upper_to_lower", UpperToLowerCallBack);
	ros::ServiceServer handCloseFinger = n.advertiseService("close_finger", closeFingerCallBack);
	ros::ServiceServer shutdownHand = n.advertiseService("shutdown_hand", shutdownHandCallBack);
	ros::ServiceServer calibrateHand = n.advertiseService("calibrate_hand", upperOpenFingerCallBack);
	ros::ServiceServer resetHand = n.advertiseService("reset_hand", resetHandCallBack);

	//close finger with force feedback
	//ros::ServiceServer handCloseFingerWithForceSensor = n.advertiseService("close_finger_with_force_sensor", closeFingerVelocityCallBack);

	//return cuurent hand configuratin
	//ros::ServiceServer showHandCurrentConfiguration = n.advertiseService("show_hand_current_configuration", showHandConfigurationCallBack);
	
	//grasp types
	//ros::ServiceServer lower_sphere_parallel_grasp_srv = n.advertiseService("lower_sphere_parallel_grasp", lowerParallelCallBack);
	//ros::ServiceServer lower_sphere_cylindrical_grasp_srv = n.advertiseService("lower_sphere_cylindrical_grasp", lowerCylindricalCallBack);
	//ros::ServiceServer lower_sphere_central_grasp_srv = n.advertiseService("lower_sphere_central_grasp", lowerCentralCallBack);
	//ros::ServiceServer upper_sphere_parallel_grasp_srv = n.advertiseService("upper_sphere_parallel_grasp", upperParallelCallBack);
	//ros::ServiceServer upper_sphere_cylindrical_grasp_srv = n.advertiseService("upper_sphere_cylindrical_grasp", upperCylindricalCallBack);
	//ros::ServiceServer upper_sphere_central_grasp_srv = n.advertiseService("upper_sphere_central_grasp", upperCentralCallBack);



	// service grasp type

	SetDefaultParameters();
	
	//PrintHeader();
	int motor_id = 1;
	while(ros::ok())// && (duration<11))
		{
	 			ros::spinOnce();
		}
	

  	return 0;

}


