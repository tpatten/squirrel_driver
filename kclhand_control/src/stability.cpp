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
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   Demo(unsigned int* p_pErrorCode);
bool  motorRotationToConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send);
int mapHandConfiguration(string hand_configuration);
bool compareConfiguration(float currentOne[], float saveOne[]);
bool holdConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send);
bool handConfigurationCheck();
void disableMotor();
bool handCurrentChecker();
bool holdMotorCurrent();
bool MotorCurrentIndividualChecker();
int PrepareVelocityDemo(unsigned int* p_pErrorCode);
bool compareConfigurationType(float configurationData[], sensor_msgs::JointState test_send);
bool palmMotorCurrentControl(int index, int current);
int DemoCurrentMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, long current_limit);
bool impedanceMotorRotationToConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send);



std_msgs::Float64MultiArray tendon_speed;
std_msgs::Float64MultiArray fingertip_sensor_data;
std_msgs::Float64MultiArray impedance_signal_data;
std_msgs::Float64MultiArray impedance_data_check;



sensor_msgs::JointState joint_target_position;
sensor_msgs::JointState real_time_sensor_value;
sensor_msgs::JointState test_send;
sensor_msgs::JointState target_position_temp;


std_msgs::Float64MultiArray palm_control_signal_data;

float temp_position[5];

std::vector<int> motor_direction;
int set_current_value;
int set_holding_current;
int motor_velocity_setup = 500;

std::vector<float> close_finger_configuration_data;
std::vector<float> open_finger_configuration_data;

float open_finger_configuration[5];
float close_finger_configuration[5];
float temp_hand_configuration[5];
float upper_sphere_central_grasp_palm[2] = {-40,-40};
float handConfigurationLowerLimit[5] ={-60,-60,-80,-80,-80};
float handConfigurationUpperLimit[5] ={80,80,70,80,70};

float impedance_configuration[5] = {60,60,5,5,5};




float configurationData[5];
const int motor_amount = 5;
const unsigned int maximumCurrent = 260;
const int motorGraspingSpeed = 1000;


unsigned char flag = 0;


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

    for (int i = 0; i<5;i++)
  	{
  		temp_position[i] = (int)joint_target_position.position[i];
  	}
  	
  	while (!motorRotationToConfiguration(motor_amount, temp_position, test_send)) 		
  		{
  			ros::spinOnce();
  		}
}


void sensorValueCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
    test_send.position.resize(5);
    real_time_sensor_value = *msg;
    test_send = real_time_sensor_value;
    flag = 1;
}


void impedanceSignalValueCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    impedance_signal_data.data.resize(3);
    impedance_signal_data = *msg;


    //impedance_configuration[1] = 30 + -impedance_signal_data.data[1] * 20.0;
    impedance_configuration[3] = -5 + impedance_signal_data.data[0] * 40.0;
	impedance_configuration[4] = -5 + impedance_signal_data.data[1] * 20.0;
	impedance_configuration[2] = -5 + impedance_signal_data.data[2] * 20.0;

	//impedance_configuration[3] = -60 + impedance_signal_data.data[0] * 40.0;
	//impedance_configuration[4] = -60 + impedance_signal_data.data[1] * 20.0;
	//impedance_configuration[2] = -60 + impedance_signal_data.data[2] * 20.0;
	

    //impedance_configuration[3] = test_send.position[3] + impedance_signal_data.data[0] * 20.0;
	//impedance_configuration[4] = test_send.position[4] + impedance_signal_data.data[1] * 20.0;


    impedance_data_check.data.resize(5);

    for(int i = 0; i<=4; i++)
    {
    	
    	impedance_data_check.data[i] = impedance_configuration[i];

    }

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




bool checkIfGraspObject()
{

    msleep(1500);
	ros::spinOnce();
	bool flag = true;
	for (int i = 2; i<=4; i++)
	{
		if (abs(test_send.position[i] - temp_hand_configuration[i]) <= 4)
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

	else return true;
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


    while(handConfigurationCheck() == true)
    {
    	for(int i=0;i<=4;i++)
   		{
    		temp_hand_configuration[i] = test_send.position[i];	
    	}   	
	 	
	 	for(index=2; index<motorTotalNumber; index++)
	 	{

		 	motor_id = 2*index + 1;		    
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

				if((lResult = Demo(&ulErrorCode, motorGraspingSpeed*motor_direction[index]))!=MMC_SUCCESS)
				{
					return lResult;
				}
			}

		}

		if(checkIfGraspObject()) break;
		else continue;
	
	}


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



bool impedanceMotorRotationToConfiguration(unsigned short motorTotalNumber, float configurationData[], sensor_msgs::JointState test_send)
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

	for(index=2; index<motorTotalNumber; index++)
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
			}

        }



	for(index=2; index<motorTotalNumber; index++)
	 	{
		 	motor_id = 2*index + 1;
		    if(flag==1) motor_rotation_value = (long)(test_send.position[index]*scale);		    
		    if(motor_state == 1)
			{
				ChooseMotor(motor_id);							
				//target_position_motor = (long)configurationData[index];
				target_position_motor = configurationData[index];
										stringstream msg1,msg2;
						msg1 << "the target_posiiton'" << target_position_motor << "'!";
						msg2 << "sensor position'" << motor_rotation_value << "'!";
						LogInfo(msg1.str());
						LogInfo(msg2.str());

				if((abs((long)target_position_motor - motor_rotation_value)) > 4.0 && (target_position_motor !=0))
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

	if (judge_traget_reach == 6)
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


	if (handConfigurationCheck() != false) 
		{
	   		return motorRotationWithVelocity(5);
		}
    
    else return false;
}

bool shutdownHandCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    disableMotor();
  	return true; 		
}

bool upperOpenFingerCallBack(kclhand_control::graspPreparation::Request&, kclhand_control::graspPreparation::Response &res)
{
	int open_finger_flag = 1;	
	while (!motorRotationToConfiguration(motor_amount, open_finger_configuration, test_send)) 		
  		{
  			ros::spinOnce();
  		}
	
  	res.preparationFlag = open_finger_flag;
	return true;
}

bool testHandCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	
	for (int i = 0; i<=10; i++)
	{
		while (!motorRotationToConfiguration(motor_amount, open_finger_configuration, test_send)) 		
	  		{
	  			ros::spinOnce();
	  		}
		msleep(1000);

		while (!motorRotationToConfiguration(motor_amount, close_finger_configuration, test_send)) 		
	  		{
	  			ros::spinOnce();
	  		}
		msleep(1000);
	}
	return true;
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



bool impedanceCallBack(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	int open_finger_flag = 1;	

	while (!impedanceMotorRotationToConfiguration(5, impedance_configuration, test_send)) 		
  		{
  			ros::spinOnce();
  		}
	

	return true;
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
    while (abs(test_send.position[4]) > 5 )
    {

    motor_current[0] = palm_control_signal_data.data[0] * 500 + 100;
	motor_current[1] = palm_control_signal_data.data[1] * 500;
   	palmMotorCurrentControl(3, long(motor_current[0]));
   	//lResult = Demo(&ulErrorCode, motor_current[0]*motor_direction[4]);
					


   	//palmMotorCurrentControl(4, int(motor_current[0]));
	stringstream msg1;
	msg1 << "current is: " << int(motor_current[0]) << "'!";
	LogInfo(msg1.str());
   	ros::spinOnce();
    //palmMotorCurrentControl(1,50);

	}    

	//shutdown hand
	palmMotorCurrentControl(0,0);	


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
    //palmMotorCurrentControl(1,0);

	//disableMotor();
    
    return false;
}

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


	if(n.getParam("close_finger_configuration", close_finger_configuration_data))
	{
			for (int i = 0; i<= close_finger_configuration_data.size(); i++)
			{
				close_finger_configuration[i] = close_finger_configuration_data[i];
			}
	}

	if(n.getParam("open_finger_configuration", open_finger_configuration_data))
	{
			for (int i = 0; i<= open_finger_configuration_data.size(); i++)
			{
				open_finger_configuration[i] = open_finger_configuration_data[i];
			}
	}
	
	if(!n.getParam("set_motor_current_value", set_current_value))
		{
			n.param<int>("set_motor_current_value", set_current_value, 250);
		}

	if(!n.getParam("set_motor_holding_current", set_holding_current))
		{
			n.param<int>("set_motor_holding_current", set_holding_current, 250);
		}


	ros::Rate loop_rate(100.0); 	
	ros::Subscriber palm_control_signal_sub = n.subscribe("palm_control_signal", 2, palm_control_signal_call_back);	
	ros::Subscriber sub = n.subscribe("motor_value_pub", 10, setTargetPositionCallBack);


	ros::Publisher impedance_send = n.advertise<std_msgs::Float64MultiArray>("impedance_motor", 30);


	// subscribe the sensor values
	ros::Subscriber sub_sensor_value = n.subscribe("sensorToForwardKinematics", 30, sensorValueCallBack);
	ros::Subscriber impedance_signal = n.subscribe("impedance_signal_send", 30, impedanceSignalValueCallBack);
	

	// subscribe the fingertip data 
	ros::Subscriber fingertip_sensor_value = n.subscribe("fingertips", 10, fingertipDataCallBack);
    ros::ServiceServer handOpenFinger = n.advertiseService("open_finger", upperOpenFingerCallBack); 
	ros::ServiceServer handCloseFinger = n.advertiseService("close_finger", closeFingerCallBack);
	ros::ServiceServer shutdownHand = n.advertiseService("shutdown_hand", shutdownHandCallBack);
	ros::ServiceServer calibrateHand = n.advertiseService("calibrate_hand", upperOpenFingerCallBack);
	ros::ServiceServer testHand = n.advertiseService("test_hand", testHandCallBack);

	ros::ServiceServer palm_stability = n.advertiseService("palm_stability", palmStabilityCallBack);

	ros::ServiceServer impedance_controller = n.advertiseService("impedance_control", impedanceCallBack);


	
	

	SetDefaultParameters();
	
	//PrintHeader();
	int motor_id = 1;
	while(ros::ok())// && (duration<11))
		{
	 			ros::spinOnce();

	 			impedance_send.publish(impedance_data_check);
	 			
	 			loop_rate.sleep();
		}
	

  	return 0;

}


