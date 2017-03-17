#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>
#include <iostream>
#include <cmath>


#define SENSOR_TOPIC "/wrist"

using namespace std;


std_msgs::Bool detector(std::vector<double>  wrist_sensor_values_);
std::vector<double>  wrist_sensor_values_(6,0.0);
std::vector<double>  f_mags;
void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg);

int main(int argc, char** args) {

    ros::init(argc, args, "wrist_safety");
    ros::NodeHandle node;
    ros::Rate lrate(50.0);

    ros::Publisher safety_pub_  = node.advertise<std_msgs::Bool>("/wrist/wrist_bumper", 1);

    sleep(1);
    std_msgs::Bool detected_ ;
    detected_.data = false;
    
    while(1){

        detected_ = detector(wrist_sensor_values_);
        safety_pub_.publish(detected_);

        ros::spinOnce();
        lrate.sleep();
    }


    return 0;

}

void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg){

    wrist_sensor_values_.at(0) = msg.data.at(0);
    wrist_sensor_values_.at(1) = msg.data.at(1);
    wrist_sensor_values_.at(2) = msg.data.at(2);
    wrist_sensor_values_.at(3) = msg.data.at(3);
    wrist_sensor_values_.at(4) = msg.data.at(4);
    wrist_sensor_values_.at(5) = msg.data.at(5);

}

std_msgs::Bool detector(std::vector<double>  wrist_sensor_values_){
    std_msgs::Bool detected_ ;
	detected_.data = false;
	
	//calculate force magnitude from x,z,y values
	double force_mag=sqrt(pow(wrist_sensor_values_.at(0),2)+pow(wrist_sensor_values_.at(1),2)+pow(wrist_sensor_values_.at(2),2));
	if(f_mags.size()<3)
	{
		f_mags.push_back(force_mag);
		return detected_ ;	//if we have less than 3 points we don't do anything
	}

			//if we have three points swap
	f_mags[0]=f_mags[1];
	f_mags[1]=f_mags[2];
	f_mags[2]=force_mag;
	
	double f_diff=f_mags[0]-f_mags[2];
	
	if(abs(f_diff)>1.8)	//threashold from experimental data
	{
		detected_.data = true;
	}
	
    return detected_ ;
}


