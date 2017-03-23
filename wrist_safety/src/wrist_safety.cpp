#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <iostream>
#include <cmath>


#define SENSOR_TOPIC "/wrist"
#define RESET_TOPIC "/reset_safety"

using namespace std;

double f_diff;

std_msgs::Bool detector(std::vector<double>  wrist_sensor_values_);
std::vector<double>  wrist_sensor_values_(6,0.0);
std::vector<double>  f_mags;
void sensorReadCallbackWrist(std_msgs::Float64MultiArray msg);
void resetCallback(std_msgs::Bool msg);
std_msgs::Bool detected_ ;
double wrist_safety_threshold_;

int main(int argc, char** args) {

    ros::init(argc, args, "wrist_safety");
    ros::NodeHandle node;
    ros::Rate lrate(50.0);

    node.param("/wrist_safety_node/wrist_safety_threshold", wrist_safety_threshold_, 5.0);
    ROS_INFO("(Wrist safety) threshold set to %f", wrist_safety_threshold_);

    ros::Publisher safety_pub_  = node.advertise<std_msgs::Bool>("/wrist/wrist_bumper", 1);
    ros::Publisher diff_pub_  = node.advertise<std_msgs::Float64>("/wrist_diff", 1);
    ros::Subscriber sensor_sub_ = node.subscribe(SENSOR_TOPIC, 100, sensorReadCallbackWrist);
    ros::Subscriber safety_sub_ = node.subscribe(RESET_TOPIC, 100, resetCallback);
    sleep(1);
    detected_.data = false;
    std_msgs::Float64 diff;
    while(1){

        if (!detected_.data) detected_ = detector(wrist_sensor_values_);
        safety_pub_.publish(detected_);
        diff.data =f_diff;
        diff_pub_.publish(diff);
        if(detected_.data) cout<< " (Wrist safety) Colission detected"<<endl;
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

void resetCallback(std_msgs::Bool msg){

    if (msg.data) {
         detected_.data = false;
        cout<< " (Wrist safety) Reset safety called "<<endl;
    }
    
}



std_msgs::Bool detector(std::vector<double>  wrist_sensor_values_){
    std_msgs::Bool detected_ ;
	detected_.data = false;
	
	//calculate force magnitude from x,z,y values
	double force_mag=sqrt(pow(wrist_sensor_values_.at(0),2)+pow(wrist_sensor_values_.at(1),2)+pow(wrist_sensor_values_.at(2),2));
	//cout<<"mag "<< force_mag<<endl;
        if(f_mags.size()<3)
	{
		f_mags.push_back(force_mag);
		return detected_ ;	//if we have less than 3 points we don't do anything
	}

			//if we have three points swap
	f_mags[0]=f_mags[1];
	f_mags[1]=f_mags[2];
	f_mags[2]=force_mag;
	
	f_diff=f_mags[0]-f_mags[2];
	
		if(abs(f_diff) > wrist_safety_threshold_)	//threashold from experimental data
	{
      //                cout<< " abs diff "<< f_diff<<endl;
		detected_.data = true;
	}
        //cout<<f_diff<<endl;
	
    return detected_ ;
}


