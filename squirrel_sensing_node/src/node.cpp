#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "../include/squirrel_sensing_node/node.h"
#include "../include/squirrel_sensing_node/sensing_drivers.h"


using namespace ros;
using namespace std;


const double SensingNode::pause=1000.0;    //this might be became a constructor parameter

//consider instantiating everything in a configure() function instead of the constructor
SensingNode::SensingNode(const std::string& name){

    this->name=name;

    //IMPORTANT: is operator= implemented by NodeHandle?
    tactile_pub=(node.advertise<std_msgs::Float64MultiArray>(name,1));    //1 is maximum number of messages sent before going in overflow
    proximity_pub=(node.advertise<std_msgs::Float64MultiArray>(name,1));    //1 is maximum number of messages sent before going in overflow
    wrist_pub=(node.advertise<std_msgs::Float64MultiArray>(name,1));    //1 is maximum number of messages sent before going in overflow

    loop_rate=new Rate(pause);

    //sensor=new Sensor();      //to implement
}

SensingNode::~SensingNode(){

    if(sensor!=NULL){
        delete sensor;
    }

    if(loop_rate!=NULL){
        delete loop_rate;
    }

}

void SensingNode::run(){ //this function will make the node loop as long as ros::ok() is true

    //here can be added every function needed to set up the sensor execution (i.e. config file reading)
    //sensor->setup();

    while(ros::ok()){
        //read from sensor

        //fill in msg

        //ROS_INFO("Broadcasting...")
        //tactile_pub->publish(msg);

        ros::spinOnce();

        loop_rate->sleep();
    }

}


string& SensingNode::getName(){
    return name;
}
