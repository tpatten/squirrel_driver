#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "../include/squirrel_sensing_node/node.h"
#include "../include/squirrel_sensing_node/sensing_drivers.h"


using namespace ros;
using namespace std;


// The rate at which the sensors publish. 100 Hz seem enough.
const double SensingNode::pause=100.0;    //this might be became a constructor parameter

//consider instantiating everything in a configure() function instead of the constructor
SensingNode::SensingNode(const std::string& name, const std::vector<std::string>& portnames){

    this->name=name;

    tactile_pub=(node.advertise<std_msgs::Float64MultiArray>("fingertips",1));    //1 is maximum number of messages sent before going in overflow
    //proximity_pub=(node.advertise<std_msgs::Float64MultiArray>("pro",1));    //1 is maximum number of messages sent before going in overflow
    wrist_pub=(node.advertise<std_msgs::Float64MultiArray>("wrist",1));    //1 is maximum number of messages sent before going in overflow

    torqPerc_pub=(node.advertise<std_msgs::Float64MultiArray>("torque_percs",1));    //1 is maximum number of messages sent before going in overflow

    for(int i=0;i<3;i++)
        dominantTorques.push_back(0.0); //initialise dominant torque vector

    loop_rate=new Rate(pause);

    sensor=new Tactile(portnames[ArduinoPort]);      //the port name is given from command line
#ifdef _FT17_AVAIL
    wrist=new Wrist(portnames[FT17Port]);
#else
    wrist=NULL;
#endif
}

SensingNode::~SensingNode(){

    if(sensor!=NULL){
        delete sensor;
    }

    if(wrist!=NULL){
        delete wrist;
    }

    if(loop_rate!=NULL){
        delete loop_rate;
    }

}

void SensingNode::run(){ //this function will make the node loop as long as ros::ok() is true

    //here can be added every function needed to set up the sensor execution (i.e. config file reading)

    sensor->flush();
    while(ros::ok()){
        //read from sensor

        //proximity and tactile
        vector<double> vals;
        bool isValid=sensor->readData(vals);

        vector<double> &torqPerc=((Tactile*)sensor)->readTorquePerc();   //obtain percentages

        pickDominants(torqPerc);  //this can be copied blindly in the msg

        std_msgs::Float64MultiArray msg;
        msg.data.resize(vals.size());

        //fill in msg
        for(int i=0;i<vals.size();i++){
            msg.data[i]=vals.at(i);
        }

        vector<double> padProxs;
        padProxs.push_back(vals.operator[](10));
        padProxs.push_back(vals.operator[](12));
        padProxs.push_back(vals.operator[](14));

        //torque percs
        std_msgs::Float64MultiArray msgTorq;
        msgTorq.data.resize(dominantTorques.size()+padProxs.size());

        //fill in msg
        for(int i=0;i<dominantTorques.size();i++){
            msgTorq.data[i]=dominantTorques.at(i);
        }
        for(int i=dominantTorques.size(), j=0;j<padProxs.size();i++,j++){
            msgTorq.data[i]=padProxs.at(j);
        }

#ifdef _FT17_AVAIL
        //wrist
        vector<double> wriVals;

        bool isGood=wrist->readData(wriVals);

        std_msgs::Float64MultiArray msgWri;
        msgWri.data.resize(wriVals->size());

        //fill in msg
        for(int i=0;i<wriVals->size();i++){
            msgWri.data[i]=wriVals->at(i);
        }
#endif

        //ROS_INFO("Broadcasting...");
        if(isValid) //publish only if we read valid data
        {
            tactile_pub.publish(msg);
            torqPerc_pub.publish(msgTorq);
        }
#ifdef _FT17_AVAIL
        wrist_pub.publish(msgWri);
#endif

        ros::spinOnce();

        loop_rate->sleep();
    }

}

void SensingNode::pickDominants(vector<double> &torqPerc)
{
    for(int i=0,j=0;i<torqPerc.size();i+=2,++j)
    {
        if(torqPerc[i]>torqPerc[i+1])
        {
            dominantTorques[j]=torqPerc[i];
        }
        else
        {
            dominantTorques[j]=torqPerc[i+1];
        }
    }

}


string& SensingNode::getName(){
    return name;
}

