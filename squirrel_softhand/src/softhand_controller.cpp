#include "ros/ros.h"
#include "squirrel_manipulation_msgs/SoftHandGrasp.h"
#include "stdlib.h"
#include "math.h"
#include "../include/qbmove_communications.h"

struct global_args {
    int device_id;
    comm_settings comm_settings_t;
} global_args;

bool actuate(squirrel_manipulation_msgs::SoftHandGrasp::Request & req,
             squirrel_manipulation_msgs::SoftHandGrasp::Response & res)
{
    short int inputs[2];
    inputs[0] = ceil(req.position*17000.0);
    ROS_INFO("Position: %d", inputs[0]);
    inputs[1] = 0;    
    commSetInputs(&global_args.comm_settings_t, global_args.device_id, inputs);
    if(global_args.comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        ROS_ERROR("Couldn't move hand!");
        res.success = false;
        return EXIT_FAILURE;
    }
    res.success = true;
    return true;    
}

bool do_calibration()
{
    // CONFIGURE LIMITS
    int limits[4];
    limits[0] = 0;
    limits[1] = 17000;
    limits[2] = 0;
    limits[3] = 0;
    int speed = 200;
    int repetitions = 15;
    commSetParam(&global_args.comm_settings_t, global_args.device_id, PARAM_POS_LIMIT, limits, 4);
    commCalibrate(&global_args.comm_settings_t, global_args.device_id);
    commHandCalibrate(&global_args.comm_settings_t, global_args.device_id, speed, repetitions);
    ROS_WARN("Not yet implemented!");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "squirrel_softhand");
    ros::NodeHandle n;

    char ports[10][255];
    RS485listPorts(ports);
    std::string port;
    ros::param::param<std::string>("port", port, ports[0]);
    ROS_INFO("Selected port: %s", port.c_str());

    ros::param::param<int>("device_id", global_args.device_id, 1);
    ROS_INFO("Device ID: %d", global_args.device_id);

    // CONNECT
    openRS485(&global_args.comm_settings_t, port.c_str());
    if(global_args.comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        ROS_ERROR("Couldn't connect to serial port: %p", port.c_str());
        return EXIT_FAILURE;
    } else {
        ROS_INFO("Connected to %s.", port.c_str());
    }    
    commActivate(&global_args.comm_settings_t, global_args.device_id, 1);    
    ros::ServiceServer service = n.advertiseService("softhand_grasp", actuate);
    ROS_INFO("Ready to grasp.");

    // CALIBRATE
    bool calibrate;
    ros::param::param<bool>("calibrate", calibrate, true);
    if(calibrate)
    {
        ROS_INFO("Performing autocalibration.");
        if(do_calibration())
        {
            ROS_INFO("Autocalibration done.");
        } else
        {
            ROS_ERROR("Autocalibration failed!");
            return EXIT_FAILURE;
        }        
    } else {
        ROS_INFO("Skipping autocalibration.");
    }

    // SERVICE
    while(ros::ok()){
        ros::spinOnce();
    }

    // SHUTDOWN
    commActivate(&global_args.comm_settings_t, global_args.device_id, 0);
    closeRS485(&global_args.comm_settings_t);

    return EXIT_SUCCESS;
}
