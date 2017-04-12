//
// Created by Philipp Zech on 11.04.17.
//

#include <squirrel_control/squirrel_hardware_interface.h>

namespace squirrel_control {


}

int main(int argc, char** argv)
{
    ROS_INFO_STREAM_NAMED("hardware_interface","Starting hardware interface...");

    ros::init(argc, argv, "squirrel_hardware_interface");

    // Allow the action server to recieve and send ros messages
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    squirrel_control::SquirrelHardwareInterface robot();

    ros::spin();

    ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

    return 0;
}