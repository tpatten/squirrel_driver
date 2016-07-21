#include <ros/ros.h>
#include <iostream>

#include <uibk_robot_driver/base_controller.hpp>


using namespace std;

int main(int argc, char** args) {

    ros::init(argc, args, "base_move_test");
    ros::NodeHandle node;
    usleep(1e6);

    BaseController robotino(node, 20.0, 0.6, 1, 1);
    ros::Rate lRate(20.0);

    double target = robotino.getCurrentState();
    vector<double> current_pose = robotino.getCurrentPose();
    double cx = current_pose.at(0);
    double cy = current_pose.at(1);

    //cout <<"start pose "<<current_pose.at(0)<<endl<<current_pose.at(1)<<endl<<current_pose.at(2)<<endl;

    while (ros::ok){

        vector<double> current_pose = robotino.getCurrentPose();
        cout <<"current pose "<<current_pose.at(0)<<endl<<current_pose.at(1)<<endl<<current_pose.at(2)<<endl;
       // cout << cx+ 0.1 << endl << cy +0.1 << endl;
       robotino.move(target, cx+0.80, cy+0.80);
        lRate.sleep();


    }


}
