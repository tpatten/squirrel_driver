#include <ros/ros.h>
#include <iostream>
#include <uibk_robot_driver/base_controller.hpp>
#define ROBOTINO_ODOM_TOPIC "/odom"
#define ROBOTINO_MOVE_TOPIC "/cmd_vel"

using namespace std;
const float PI = 3.1416;
class Listener {
public:

nav_msgs::Odometry odom1;
void callback(nav_msgs::Odometry msg){
odom1 =msg;
}
};

 Listener listener;

// float abs(float x){

//     return x>0 ? x : -1*x;
// }

int main(int argc, char** args) {

    ros::init(argc, args, "base_move_test_by_Qusai");
    ros::NodeHandle node;
    ros::Publisher pubMove;
    geometry_msgs::Twist vel;
    pubMove = node.advertise<geometry_msgs::Twist>(ROBOTINO_MOVE_TOPIC, 1);


    ros::Subscriber subOdometry = node.subscribe(ROBOTINO_ODOM_TOPIC, 1, &Listener::callback, &listener);
    ros::Rate lRate(20);


    vel.linear.x=0;
    vel.linear.y=0;
    vel.linear.z=0;
    vel.angular.x=0;
    vel.angular.y=0;
    vel.angular.z=0;
enum State {state1,state2,state3,state4};
State currentState;
const float kp=0.8;
    currentState=state1;
    float error ;
    while (ros::ok){

//        vector<double> current_pose = robotino.getCurrentPose();
//        // cout <<"current pose "<<current_pose.at(0)<<endl<<current_pose.at(1)<<endl<<current_pose.at(2)<<endl;

//       // cout<<"current state "<<robotino.getCurrentPose()<<endl;
//        target = target - 0.05;
//       // cx = cx - 0.01;
//       // cy = cy - 0.01;
        ros::spinOnce();

        switch(currentState)
        {
        case state1:
             error = 0.3-listener.odom1.pose.pose.position.x ;
            vel.linear.x= kp*error;
            if (abs(error) < 0.01){
                currentState=state2;
                vel.linear.x=0;
            }
        break;
        case state2:
             error = 0.3-listener.odom1.pose.pose.position.y ;
            vel.linear.y= kp*error;
            if (abs(error) < 0.01){
                currentState=state3;
                vel.linear.y=0;
            }
        break;
        case state3:
             error = -0.3-listener.odom1.pose.pose.position.x ;
            vel.linear.x= kp*error;
            if (abs(error) < 0.01){
                currentState=state4;
                vel.linear.x=0;
            }
        break;
        case state4:
             error = -0.3-listener.odom1.pose.pose.position.y ;
            vel.linear.y= kp*error;
            if (abs(error) < 0.01){
                currentState=state1;
                vel.linear.y=0;
            }
        break;

        }

        cout << "my x y coordinates are : " << listener.odom1.pose.pose.position.x << "  " << listener.odom1.pose.pose.position.y << endl;
        pubMove.publish(vel);
        lRate.sleep();


    }


}
