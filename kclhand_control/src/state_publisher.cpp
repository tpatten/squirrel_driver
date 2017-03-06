#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

using namespace std;

sensor_msgs::JointState joint_value;
sensor_msgs::JointState joint_state;

void post_joint_valueCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
        joint_value = *msg;
        joint_state.name.resize(10);
        joint_state.position.resize(10);
   
        for(int i = 0; i<=9; i++)
        {
        joint_state.name[i] = joint_value.name[i];
        joint_state.position[i] = joint_value.position[i];

        }

   

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber sub = n.subscribe("post_joint_value", 100, post_joint_valueCallback);

    ros::Rate loop_rate(300);

    const double degree = M_PI/180;

    double left_crank_base_joint = 0, right_crank_base_joint = 0;
    string joint_name[10] = {"left_crank_base_joint", "right_crank_base_joint", 
                              "left_coupler_crank_joint", "right_coupler_crank_joint",
                               "left_finger_lower_joint", "left_finger_upper_joint" ,
                               "middle_finger_lower_joint", "middle_finger_upper_joint",
                               "right_finger_lower_joint", "right_finger_upper_joint" };

    float kkk=0;
    //ros::spin();

    while (ros::ok()) {
        //update joint_state
        
        joint_state.header.stamp = ros::Time::now();

       
        joint_pub.publish(joint_state);

        
        kkk = kkk+ 0.01;
        // Create new robot state
        ros::spinOnce();

    }


    return 0;
}