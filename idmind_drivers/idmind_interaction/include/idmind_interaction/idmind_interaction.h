#ifndef IDMIND_INTERACTION_H
#define IDMIND_INTERACTION_H

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "idmind_serial/idmind_serial.h"
//#include <std_msgs/UInt8.h>
#include <std_msgs/Int64.h>
#include "idmind_interaction/MotorControl.h"
#include "idmind_interaction/LedControl.h"
#include "idmind_interaction/LedControlBase.h"
#include "idmind_interaction/LedControlMouth.h"

class IdmindInteraction
{
public:
    IdmindInteraction();

    void initialize();
    void runPeriodically();
    void shutdown();

    bool green_light;

private:

    void headCallback(const idmind_interaction::MotorControl::ConstPtr& msg);
    void sendHeadCommand(bool init = false);

    void neckCallback(const idmind_interaction::MotorControl::ConstPtr& msg);
    void sendNeckCommand(bool init = false);

    void headNeckCallback(const idmind_interaction::MotorControl::ConstPtr& msg);
    void sendHeadNeckCommand();

    void cameraTiltCallback(const idmind_interaction::MotorControl::ConstPtr& msg);
    void sendCameraTiltCommand();

    void ledsCallback(const idmind_interaction::LedControl::ConstPtr& msg);
    void sendLedsCommand();

    void ledsBaseCallback(const idmind_interaction::LedControlBase::ConstPtr& msg);
    void sendLedsBaseCommand();

    void ledsMouthCallback(const idmind_interaction::LedControlMouth::ConstPtr& msg);
    void sendLedsMouthCommand();

    bool setNumberBaseLeds();

    void getMotorPosition(bool publish_topics = false);
    int sign(int num);

    ros::NodeHandle n_;
    ros::Subscriber head_sub_,neck_sub_, head_neck_sub_,camera_sub_;
    ros::Subscriber leds_sub_,leds_base_sub_,leds_mouth_sub_;
    ros::Publisher head_position_pub_,neck_position_pub_,camera_position_pub_;
    IdmindSerial serial_;
    std::string idmind_ns_;

    int motor_head_,motor_neck_,motor_camera_;

    int set_position_command_,set_velocity_command_,get_position_command_;
    int set_base_leds_command_,set_mouth_leds_command_;
    int set_num_base_leds_command_;
    uint8_t head_[2];
    uint8_t neck_[2];
    uint8_t cam_tilt_[2];

    int D_head,D_neck,D_cam;
    int head_position_,neck_position_,camera_position_;
    int dist_inc_;
    int vel_head, vel_neck, vel_cam;
    uint8_t last_head_vel_,last_neck_vel_,last_cam_vel_;

    int final_pos_;
    float ref_time_;
    float sleep_time_;

    bool send_head_,send_neck_,send_headneck_,send_camera_;

    uint8_t led_device_;
    uint8_t led_rgb_[3];
    int number_of_leds_; //4 in case of head, 42 in case of base
    const int BASE_LEDS,MOUTH_LEDS;
    uint8_t array_leds_mouth_[12];
    uint8_t array_leds_base_[126];
    bool send_leds_,send_leds_base_,send_leds_mouth_;

    bool initialize_;

};

#endif