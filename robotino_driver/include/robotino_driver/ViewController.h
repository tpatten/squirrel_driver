/**
 * ViewController.h
 *
 * Sets the view of the robot's camera. This will typically use the pan and
 * tilt joints, but could actually also use the robot base if needed.
 * It will take view requests from clients and decide, also based on THEORY
 * priority of requests, where to move the camera.
 * 
 * @author Markus 'Bajo' Bajones bajones@acin.tuwien.ac.at
 * @date Feb 2017
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef VIEW_CONTROLLER_H
#define VIEW_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_view_controller_msgs/FixateOnPoseAction.h>
#include "std_msgs/Float64.h"
#include <dynamixel_controllers/SetSpeed.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

class FixateOnPoseAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer <squirrel_view_controller_msgs::FixateOnPoseAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  squirrel_view_controller_msgs::FixateOnPoseFeedback feedback_;
  squirrel_view_controller_msgs::FixateOnPoseResult result_;

public:

  geometry_msgs::PointStamped point, pan, tilt;
  std_msgs::Float64 panMsg, tiltMsg;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  
  ros::ServiceClient pan_speed_client_;
  ros::ServiceClient tilt_speed_client_;
  ros::Publisher pan_pub_;
  ros::Publisher tilt_pub_;
  
  void executeCB(const squirrel_view_controller_msgs::FixateOnPoseGoalConstPtr &goal);

  FixateOnPoseAction(std::string name);
  ~FixateOnPoseAction();
};
#endif // VIEW_CONTROLLER_H
