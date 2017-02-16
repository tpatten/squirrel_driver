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
#include <robotino_msgs/LookAtImagePosition.h>
#include <robotino_msgs/LookAtPosition.h>
#include <robotino_msgs/FixatePosition.h>
#include <robotino_msgs/FixatePanTilt.h>
#include <robotino_msgs/ClearFixation.h>

class ViewController
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<squirrel_view_controller_msgs::FixateOnPoseAction> as_;  // NodeHandle instance must be
                                                                                         // created before this line.
                                                                                         // Otherwise strange error
                                                                                         // occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  squirrel_view_controller_msgs::FixateOnPoseFeedback feedback_;
  squirrel_view_controller_msgs::FixateOnPoseResult result_;

  ros::Publisher pan_pub_, tilt_pub_;
  ros::Subscriber pan_state_sub_, tilt_state_sub_;
  ros::ServiceServer look_image_srv_, look_srv_, fixate_srv_, fixate_pantilt_srv_, clear_srv_, reset_srv_;
  std::string pan_command_topic_, pan_status_topic_, tilt_command_topic_, tilt_status_topic_;
  boost::mutex joint_mutex_;

  geometry_msgs::PointStamped point, pan, tilt;
  std_msgs::Float64 panMsg, tiltMsg;
  tf::StampedTransform transform;
  tf::TransformListener listener;

  ros::ServiceClient pan_speed_client_;
  ros::ServiceClient tilt_speed_client_;

  float default_pan_, default_tilt_;
  float pan_, tilt_;
  std::string who_fixed_it;

  void movePanTilt(float pan, float tilt);
  bool lookAtImagePosition(robotino_msgs::LookAtImagePosition::Request &req,
                           robotino_msgs::LookAtImagePosition::Response &res);
  bool lookAtPosition(robotino_msgs::LookAtPosition::Request &req, robotino_msgs::LookAtPosition::Response &res);
  bool fixatePosition(robotino_msgs::FixatePosition::Request &req, robotino_msgs::FixatePosition::Response &res);
  bool fixatePanTilt(robotino_msgs::FixatePanTilt::Request &req, robotino_msgs::FixatePanTilt::Response &res);
  bool clearFixation(robotino_msgs::ClearFixation::Request &req, robotino_msgs::ClearFixation::Response &res);
  bool resetPosition(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
  void panStateCallback(const dynamixel_msgs::JointState::ConstPtr &panStateMsg);
  void tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr &tiltStateMsg);

public:
  void executeCB(const squirrel_view_controller_msgs::FixateOnPoseGoalConstPtr &goal);

  ViewController(std::string name);
  ~ViewController();
};
#endif  // VIEW_CONTROLLER_H
