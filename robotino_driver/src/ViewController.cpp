/**
 * ViewController.cpp
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


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_view_controller_msgs/FixateOnPoseAction.h>
#include "std_msgs/Float64.h"
#include <dynamixel_controllers/SetSpeed.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include "ViewController.h"
/*
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
*/
  FixateOnPoseAction::~FixateOnPoseAction(void)
  {
  }

  FixateOnPoseAction::FixateOnPoseAction(std::string name) :
    as_(nh_, name, boost::bind(&FixateOnPoseAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    pan_speed_client_ =
      nh_.serviceClient<dynamixel_controllers::SetSpeed>("/pan_controller/set_speed", true);
    tilt_speed_client_ =
      nh_.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_controller/set_speed", true);
    pan_pub_ = nh_.advertise<std_msgs::Float64>("/pan_controller/relative_command", 0, false);
    tilt_pub_ = nh_.advertise<std_msgs::Float64>("/tilt_controller/relative_command", 0, false);

  }

  void FixateOnPoseAction::executeCB(const squirrel_view_controller_msgs::FixateOnPoseGoalConstPtr &goal)
  {
  // helper variables
    ros::Rate r(25);
    bool success = true;

  if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      return;
    }
    
  point.header.stamp = ros::Time::now();
  point.header.frame_id = goal->pose.header.frame_id;
  point.point.x = goal->pose.pose.position.x;
  point.point.y = goal->pose.pose.position.y;
  point.point.z = goal->pose.pose.position.z;

  try
    {
      ros::Time now = ros::Time(0);
      listener.waitForTransform("/pan_link", point.header.frame_id, now, ros::Duration(3.0));
      listener.waitForTransform("/tilt_link", point.header.frame_id, now, ros::Duration(3.0));
      listener.transformPoint("/pan_link", now, point, point.header.frame_id, pan);
      listener.transformPoint("/tilt_link", now, point , point.header.frame_id, tilt);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    panMsg.data = atan2(pan.point.y, pan.point.x);
    tiltMsg.data = -atan2(tilt.point.y, tilt.point.x);

    if (fabs(panMsg.data) > 0.001)
    {
      pan_pub_.publish(panMsg);
    }
    else
    {
      ROS_DEBUG("Relative pan angle: %f (rad)", panMsg.data);
    }
    if (fabs(tiltMsg.data) > 0.001)
    {
      tilt_pub_.publish(tiltMsg);
    }
    else
    {
      ROS_DEBUG("Relative tilt angle: %f (rad)", tiltMsg.data);
    } 
  
  r.sleep();
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "squirrel_view_controller");

  FixateOnPoseAction fixate("squirrel_view_controller");
  ros::spin();

  return 0;
}
