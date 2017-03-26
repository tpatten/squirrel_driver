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
#include <dynamixel_controllers/SetRelativePosition.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "ViewController.h"

ViewController::~ViewController(void)
{
  nh_.shutdown();
}

ViewController::ViewController(std::string name)
  : as_(nh_, name, boost::bind(&ViewController::executeCB, this, _1), false), action_name_(name)
{
  as_.start();

  bool have_all = true;
  if (!ros::topic::waitForMessage<dynamixel_msgs::JointState>(pan_status_topic_, ros::Duration(30.0)))
  {
    ROS_WARN("pan controller not running, shutting down the node");
    have_all = false;
  }
  if (!ros::topic::waitForMessage<dynamixel_msgs::JointState>(tilt_status_topic_, ros::Duration(30.0)))
  {
    ROS_WARN("tilt controller not running, shutting down the node");
    have_all = false;
  }

  if (have_all)
  {
    boost::mutex::scoped_lock lock(joint_mutex_);
    nh_.param<float>("default_pan_angle", default_pan_ , 0.0);
    nh_.param<float>("default_tilt_angle", default_tilt_ , 0.6);

    // connect to services
    pan_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/pan_controller/set_speed", true);
    tilt_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_controller/set_speed", true);
    rel_tilt_client_ = nh_.serviceClient<dynamixel_controllers::SetRelativePosition>("/tilt_controller/set_relative_position", true);
    rel_pan_client_ = nh_.serviceClient<dynamixel_controllers::SetRelativePosition>("/pan_controller/set_relative_position", true);

    // setup publishers
    pan_pub_ = nh_.advertise<std_msgs::Float64>("/pan_controller/command", 0, false);
    tilt_pub_ = nh_.advertise<std_msgs::Float64>("/tilt_controller/command", 0, false);
    rel_pan_pub_ = nh_.advertise<std_msgs::Float64>("/pan_controller/relative_command", 0, false);
    rel_tilt_pub_ = nh_.advertise<std_msgs::Float64>("/tilt_controller/relative_command", 0, false);
    vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    // advertise services
    look_image_srv_ = nh_.advertiseService("/squirrel_view_controller/look_at_image_position", &ViewController::lookAtImagePosition, this);
    look_srv_ = nh_.advertiseService("/squirrel_view_controller/look_at_position", &ViewController::lookAtPosition, this);
    fixate_pantilt_srv_ =  nh_.advertiseService("/squirrel_view_controller/fixate_pantilt", &ViewController::fixatePanTilt, this);
    clear_srv_ = nh_.advertiseService("/squirrel_view_controller/clear_fixation", &ViewController::clearFixation, this);
    reset_srv_ = nh_.advertiseService("/squirrel_view_controller/reset", &ViewController::resetPosition, this);

    ROS_INFO("moving to default pan/tilt position: %f / %f [rad])", default_pan_, default_tilt_);
    movePanTilt(default_pan_, default_tilt_);
    ROS_INFO("ViewController ready...");
  }
  else
  {
    nh_.shutdown();
  }
}

std::vector<double> ViewController::pose2PanTilt(geometry_msgs::PoseStamped pose)
{
   std::vector<double> v;
   geometry_msgs::PointStamped point, pan, tilt;
   
   point.header.stamp = ros::Time::now();
   point.header.frame_id = pose.header.frame_id;
   point.point.x = pose.pose.position.x;
   point.point.y = pose.pose.position.y;
   point.point.z = pose.pose.position.z;

  try
  {
    ros::Time now = ros::Time(0);
    listener_.waitForTransform("/pan_link", point.header.frame_id, now, ros::Duration(3.0));
    listener_.waitForTransform("/tilt_link", point.header.frame_id, now, ros::Duration(3.0));
    listener_.transformPoint("/pan_link", now, point, point.header.frame_id, pan);
    listener_.transformPoint("/tilt_link", now, point, point.header.frame_id, tilt);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  float rel_pan = atan2(pan.point.y,pan.point.x);
  v.push_back(rel_pan);
  float rel_tilt = -atan2(tilt.point.y,tilt.point.x);
  if (rel_tilt < -M_PI/2)
  {
    rel_tilt += 0.0;
  }
  v.push_back(rel_tilt);

  ROS_DEBUG("Pan: x: %f, y: %f", pan.point.x, pan.point.y);
  ROS_DEBUG("Tilt: x: %f, y: %f", tilt.point.x, tilt.point.y);
  ROS_DEBUG("Moving camera relative to pan: %f, tilt: %f ", rel_pan, rel_tilt);

  return v;
}

void ViewController::executeCB(const squirrel_view_controller_msgs::FixateOnPoseGoalConstPtr &goal)
{
  // helper variables
  ros::Rate r(10);
  bool success = true;
  std::vector<double> v;
  std_msgs::Float64 msg;

  ROS_INFO("Received goal. %s: x: %f ,y: %f", goal->pose.header.frame_id.c_str(), goal->pose.pose.position.x, goal->pose.pose.position.y);
  while (true)
  {

    if (as_.isPreemptRequested() || !ros::ok() || !goal->enable)
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      return;
    }

    v = pose2PanTilt(goal->pose);
    publishPoseMarker(goal->pose);
    //moveRelativePanTilt(v[0], v[1]);
    //r.sleep();
    msg.data = v[0];
    if (std::isfinite(msg.data))
    {
      if (fabs(msg.data) > 0.001)
      {
        rel_pan_pub_.publish(msg);
        r.sleep();
      }
      else
        ROS_DEBUG("Relative pan angle: %f (rad)", msg.data);
    }
    v = pose2PanTilt(goal->pose);
    msg.data = v[1];
    if (std::isfinite(msg.data))
    {
      if (fabs(msg.data) > 0.001)
      {
        rel_tilt_pub_.publish(msg);
        r.sleep();
      }
      else
        ROS_DEBUG("Relative tilt angle: %f (rad)", msg.data);
    }
  }
}


void ViewController::publishPoseMarker(geometry_msgs::PoseStamped pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = pose.header.stamp;
    marker.ns = "fixation point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.pose.position.x;
    marker.pose.position.y = pose.pose.position.y;
    marker.pose.position.z = pose.pose.position.z;
    marker.pose.orientation.x = pose.pose.orientation.x;
    marker.pose.orientation.y = pose.pose.orientation.y;
    marker.pose.orientation.z = pose.pose.orientation.z;
    marker.pose.orientation.w = pose.pose.orientation.w;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0; 
    vis_pub_.publish(marker);

}

void ViewController::movePanTilt(float pan, float tilt)
{
  std_msgs::Float64 panMsg, tiltMsg;
  panMsg.data = pan;
  tiltMsg.data = tilt;
  if (std::isfinite(panMsg.data) && std::isfinite(tiltMsg.data))
  {
    pan_pub_.publish(panMsg);
    tilt_pub_.publish(tiltMsg);
  }
}

void ViewController::moveRelativePanTilt(float pan, float tilt)
{
  std_msgs::Float64 panMsg, tiltMsg;
  panMsg.data = pan;
  tiltMsg.data = tilt;
  if (std::isfinite(panMsg.data) && std::isfinite(tiltMsg.data))
  {
    if (fabs(panMsg.data) > 0.001)
      rel_pan_pub_.publish(panMsg);
    else
      ROS_DEBUG("Relative pan angle: %f (rad)", panMsg.data);
    
    if (fabs(tiltMsg.data) > 0.001)
      rel_tilt_pub_.publish(tiltMsg);
    else
      ROS_DEBUG("Relative tilt angle: %f (rad)", tiltMsg.data);
  }
  else
  {
    ROS_DEBUG("Infinity check failed");
  }
  return;
}

void ViewController::panStateCallback(const dynamixel_msgs::JointState::ConstPtr &panStateMsg)
{
  boost::mutex::scoped_lock lock(joint_mutex_);
  pan_ = panStateMsg->current_pos;
}

void ViewController::tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr &tiltStateMsg)
{
  boost::mutex::scoped_lock lock(joint_mutex_);
  tilt_ = tiltStateMsg->current_pos;
}

bool ViewController::resetPosition(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock lock(joint_mutex_);
  ROS_INFO("moving to default pan/tilt position: %f / %f [rad])", default_pan_, default_tilt_);
  movePanTilt(default_pan_, default_tilt_);
  who_fixed_it = "";
  return true;
}

bool ViewController::fixatePanTilt(squirrel_view_controller_msgs::FixatePanTilt::Request &req,
                                   squirrel_view_controller_msgs::FixatePanTilt::Response &res)
{
  if (who_fixed_it.empty())
  {
    boost::mutex::scoped_lock lock(joint_mutex_);
    movePanTilt(req.pan, req.tilt);
    if (!req.reason.empty())
      who_fixed_it = req.reason;
    else
      who_fixed_it = "*";
    return true;
  }
  else
  {
    ROS_INFO("view is already fixed by '%s'", who_fixed_it.c_str());
    return false;
  }
}

bool ViewController::clearFixation(squirrel_view_controller_msgs::ClearFixation::Request &req,
                                   squirrel_view_controller_msgs::ClearFixation::Response &res)
{
  if (!who_fixed_it.empty())
  {
    boost::mutex::scoped_lock lock(joint_mutex_);
    std::string reason = req.reason;
    if (reason.empty())
      reason = "*";
    if (who_fixed_it == req.reason)
    {
      who_fixed_it = "";
      return true;
    }
    else
    {
      ROS_INFO("view was fixed by '%s', but you are '%s', ignoring", who_fixed_it.c_str(), req.reason.c_str());
      return false;
    }
  }
  else
  {
    ROS_INFO("view was not fixed, ignoring");
    return false;
  }
}

bool ViewController::lookAtImagePosition(squirrel_view_controller_msgs::LookAtImagePosition::Request &req,
                                             squirrel_view_controller_msgs::LookAtImagePosition::Response &res)
{
  if(who_fixed_it.empty())
  {
    boost::mutex::scoped_lock lock(joint_mutex_);
    // HACK: the focal length is hardcoded for the Kinect/Asus
    movePanTilt(pan_ - atan2(req.x, 525), tilt_ + atan2(req.y, 525));
    //ROS_INFO("pan/tilt relative move move (deg): %.f %.f", -atan2(req.x, 525)*180./M_PI, atan2(req.y, 525*180./M_PI));
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewController::callServoService(ros::ServiceClient *client, dynamixel_controllers::SetRelativePosition srv)
{
  ROS_INFO("service data is: %f", srv.request.position);
  if (!(ros::service::waitForService(client->getService(), ros::Duration(5.0))))
            return false;
  if (client->call(srv))
  {
    ROS_INFO("goal reached: %d", srv.response.goal_reached);
    ROS_INFO("min_max_limit reached: %d", srv.response.min_max_limit_reached);
    if (srv.response.goal_reached)
      return true;
  }
  else
  {
    ROS_ERROR("Failed to call service %s", client->getService().c_str());
    return false;
  }
}

bool ViewController::lookAtPosition(squirrel_view_controller_msgs::LookAtPosition::Request &req,
                                    squirrel_view_controller_msgs::LookAtPosition::Response &res)
{
  std::vector<double> v;
  ROS_INFO("ViewController: lookAtPosition: %s, x: %f, y: %f", req.target.header.frame_id.c_str(), req.target.pose.position.x, req.target.pose.position.y);
  v = pose2PanTilt(req.target);
  // commented out when using services
  //moveRelativePanTilt(v[0], v[1]);
  dynamixel_controllers::SetRelativePosition srv;
  srv.request.position = v[0];
  bool pan_success = callServoService(&rel_pan_client_, srv);

  v = pose2PanTilt(req.target);
  srv.request.position = v[1];
  bool tilt_success = callServoService(&rel_tilt_client_, srv);
  return pan_success && tilt_success;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "squirrel_view_controller");

  ViewController fixate(ros::this_node::getName());
  ros::spin();

  return 0;
}
