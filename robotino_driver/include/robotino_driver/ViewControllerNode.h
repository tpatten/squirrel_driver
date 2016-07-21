/**
 * ViewControllerNode.h
 *
 * Sets the view of the robot's camera.
 * This will typically use the pan and tilt joints, but could actually also use
 * the robot base if needed.
 * It will take view requests from clients and decide, also based on priority of
 * requests, where to move the camera.
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#ifndef VIEW_CONTROLLER_NODE_H_
#define VIEW_CONTROLLER_NODE_H_

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <dynamixel_msgs/JointState.h>
#include <robotino_msgs/LookAtImagePosition.h>
#include <robotino_msgs/LookAtPosition.h>
#include <robotino_msgs/FixatePosition.h>
#include <robotino_msgs/FixatePanTilt.h>
#include <robotino_msgs/ClearFixation.h>

class ViewControllerNode
{
public:
  ViewControllerNode();
  virtual ~ViewControllerNode();
  void init();

private:
  void movePanTilt(float pan, float tilt);
  bool lookAtImagePosition(robotino_msgs::LookAtImagePosition::Request &req,
                           robotino_msgs::LookAtImagePosition::Response &res);
  bool lookAtPosition(robotino_msgs::LookAtPosition::Request &req,
                      robotino_msgs::LookAtPosition::Response &res);
  bool fixatePosition(robotino_msgs::FixatePosition::Request &req,
                      robotino_msgs::FixatePosition::Response &res);
  bool fixatePanTilt(robotino_msgs::FixatePanTilt::Request &req,
                     robotino_msgs::FixatePanTilt::Response &res);
  bool clearFixation(robotino_msgs::ClearFixation::Request &req,
                      robotino_msgs::ClearFixation::Response &res);
  bool resetPosition( std_srvs::Empty::Request&, std_srvs::Empty::Response& );
  void panStateCallback(const dynamixel_msgs::JointState::ConstPtr& panStateMsg);
  void tiltStateCallback(const dynamixel_msgs::JointState::ConstPtr& tiltStateMsg);
  
  ros::NodeHandle private_nh_, public_nh_;
  ros::Publisher pan_pub_, tilt_pub_;
  ros::Subscriber pan_state_sub_, tilt_state_sub_;
  ros::ServiceServer look_image_srv_, look_srv_, fixate_srv_, fixate_pantilt_srv_, clear_srv_, reset_srv_;
  std::string pan_command_topic_, pan_status_topic_, tilt_command_topic_, tilt_status_topic_;
  boost::mutex joint_mutex_;

  float default_pan_, default_tilt_;
  float pan_, tilt_;
  std::string who_fixed_it;
};

#endif
