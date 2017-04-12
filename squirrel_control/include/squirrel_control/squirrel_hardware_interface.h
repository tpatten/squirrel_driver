//
// Created by Philipp Zech on 11.04.17.
//

#ifndef SQUIRREL_CONTROL_SQUIRREL_HARDWARE_INTERFACE_H
#define SQUIRREL_CONTROL_SQUIRREL_HARDWARE_INTERFACE_H

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <squirrel_control/arm_interface.h>
#include <squirrel_control/arm_hardware_interface.h>
#include <error/throwControlError.h>

namespace squirrel_control
{

    static const double NUM_ROBOT_JOINTS = 8;  //or just 6?

    class SquirrelHardwareInterface : public hardware_interface::RobotHW
    {
    private:

        // Node Handles
        ros::NodeHandle nh_; // no namespace

        // Timing
        ros::Duration control_period_;
        ros::Time last_sim_time_ros_;
        ros::Duration elapsed_time_;
        double loop_hz_;

        // Interfaces
        hardware_interface::JointStateInterface    js_interface_;
        hardware_interface::PositionJointInterface pj_interface_;

        // sub-hardware interfaces
        ArmInterfacePtr arm_hw_;

        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        ros::Timer non_realtime_loop_;

        // Subscriber
        ros::Subscriber sub_joint_state_;

    public:

        /**
         * \brief Constructor/Destructor
         */
        SquirrelHardwareInterface();
        ~SquirrelHardwareInterface();

        /**
         * \brief Checks if the state message from Baxter is out of date
         * \return true if expired
         */
        bool stateExpired();

        void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

        void update(const ros::TimerEvent& e);

    };

} // namespace

#endif //SQUIRREL_CONTROL_SQUIRREL_HARDWARE_INTERFACE_H
