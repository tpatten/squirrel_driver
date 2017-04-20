//
// Created by Philipp Zech on 11.04.17.
//

#ifndef SQUIRREL_CONTROL_SQUIRREL_ARM_INTERFACE_H
#define SQUIRREL_CONTROL_SQUIRREL_ARM_INTERFACE_H

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>


namespace squirrel_control
{

    enum SquirrelControlMode {POSITION, VELOCITY, TORQUE};

    class ArmInterface
    {
    protected:

        // Node Handles
        ros::NodeHandle nh_; // no namespace

        // Number of joints we are using
        unsigned int n_dof_;

        std::vector<std::string> joint_names_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_effort_command_;
        std::vector<double> joint_velocity_command_;

        // Speed of hardware loop
        double loop_hz_;

        // Name of this arm
        std::string arm_name_;

        // Track current hardware interface mode we are in
        int* joint_mode_;

    public:

        /**
         * \brief Constructor/Descructor
         */
        ArmInterface(const std::string &arm_name, double loop_hz)
                : arm_name_(arm_name),
                  loop_hz_(loop_hz)
        {};

        ~ArmInterface()
        {};

        /**
         * \brief Initialize hardware interface
         * \return false if an error occurred during initialization
         */
        virtual bool init(
                hardware_interface::JointStateInterface&    js_interface,
                hardware_interface::VelocityJointInterface& vj_interface,
                hardware_interface::PositionJointInterface& pj_interface,
                hardware_interface::EffortJointInterface&   ej_interface,
                int* joint_mode,
                sensor_msgs::JointStateConstPtr state_msg
        )
        { return true; };


        virtual void read( sensor_msgs::JointStateConstPtr &state_msg )
        {};


        virtual void write(ros::Duration elapsed_time)
        {};


        virtual void robotDisabledCallback()
        {};

    };

    typedef boost::shared_ptr<squirrel_control::ArmInterface> ArmInterfacePtr;
    typedef boost::shared_ptr<const squirrel_control::ArmInterface> ArmInterfaceConstPtr;

} // namespace

#endif //SQUIRREL_CONTROL_SQUIRREL_ARM_INTERFACE_H
