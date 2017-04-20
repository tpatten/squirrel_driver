//
// Created by Philipp Zech on 11.04.17.
//

#ifndef SQUIRREL_CONTROL_SQUIRREL_ARM_HARDWARE_INTERFACE_H
#define SQUIRREL_CONTROL_SQUIRREL_ARM_HARDWARE_INTERFACE_H

// ROS
#include <trajectory_msgs/JointTrajectory.h>

#include <squirrel_control_msgs/JointCommand.h>
#include <squirrel_safety_msgs/Safety.h>

// Parent class
#include <squirrel_control/arm_interface.h>

namespace squirrel_control
{

    static const double STATE_EXPIRED_TIMEOUT = 2.0;

    class ArmHardwareInterface : public ArmInterface
    {
    private:

        // Publishers
        ros::Publisher pub_joint_command_;
        ros::Publisher pub_trajectory_command_;

        // Subscriber
        ros::Subscriber e_squeezed_sub_;

        // Track button status
        bool e_squeezed_previous;

        // Convert a joint states message to our ids
        std::vector<int> joint_id_to_joint_states_id_;

        // Messages to send
        squirrel_control_msgs::JointCommand output_msg_;
        trajectory_msgs::JointTrajectory trajectory_command_msg_;

    public:

        /**
         * \brief Constructor/Destructor
         */
        ArmHardwareInterface(const std::string &arm_name, double loop_hz);
        ~ArmHardwareInterface();

        /**
         * \brief Initialize hardware interface
         * \return false if an error occurred during initialization
         */
        bool init(
                hardware_interface::JointStateInterface&    js_interface,
                hardware_interface::VelocityJointInterface& vj_interface,
                hardware_interface::PositionJointInterface& pj_interface,
                hardware_interface::EffortJointInterface&   ej_interface,
                int* joint_mode,
                sensor_msgs::JointStateConstPtr state_msg
        );


        void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

        void eSqueezedCallback(const squirrel_safety_msgs::SafetyConstPtr& msg);

        void read(sensor_msgs::JointStateConstPtr &state_msg );

        void write(ros::Duration elapsed_time);

        void robotDisabledCallback();

        void publishCurrentLocation();
    };

} // namespace

#endif //SQUIRREL_CONTROL_SQUIRREL_ARM_HARDWARE_INTERFACE_H
