//
// Created by Philipp Zech on 11.04.17.
//

#ifndef SQUIRREL_CONTROL_SQUIRREL_ARM_HARDWARE_INTERFACE_H
#define SQUIRREL_CONTROL_SQUIRREL_ARM_HARDWARE_INTERFACE_H

// ROS
#include <trajectory_msgs/JointTrajectory.h>

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

        // Messages to send
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
                hardware_interface::PositionJointInterface& pj_interface,
                int* joint_mode,
                sensor_msgs::JointStateConstPtr state_msg
        );

        /**
         * \brief Buffers joint state info from Baxter ROS topic
         * \param
         */
        void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

        /**
         * \brief Copy the joint state message into our hardware interface datastructures
         */
        void read( sensor_msgs::JointStateConstPtr &state_msg );

        /**
         * \brief Publish our hardware interface datastructures commands to Baxter hardware
         */
        void write(ros::Duration elapsed_time);

        /**
         * \brief This is called when Baxter is disabled, so that we can update the desired positions
         */
        void robotDisabledCallback();

        /**
         * \brief inform the trajectory controller to update its setpoint
         */
        void publishCurrentLocation();
    };

} // namespace

#endif //SQUIRREL_CONTROL_SQUIRREL_ARM_HARDWARE_INTERFACE_H
