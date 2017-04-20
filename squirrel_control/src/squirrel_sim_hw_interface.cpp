//
// Created by Philipp Zech on 20.04.17.
//

#include "squirrel_control/squirrel_sim_hw_interface.h"

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace squirrel_control {
    SquirrelSimHWInterface::SquirrelSimHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
            : SquirrelHWInterface(nh, urdf_model)
            , name_("squirrel_sim_hw_interface") {
        // Load rosparams

        ros::NodeHandle rpnh(nh_, "squirrel_sim_hardware_interface");
        std::size_t error = 0;
        error += !rosparam_shortcuts::get(name_, rpnh, "sim_control_mode", sim_control_mode_);
        if (error)
        {
            ROS_WARN_STREAM_NAMED(name_, "SquirrelSimHWInterface now requires the following config in the yaml:");
            ROS_WARN_STREAM_NAMED(name_, "   sim_control_mode: 0 # 0: position, 1: velocity");
        }
        rosparam_shortcuts::shutdownIfError(name_, error);
    }


    SquirrelSimHWInterface::~SquirrelSimHWInterface() {

    }


    void SquirrelSimHWInterface::init() {
        // Call parent class version of this function
        SquirrelHWInterface::init();

        // Resize vectors
        joint_position_prev_.resize(num_joints_, 0.0);

        ROS_INFO_NAMED(name_, "SquirrelSimHWInterface Ready.");
    }


    void SquirrelSimHWInterface::read(ros::Duration &elapsed_time)
    {
        // No need to read since our write() command populates our state for us
    }


    void SquirrelSimHWInterface::write(ros::Duration &elapsed_time)
    {
        // Safety
        enforceLimits(elapsed_time);

        // NOTE: the following is a "simulation" example so that this boilerplate can be run without being
        // connected to hardware
        // When converting to your robot, remove the built-in PID loop and instead let the higher level
        // ros_control controllers take
        // care of PID loops for you. This P-controller is only intended to mimic the delay in real
        // hardware, somewhat like a simulator
        for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        {
            switch (sim_control_mode_)
            {
                case 0:  // hardware_interface::MODE_POSITION:
                    positionControlSimulation(elapsed_time, joint_id);
                    break;

                case 1:  // hardware_interface::MODE_VELOCITY:

                    // // Move all the states to the commanded set points slowly
                    // joint_position_[joint_id] += joint_velocity_[joint_id] * elapsed_time.toSec();

                    // v_error_ = joint_velocity_command_[joint_id] - joint_velocity_[joint_id];

                    // // scale the rate it takes to achieve velocity by a factor that is invariant to the feedback loop
                    // joint_velocity_[joint_id] += v_error_ * VELOCITY_STEP_FACTOR;

                    // Naive
                    joint_velocity_[joint_id] = joint_velocity_command_[joint_id];
                    joint_position_[joint_id] += joint_velocity_command_[joint_id] * elapsed_time.toSec();

                    break;

                case 2:  // hardware_interface::MODE_EFFORT:
                    ROS_ERROR_STREAM_NAMED(name_, "Effort not implemented yet");
                    break;
            }
        }
    }


    void SquirrelSimHWInterface::enforceLimits(ros::Duration &period)
    {
        // Enforces position and velocity
        pos_jnt_sat_interface_.enforceLimits(period);
    }


    void SquirrelSimHWInterface::positionControlSimulation(ros::Duration &elapsed_time, const std::size_t joint_id)
    {
        const double max_delta_pos = joint_velocity_limits_[joint_id] * elapsed_time.toSec();

        // Move all the states to the commanded set points at max velocity
        p_error_ = joint_position_command_[joint_id] - joint_position_[joint_id];

        const double delta_pos = std::max(std::min(p_error_, max_delta_pos), -max_delta_pos);
        joint_position_[joint_id] += delta_pos;

        // Bypass max velocity p controller:
        //joint_position_[joint_id] = joint_position_command_[joint_id];

        // Calculate velocity based on change in positions
        if (elapsed_time.toSec() > 0)
        {
            joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
        }
        else
            joint_velocity_[joint_id] = 0;

        // Save last position
        joint_position_prev_[joint_id] = joint_position_[joint_id];
    }
}
