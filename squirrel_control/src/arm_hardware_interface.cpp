//
// Created by Philipp Zech on 11.04.17.
//


#include <squirrel_control/arm_hardware_interface.h>

namespace squirrel_control
{

    ArmHardwareInterface::ArmHardwareInterface(const std::string &arm_name, double loop_hz) :
            ArmInterface(arm_name, loop_hz)
    {
        // Populate joints in this arm
        joint_names_.push_back(arm_name_+"_j0");
        joint_names_.push_back(arm_name_+"_j1");
        joint_names_.push_back(arm_name_+"_j2");
        joint_names_.push_back(arm_name_+"_j3");
        joint_names_.push_back(arm_name_+"_j4");
        n_dof_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(n_dof_);
        joint_velocity_.resize(n_dof_);
        joint_effort_.resize(n_dof_);
        joint_position_command_.resize(n_dof_);
        joint_effort_command_.resize(n_dof_);
        joint_velocity_command_.resize(n_dof_);
        output_msg_.command.resize(n_dof_);
        output_msg_.names.resize(n_dof_);
        trajectory_command_msg_.joint_names.resize(n_dof_);
        joint_id_to_joint_states_id_.resize(n_dof_);

        for (std::size_t i = 0; i < n_dof_; ++i)
        {
            joint_position_[i] = 0.0;
            joint_velocity_[i] = 0.0;
            joint_effort_[i] = 0.0;
            joint_position_command_[i] = 0.0;
            joint_effort_command_[i] = 0.0;
            joint_velocity_command_[i] = 0.0;
            trajectory_command_msg_.joint_names[i] = joint_names_[i];
        }

        // Set trajectory to have two point
        trajectory_msgs::JointTrajectoryPoint single_pt;
        single_pt.positions.resize(n_dof_);
        single_pt.time_from_start = ros::Duration(0);
        trajectory_command_msg_.points.push_back(single_pt);

        trajectory_msgs::JointTrajectoryPoint single_pt2;
        single_pt2.positions.resize(n_dof_);
        single_pt2.time_from_start = ros::Duration(0.5);
        trajectory_command_msg_.points.push_back(single_pt2);
    }

   bool ArmHardwareInterface::init(hardware_interface::JointStateInterface &js_interface,
                                   hardware_interface::VelocityJointInterface& vj_interface,
                                   hardware_interface::PositionJointInterface &pj_interface,
                                   hardware_interface::EffortJointInterface &ej_interface, int *joint_mode,
                                   sensor_msgs::JointStateConstPtr state_msg) {
       joint_mode_ = joint_mode;

       for (std::size_t i = 0; i < n_dof_; ++i)
       {
           // Create joint state interface for all joints
           js_interface.registerHandle(hardware_interface::JointStateHandle(
                   joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

           // Create position joint interface
           pj_interface.registerHandle(hardware_interface::JointHandle(
                   js_interface.getHandle(joint_names_[i]),&joint_position_command_[i]));

           // Create velocity joint interface
           vj_interface.registerHandle(hardware_interface::JointHandle(
                   js_interface.getHandle(joint_names_[i]),&joint_velocity_command_[i]));

           // Create effort joint interface
           ej_interface.registerHandle(hardware_interface::JointHandle(
                   js_interface.getHandle(joint_names_[i]),&joint_effort_command_[i]));
       }

       // Start publishers
       pub_joint_command_ = nh_.advertise<squirrel_control_msgs::JointCommand>(arm_name_+"/joint_command",10);

       pub_trajectory_command_ = nh_.advertise<trajectory_msgs::JointTrajectory>(arm_name_+"_trajectory_controller/command",10);


       // Make a mapping of joint names to indexes in the joint_states message
       for (std::size_t i = 0; i < n_dof_; ++i)
       {
           std::vector<std::string>::iterator iter = std::find(state_msg->name.begin(), state_msg->name.end(), joint_names_[i]);
           size_t joint_states_id = std::distance(state_msg->name.begin(), iter);
           if(joint_states_id == state_msg->name.size())
           {
               ROS_ERROR_STREAM_NAMED(arm_name_,"Unable to find joint " << i << " named " << joint_names_[i] << " in joint state message");
           }

           joint_id_to_joint_states_id_[i] = joint_states_id;

           ROS_DEBUG_STREAM_NAMED("arm_hardware_interface","Found joint " << i << " at " << joint_states_id << " named " << joint_names_[i]);
       }

       // Set the initial command values based on current state
       for (std::size_t i = 0; i < n_dof_; ++i)
       {
           joint_position_command_[i] = state_msg->position[joint_id_to_joint_states_id_[i]];

           // Pre-load the joint names into the output messages just once
           output_msg_.names[i] = joint_names_[i];
       }

       ROS_INFO_NAMED(arm_name_, "Loaded squirrel_hardware_interface.");
       return true;
   }


    void ArmHardwareInterface::read( sensor_msgs::JointStateConstPtr &state_msg ) {
        for (std::size_t i = 0; i < n_dof_; ++i)
        {
            ROS_INFO_STREAM_NAMED("arm_hardware_interface","Joint " << i << "("<< joint_names_[i] << ") -> " << joint_id_to_joint_states_id_[i] << " position= " << state_msg->position[joint_id_to_joint_states_id_[i]]);
            joint_position_[i] = state_msg->position[joint_id_to_joint_states_id_[i]];
            joint_velocity_[i] = state_msg->velocity[joint_id_to_joint_states_id_[i]];
            joint_effort_[i] = state_msg->effort[joint_id_to_joint_states_id_[i]];
        }
    }

    void ArmHardwareInterface::write(ros::Duration elapsed_time) {
        // Send commands to robot in different modes
        switch (*joint_mode_)
        {
            case hardware_interface::MODE_POSITION:
                output_msg_.command = joint_position_command_;
                output_msg_.mode = squirrel_control_msgs::JointCommand::POSITION_MODE;
                break;
            case hardware_interface::MODE_VELOCITY:
                output_msg_.command = joint_velocity_command_;
                output_msg_.mode = squirrel_control_msgs::JointCommand::VELOCITY_MODE;
                break;
            case hardware_interface::MODE_EFFORT:
                output_msg_.command = joint_effort_command_;
                output_msg_.mode = squirrel_control_msgs::JointCommand::TORQUE_MODE;
                break;
        }

        // Publish
        pub_joint_command_.publish(output_msg_);
    }


    void ArmHardwareInterface::eSqueezedCallback(const squirrel_safety_msgs::SafetyConstPtr& msg) {
        // Check if button is pressed
        if( msg->emergency_state == true ) {
            e_squeezed_previous = true;
        }
        else  // button not pressed
        {
            if (e_squeezed_previous) {

                publishCurrentLocation();
            }
            e_squeezed_previous = false;
        }
    }


    void ArmHardwareInterface::publishCurrentLocation()
    {
        // Publish this new trajectory just once, on safety release
        ROS_INFO_STREAM_NAMED(arm_name_, "Sent updated trajectory to trajectory controller");

        // Update the trajectory message with the current positions
        for (std::size_t i = 0; i < n_dof_; ++i)
        {
            trajectory_command_msg_.points[0].positions[i] = joint_position_[i];
            trajectory_command_msg_.points[1].positions[i] = joint_position_[i];
        }

        // Send a trajectory
        pub_trajectory_command_.publish(trajectory_command_msg_);
    }

    void ArmHardwareInterface::robotDisabledCallback()
    {
        publishCurrentLocation();
    }



}