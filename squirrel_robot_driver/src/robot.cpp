#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class MyRobot : public hardware_interface::RobotHW
{
public:
    MyRobot()
    {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_b);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
        jnt_pos_interface.registerHandle(pos_handle_a);

        hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
        jnt_pos_interface.registerHandle(pos_handle_b);

        registerInterface(&jnt_pos_interface);
    }

    bool init()
    {
        // Populate pos_cmd_interface_ with joint handles...

        // Get joint handle of interest
        JointHandle joint_handle = pos_cmd_interface_.getHandle("foo_joint");

        JointLimits limits;
        SoftJointLimits soft_limits;
        // Populate with any of the methods presented in the previous example...

        // Register handle in joint limits interface
        PositionJointSoftLimitsHandle handle(joint_handle, // We read the state and read/write the command
                                             limits,       // Limits spec
                                             soft_limits)  // Soft limits spec

        jnt_limits_interface_.registerHandle(handle);
    }

    void read(ros::Time time, ros::Duration period)
    {
        // Read actuator state from hardware...

        // Propagate current actuator state to joints...
    }

    void write(ros::Time time, ros::Duration period)
    {
        // Enforce joint limits for all registered handles
        // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
        jnt_limits_interface_.enforceLimits(period);

        // Porpagate joint commands to actuators...

        // Send actuator command to hardware...
    }



private:
    hardware_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
};