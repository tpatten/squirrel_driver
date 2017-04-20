//
// Created by Philipp Zech on 20.04.17.
//

#ifndef SQUIRREL_CONTROL_SQUIRREL_SIM_HW_INTERFACE_H
#define SQUIRREL_CONTROL_SQUIRREL_SIM_HW_INTERFACE_H

#include <squirrel_control/squirrel_hw_interface.h>

namespace squirrel_control
{

/** \brief Hardware interface for a robot */
    class SquirrelSimHWInterface : public SquirrelHWInterface
    {
    public:
        /**
         * \brief Constructor
         * \param nh - Node handle for topics.
         */
        SquirrelSimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

        ~SquirrelSimHWInterface();

        /** \brief Initialize the robot hardware interface */
        virtual void init();

        /** \brief Read the state from the robot hardware. */
        virtual void read(ros::Duration &elapsed_time);

        /** \brief Write the command to the robot hardware. */
        virtual void write(ros::Duration &elapsed_time);

        /** \breif Enforce limits for all values before writing */
        virtual void enforceLimits(ros::Duration &period);

    protected:

        /** \brief Basic model of system for position control */
        virtual void positionControlSimulation(ros::Duration &elapsed_time, const std::size_t joint_id);

        // Name of this class
        std::string name_;

        // Simulated controller
        double p_error_;
        double v_error_;

        // For position controller to estimate velocity
        std::vector<double> joint_position_prev_;

        // Send commands in different modes
        int sim_control_mode_ = 0;

    };  // class

}  // namespace


#endif //SQUIRREL_CONTROL_SQUIRREL_SIM_HW_INTERFACE_H
