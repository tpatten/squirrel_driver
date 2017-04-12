//
// Created by Philipp Zech on 11.04.17.
//


#include <squirrel_control/arm_hardware_interface.h>

namespace squirrel_control
{

    ArmHardwareInterface::ArmHardwareInterface(const std::string &arm_name, double loop_hz) :
            ArmInterface(arm_name, loop_hz)
    {

    }

}