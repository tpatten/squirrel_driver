//
// Created by Philipp Zech on 12.04.17.
//

#include <squirrel_control/motor_utilities.h>


namespace motor_control {

    MotorUtilities::MotorUtilities() {
    }

    MotorUtilities::~MotorUtilities() {
        stopMotors();
        for(auto const& motor : motors_) {
            delete motor.tool;
        }
        delete portHandler_;
        delete packetHandler_;
    }


    void MotorUtilities::setMode(control_modes::ControlMode mode) {
	    UINT8_T error;
	    try {
		    motorLock_.lock();
		    currentMode_ = mode;
		    for(const auto motor : motors_){
			    packetHandler_->Write1ByteTxRx(portHandler_, motor.id,
			                                        motor.tool->ctrl_table_["operating_mode"]->address,
			                                        mode, &error);
			    throw_control_error(error,
			                        "Failed to switch motor " << motor.id << " (" << motor.tool->model_name_
			                                                  << ") into mode " << mode);
		    }
		    motorLock_.unlock();
	    } catch (std::exception &ex) {
		    throw ex;
	    }
    }


	control_modes::ControlMode MotorUtilities::getMode() {
        return currentMode_;
    }


    std::vector<Motor> MotorUtilities::getMotors() {
        return motors_;
    }


    bool MotorUtilities::motorsReady() {
        return motorsReady_;
    }


    bool MotorUtilities::torqueEnabled() {
        return torqueEnabled_;
    }


    bool MotorUtilities::initMotors(std::string motor_port, std::vector<int> motors) {
        try {
	        portHandler_ = ROBOTIS::PortHandler::GetPortHandler(motor_port.c_str());
	        portHandler_->SetBaudRate(BAUD_RATE_);
	        portHandler_->OpenPort();
        } catch(const std::exception &e) {
            throw_control_error(true, e.what());
        }

        //do find the whole gang?
	    packetHandler_ = ROBOTIS::Protocol2PacketHandler::GetInstance();
        UINT8_T error = 0;
        for(auto const& motor: motors) {
	        packetHandler_->Ping(portHandler_, motor, &error);
            throw_control_error(error, "Failed to ping motor: " << motor);
        }

        //initialize them
        motors_ = std::vector<Motor>();
        for(auto const& motor: motors) {
            uint16_t model_id = packetHandler_->Read2ByteTx(portHandler_, motor, 0);
            dynamixel_tool::DynamixelTool tool = dynamixel_tool::DynamixelTool(motor, model_id, 2.0);
            Motor motor_;
            motor_.id = motor;
            motor_.tool = &tool;
            motors_.push_back(motor_);
        }
        return true;
    }


    bool MotorUtilities::startMotors() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packetHandler_->Write2ByteTxRx(portHandler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 4095, &error);
            throw_control_error(error, "Failed to loosen brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    motorsReady_ = true;
        return true;
    }


    bool MotorUtilities::stopMotors() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packetHandler_->Write1ByteTxRx(portHandler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 0, &error);
            throw_control_error(error, "Failed to set brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    motorsReady_ = false;
        return true;
    }


    bool MotorUtilities::enableTorque() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packetHandler_->Write1ByteTxRx(portHandler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 1, &error);
            throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    torqueEnabled_ = true;
        return true;
    }


    bool MotorUtilities::disableTorque() {
        UINT8_T error;
        for(auto const& motor: motors_) {
	        packetHandler_->Write1ByteTxRx(portHandler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 0, &error);
            throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
	    torqueEnabled_ = false;
        return true;
    }


    //PID control should actually already be done in the motor
    //this only consider low level (HW) limits
    //conversion from joint pos to velocity/torque done in squirrel_hw_interface or here?
    bool MotorUtilities::write(std::vector<double> commands)
    {
        throw_control_error(commands.size() != motors_.size(), "Wrong number of commands! Got " << commands.size() << ", but expected " << motors_.size());

        int i = 0;
        UINT8_T error;

	    motorLock_.lock();

        //we throw a bunch of own exceptions - let's catch them all and forward them to at all times release the lock
        try {
            for (auto const motor : motors_) {
                switch (currentMode_) {
	                case control_modes::ControlMode::POSITION_MODE: {
                        double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
                        UINT32_T goal_position = (UINT32_T) commands.at(i) / rad_per_tick;
                        UINT32_T homing_offset;
		                packetHandler_->Read4ByteTxRx(portHandler_, motor.id,
                                                           motor.tool->ctrl_table_["homing_offset"]->address,
                                                           &homing_offset, &error);
                        throw_control_error(error, "Failed to read homing offset for motor " << motor.id << " ("
                                                                                             << motor.tool->model_name_
                                                                                             << ")");
                        UINT32_T min_with_offset = homing_offset + motor.tool->value_of_min_radian_position_;
                        UINT32_T max_with_offset = homing_offset + motor.tool->value_of_max_radian_position_;
                        UINT32_T goal_position_with_offset = goal_position + homing_offset;
                        throw_control_error(goal_position_with_offset < min_with_offset || goal_position_with_offset > max_with_offset,
                                            "Motor " << motor.id << " (" << motor.tool->model_name_
                                                     << ") exceeds its limits [" << min_with_offset << "," << max_with_offset
                                                     << "] with goal position: " << goal_position);
		                packetHandler_->Write2ByteTxRx(portHandler_, motor.id,
                                                            motor.tool->ctrl_table_["goal_position"]->address,
                                                            goal_position_with_offset, &error);
                        }
                        break;

	                case control_modes::ControlMode::VELOCITY_MODE: {
                        UINT32_T velocity_limit;
		                packetHandler_->Read4ByteTxRx(portHandler_, motor.id,
                                                           motor.tool->ctrl_table_["velocity_limit"]->address,
                                                           &velocity_limit, &error);
                        throw_control_error(error, "Failed to read velocity limit for motor " << motor.id << " ("
                                                                                             << motor.tool->model_name_
                                                                                             << ")");
                        UINT32_T commanded_velocity = (UINT32_T)commands.at(i);
                        throw_control_error( commanded_velocity > velocity_limit,
                                            "Motor " << motor.id << " (" << motor.tool->model_name_
                                                     << ") exceeds its velocity [" << velocity_limit << "] with commanded velocity: " << commanded_velocity);
		                packetHandler_->Write2ByteTxRx(portHandler_, motor.id,
                                                            motor.tool->ctrl_table_["goal_velocity"]->address,
                                                            commanded_velocity, &error);
                        }
                        break;

                    case control_modes::ControlMode::TORQUE_MODE: {
                        UINT16_T torque_limit;
	                    packetHandler_->Read2ByteTxRx(portHandler_, motor.id,
                                                           motor.tool->ctrl_table_["torque_limit"]->address,
                                                           &torque_limit, &error);
                        throw_control_error(error, "Failed to read torque limit for motor " << motor.id << " ("
                                                                                              << motor.tool->model_name_
                                                                                              << ")");
                        UINT16_T commanded_torque = (UINT16_T)commands.at(i);
                        throw_control_error( commanded_torque > torque_limit,
                                             "Motor " << motor.id << " (" << motor.tool->model_name_
                                                      << ") exceeds its velocity [" << torque_limit << "] with commanded velocity: " << commanded_torque);
	                    packetHandler_->Write2ByteTxRx(portHandler_, motor.id,
                                                            motor.tool->ctrl_table_["goal_torque"]->address,
                                                            commanded_torque, &error);
                        }
                        break;

                    default:
                        throw_control_error(true, "Unknown mode: " << currentMode_);
                }
                throw_control_error(error,
                                    "Failed to command motor " << motor.id << " (" << motor.tool->model_name_ << ") in "
                                                               << currentMode_ << " mode");
                i++;
            }
        }catch(const std::exception &ex) {
	        motorLock_.unlock();
            throw ex;
        }

	    motorLock_.unlock();
        return true;
    }


    std::vector<double> MotorUtilities::read() {
	    UINT8_T error;
        switch(currentMode_) {
            case control_modes::ControlMode::POSITION_MODE:
                {
	                std::vector<double> positions = {};
					UINT16_T value;
	                for (auto const motor : motors_) {
		                packetHandler_->Read2ByteTxRx(portHandler_, motor.id,
		                                                   motor.tool->ctrl_table_["present_position"]->address,
		                                                   &value, &error);
		                throw_control_error(error, "Error reading position for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
		                positions.push_back(double(value));
	                }
                    return positions;
                }
            case control_modes::ControlMode::VELOCITY_MODE:
                {
	                std::vector<double> velocities = {};
	                UINT16_T value;
	                for (auto const motor : motors_) {
		                packetHandler_->Read2ByteTxRx(portHandler_, motor.id,
		                                                   motor.tool->ctrl_table_["present_velocity"]->address,
		                                                   &value, &error);
		                throw_control_error(error, "Error reading velocity for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
		                velocities.push_back(double(value));
	                }
	                return velocities;
                }
            case control_modes::ControlMode::TORQUE_MODE:
                {

	                //TODO correct?
	                std::vector<double> torques = {};
	                UINT16_T value;
	                for (auto const motor : motors_) {
		                packetHandler_->Read2ByteTxRx(portHandler_, motor.id,
		                                              motor.tool->ctrl_table_["present_current"]->address,
		                                              &value, &error);
		                throw_control_error(error, "Error reading torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
		                torques.push_back(abs(double(value)*CURRENT_TO_TORQUE_RATIO_));
	                }
	                return torques;
                }
            default:
                throw_control_error(true, "Unknown mode: " << currentMode_);
        }
    }

}