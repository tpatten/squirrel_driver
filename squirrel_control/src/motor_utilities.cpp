//
// Created by Philipp Zech on 12.04.17.
//

#include <squirrel_control/motor_utilities.h>


namespace motor_control {

    MotorUtilities::MotorUtilities() {
    }

    MotorUtilities::~MotorUtilities() {
        this->stopMotors();
        for(auto const& motor : this->motors) {
            delete motor.tool;
        }
        delete this->portHandler;
        delete this->packetHandler;
    }


    void MotorUtilities::setMode(squirrel_control::SquirrelControlMode mode) {
        this->current_mode = mode;
    }


    squirrel_control::SquirrelControlMode MotorUtilities::getMode() {
        return this->current_mode;
    }


    std::vector<Motor> MotorUtilities::getMotors() {
        return this->motors;
    }


    bool MotorUtilities::motorsReady() {
        return this->motors_ready;
    }


    bool MotorUtilities::torqueEnabled() {
        return this->torque_enabled;
    }


    bool MotorUtilities::initMotors(std::string motor_port, std::vector<int> motors) {
        try {
            this->portHandler = ROBOTIS::PortHandler::GetPortHandler(motor_port.c_str());
            this->portHandler->SetBaudRate(this->BAUD_RATE);
            this->portHandler->OpenPort();
        } catch(const std::exception &e) {
            throw_control_error(true, e.what());
        }

        //do find the whole gang?
        this->packetHandler = ROBOTIS::Protocol2PacketHandler::GetInstance();
        UINT8_T error = 0;
        for(auto const& motor: motors) {
            this->packetHandler->Ping(this->portHandler, motor, &error);
            throw_control_error(error, "Failed to ping motor: " << motor);
        }

        //initialize them
        this->motors = std::vector<Motor>();
        for(auto const& motor: motors) {
            uint16_t model_id = this->packetHandler->Read2ByteTx(this->portHandler, motor, 0);
            dynamixel_tool::DynamixelTool tool = dynamixel_tool::DynamixelTool(motor, model_id, 2.0);
            Motor motor_;
            motor_.id = motor;
            motor_.tool = &tool;
            this->motors.push_back(motor_);
        }
        return true;
    }


    bool MotorUtilities::startMotors() {
        UINT8_T error;
        for(auto const& motor: this->motors) {
            this->packetHandler->Write2ByteTxRx(this->portHandler, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 4095, &error);
            throw_control_error(error, "Failed to loosen brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
        this->motors_ready = true;
        return true;
    }


    bool MotorUtilities::stopMotors() {
        UINT8_T error;
        for(auto const& motor: this->motors) {
            this->packetHandler->Write1ByteTxRx(this->portHandler, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 0, &error);
            throw_control_error(error, "Failed to set brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
        this->motors_ready = false;
        return true;
    }


    bool MotorUtilities::enableTorque() {
        UINT8_T error;
        for(auto const& motor: this->motors) {
            this->packetHandler->Write1ByteTxRx(this->portHandler, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 1, &error);
            throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
        this->torque_enabled = true;
        return true;
    }


    bool MotorUtilities::disableTorque() {
        UINT8_T error;
        for(auto const& motor: this->motors) {
            this->packetHandler->Write1ByteTxRx(this->portHandler, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 0, &error);
            throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
        }
        this->torque_enabled = false;
        return true;
    }


    //TODO limit check (soft limits checked in squirrel hardware interface))
    bool MotorUtilities::write(std::vector<double> commands)
    {
        int i = 0;
        UINT8_T error;

        this->motor_lock.lock();

        //we throw a bunch of own exceptions - let's catch them all and forward them to at all times release the lock
        try {

            for (auto const motor : this->motors) {
                switch (this->current_mode) {
                    case squirrel_control::POSITION: {
                        this->packetHandler->Write1ByteTxRx(this->portHandler, motor.id,
                                                            motor.tool->ctrl_table_["operating_mode"]->address,
                                                            DYNAMIXEL_POSITION_MODE, &error);
                        throw_control_error(error,
                                            "Failed to switch motor " << motor.id << " (" << motor.tool->model_name_
                                                                      << ") into position mode");
                        UINT32_T present_position;
                        this->packetHandler->Read4ByteTxRx(this->portHandler, motor.id,
                                                           motor.tool->ctrl_table_["present_position"]->address,
                                                           &present_position, &error);
                        throw_control_error(error, "Failed to read present position for motor " << motor.id << " ("
                                                                                                << motor.tool->model_name_
                                                                                                << ")");
                        double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
                        UINT32_T goal_in_ticks = (UINT32_T) commands.at(i) / rad_per_tick;
                        UINT32_T goal_position = present_position + goal_in_ticks;
                        UINT32_T homing_offset;
                        this->packetHandler->Read4ByteTxRx(this->portHandler, motor.id,
                                                           motor.tool->ctrl_table_["homing_offset"]->address,
                                                           &homing_offset, &error);
                        throw_control_error(error, "Failed to read homing offset for motor " << motor.id << " ("
                                                                                             << motor.tool->model_name_
                                                                                             << ")");
                        UINT32_T real_lower = homing_offset + motor.tool->value_of_min_radian_position_;
                        UINT32_T real_upper = homing_offset + motor.tool->value_of_max_radian_position_;
                        throw_control_error(goal_position > real_lower && goal_position < real_upper,
                                            "Motor " << motor.id << " (" << motor.tool->model_name_
                                                     << ") exceeds its limits [" << real_lower << "," << real_upper
                                                     << "] with goal position: " << goal_position);
                        this->packetHandler->Write2ByteTxRx(this->portHandler, motor.id,
                                                            motor.tool->ctrl_table_["goal_position"]->address,
                                                            goal_position, &error);
                    }
                        break;

                    case squirrel_control::VELOCITY: {
                        this->packetHandler->Write1ByteTxRx(this->portHandler, motor.id,
                                                            motor.tool->ctrl_table_["operating_mode"]->address,
                                                            DYNAMIXEL_VELOCITY_MODE, &error);
                        throw_control_error(error,
                                            "Failed to switch motor " << motor.id << " (" << motor.tool->model_name_
                                                                      << ") into velocity mode");
                        UINT32_T present_velocity;
                        this->packetHandler->Read4ByteTxRx(this->portHandler, motor.id,
                                                           motor.tool->ctrl_table_["present_velocity"]->address,
                                                           &present_velocity, &error);
                        throw_control_error(error, "Failed to read present velocity for motor " << motor.id << " ("
                                                                                                << motor.tool->model_name_
                                                                                                << ")");

                        this->packetHandler->Write2ByteTxRx(this->portHandler, motor.id,
                                                            motor.tool->ctrl_table_["goal_velocity"]->address,
                                                            commands.at(i), &error);
                    }
                        break;
                    case squirrel_control::TORQUE: {
                        this->packetHandler->Write1ByteTxRx(this->portHandler, motor.id,
                                                            motor.tool->ctrl_table_["operating_mode"]->address,
                                                            DYNAMIXEL_TORQUE_MODE, &error);
                        throw_control_error(error,
                                            "Failed to switch motor " << motor.id << " (" << motor.tool->model_name_
                                                                      << ") into torque mode");
                        this->packetHandler->Write2ByteTxRx(this->portHandler, motor.id,
                                                            motor.tool->ctrl_table_["goal_torque"]->address,
                                                            commands.at(i),
                                                            &error);
                    }
                        break;
                    default:
                        throw_control_error(true, "Unknown mode: " << this->current_mode);
                }
                throw_control_error(error,
                                    "Failed to command motor " << motor.id << " (" << motor.tool->model_name_ << ") in "
                                                               << this->current_mode << " mode");
                i++;
            }
        }catch(const std::runtime_error & e) {
            this->motor_lock.unlock();
            throw e;
        }

        this->motor_lock.unlock();
        return true;
    }
}