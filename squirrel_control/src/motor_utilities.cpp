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
    delete port_handler_;
    delete packet_handler_;
  }
  
  
  void MotorUtilities::setMode(control_modes::ControlMode mode) {
    //remove once it is supported
    throw_control_error(true, "setMode not yet supported");
    
    UINT8_T error = 0;
    try {
      motor_lock_.lock();
      current_mode_ = mode;
      for(const auto motor : motors_){
	packet_handler_->Write1ByteTxRx(port_handler_, motor.id,
					motor.tool->ctrl_table_["operating_mode"]->address,
					(UINT8_T)current_mode_, &error);
	std::cout << "Error: " << error << std::endl;
	throw_control_error(error,
			    "Failed to switch motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_
			    << ") into mode " << mode);
      }
      motor_lock_.unlock();
    } catch (std::exception &ex) {
      motor_lock_.unlock();
      throw_control_error(true, ex.what());
    }
  }
  
  
  control_modes::ControlMode MotorUtilities::getMode() {
    return current_mode_;
  }
  
  
  std::vector<Motor> MotorUtilities::getMotors() {
    return motors_;
  }
  
  
  bool MotorUtilities::motorsReady() {
    return motors_ready_;
  }
  
  
  bool MotorUtilities::torqueEnabled() {
    return torque_enabled_;
  }
  
  
  bool MotorUtilities::initMotors(std::string motor_port, std::vector<int> motors) {
    port_handler_ = ROBOTIS::PortHandler::GetPortHandler(motor_port.c_str());
    if (!port_handler_->OpenPort()) {
      throw_control_error(true, "Failed to open motor port!");
    }		
    if (!port_handler_->SetBaudRate(BAUD_RATE_)) {
      throw_control_error(true, "Failed to set baud rate!");
    }
    
    //do we find the whole gang?
    packet_handler_ = ROBOTIS::PacketHandler::GetPacketHandler(2.0);
    UINT8_T error = 0;
    for(auto const& motor: motors) {
      packet_handler_->Ping(port_handler_, motor, &error);
      throw_control_error(error, "Failed to ping motor: " << motor);
    }
    
    //initialize them
    motors_ = std::vector<Motor>();
    
    uint8_t dynamixel_error = 0;
    int dynamixel_id = 0;
    uint16_t dynamixel_num = 0;
    
    for(dynamixel_id = 1; dynamixel_id < 253; dynamixel_id++) {
      if (packet_handler_->Ping(port_handler_, dynamixel_id, &dynamixel_num, &dynamixel_error) == 0) {
	std::cout << "Found model: " << dynamixel_num << " with id " << dynamixel_id << std::endl;
	dynamixel_tool::DynamixelTool* tool = new dynamixel_tool::DynamixelTool(dynamixel_id, dynamixel_num, 2.0);
	Motor motor_;
	motor_.id = dynamixel_id;
	motor_.tool = tool;
	motors_.push_back(motor_);
      } 
    }	
    std::cout << "Found " << motors_.size() << " motors" << std::endl;
    return true;
  }
  
  
  bool MotorUtilities::startMotors() {
    UINT8_T error = 0;
    for(auto const& motor: motors_) {
      current_mode_ = control_modes::ControlMode::POSITION_MODE;
      std::cout << "Starting motor " << static_cast<int>(motor.id)<< std::endl;
      packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 1, &error);
      throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
      packet_handler_->Write2ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 4095, &error);
      throw_control_error(error, "Failed to loosen brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
    }
    motors_ready_ = true;
    std::cout << "Motors started." << std::endl;
    return true;
  }
  
  
  bool MotorUtilities::stopMotors() {
    UINT8_T error = 0;
    for(auto const& motor: motors_) {
      packet_handler_->Write2ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["external_port_data_1"]->address, 0, &error);
      throw_control_error(error, "Failed to set brakes for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
    }
    motors_ready_ = false;
    return true;
  }
  
  
  bool MotorUtilities::enableTorque() {
    UINT8_T error = 0;
    for(auto const& motor: motors_) {
      packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 1, &error);
      throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
    }
    torque_enabled_ = true;
    return true;
  }
  
  
  bool MotorUtilities::disableTorque() {
    UINT8_T error = 0;
    for(auto const& motor: motors_) {
      packet_handler_->Write1ByteTxRx(port_handler_, motor.id, motor.tool->ctrl_table_["torque_enable"]->address, 0, &error);
      throw_control_error(error, "Failed to enable torque for motor " << motor.id << " (" << motor.tool->model_name_ << ")");
    }
    torque_enabled_ = false;
    return true;
  }
  
  
  bool MotorUtilities::write(std::vector<double> commands)
  {
    throw_control_error(commands.size() != motors_.size(), "Wrong number of commands! Got " << commands.size() << ", but expected " << motors_.size());
    
    int i = 0;
    UINT8_T error = 0;
    
    motor_lock_.lock();
    
    //we throw a bunch of own exceptions - let's catch them all and forward them to at all times release the lock
    //Motor 3 has an offset of 228000 ticks - we do not have to treat that one, the dynamiel formware takes care of that
    try {
      for (auto const motor : motors_) {
	switch (current_mode_) {
	case control_modes::ControlMode::POSITION_MODE: {
	  double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
	  int goal_position =  static_cast<int>(commands.at(i) / rad_per_tick);
	  int min_ = motor.tool->value_of_min_radian_position_;
	  int max_ = motor.tool->value_of_max_radian_position_;
	  throw_control_error(goal_position < min_ || goal_position > max_,
			      "Motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_
			      << ") exceeds its limits [" << min_ << "," << max_
			      << "] with goal position: " << goal_position);

	  std::cout << "Commanded position for motor " << static_cast<int>(motor.id) << ": " << goal_position << 
	    " with address " <<  static_cast<int>(motor.tool->ctrl_table_["goal_position"]->address) << std::endl;

	  packet_handler_->Write4ByteTxRx(port_handler_, motor.id,
					  motor.tool->ctrl_table_["goal_position"]->address,
					  goal_position, &error);	  
	}
	  break;
	  
	case control_modes::ControlMode::VELOCITY_MODE: {
	  UINT32_T velocity_limit;
	  packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["velocity_limit"]->address,
					 &velocity_limit, &error);
	  throw_control_error(error, "Failed to read velocity limit for motor " << motor.id << " ("
			      << motor.tool->model_name_
			      << ")");
	  UINT32_T commanded_velocity = (UINT32_T)commands.at(i);
	  throw_control_error( commanded_velocity > velocity_limit,
			       "Motor " << motor.id << " (" << motor.tool->model_name_
			       << ") exceeds its velocity [" << velocity_limit << "] with commanded velocity: " << commanded_velocity);
	  packet_handler_->Write4ByteTxRx(port_handler_, motor.id,
					  motor.tool->ctrl_table_["goal_velocity"]->address,
					  commanded_velocity, &error);
	}
	  break;
	  
	case control_modes::ControlMode::TORQUE_MODE: {
	  UINT16_T torque_limit;
	  packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["torque_limit"]->address,
					 &torque_limit, &error);
	  throw_control_error(error, "Failed to read torque limit for motor " << static_cast<int>(motor.id) << " ("
			      << motor.tool->model_name_
			      << ")");
	  UINT16_T commanded_torque = (UINT16_T)commands.at(i);
	  throw_control_error( commanded_torque > torque_limit,
			       "Motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_
			       << ") exceeds its velocity [" << torque_limit << "] with commanded velocity: " << commanded_torque);
	  packet_handler_->Write2ByteTxRx(port_handler_, motor.id,
					  motor.tool->ctrl_table_["goal_torque"]->address,
					  commanded_torque, &error);
	}
	  break;
	  
	default:
	  throw_control_error(true, "Unknown mode: " << current_mode_);
	}
	//this theoretically should never be evaluated, but for sake of completeness...
	throw_control_error(error,
			    "Failed to command motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ") in "
			    << current_mode_ << " mode");
	i++;
      }
    } catch(const std::exception &ex) {
      motor_lock_.unlock();
      throw_control_error(true, ex.what());
    }
    
    try{
      motor_lock_.unlock();
    } catch(std::exception &ex) {
      throw_control_error(true, ex.what());
      motor_lock_.unlock();
      return false;
    }
    return true;
  }
  
  
  std::vector<double> MotorUtilities::read() {
    UINT8_T error = 0;    
    switch(current_mode_) {
    case control_modes::ControlMode::POSITION_MODE:
      {
	std::vector<double> positions = {};
	UINT32_T value;
	for (auto const motor : motors_) {
	  packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["present_position"]->address,
					 &value, &error);
	  throw_control_error(error, "Error reading position for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")");
	  double rad_per_tick = motor.tool->max_radian_ / motor.tool->value_of_max_radian_position_;
	  int real_value = static_cast<int>(value);
	  positions.push_back(double(real_value*rad_per_tick));
	}
	
	return positions;
      }
    case control_modes::ControlMode::VELOCITY_MODE:
      {
	std::vector<double> velocities = {};
	UINT32_T value;
	for (auto const motor : motors_) {
	  packet_handler_->Read4ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["present_velocity"]->address,
					 &value, &error);
	  throw_control_error(error, "Error reading velocity for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")");
	  velocities.push_back(double(value));
	}
	return velocities;
      }
    case control_modes::ControlMode::TORQUE_MODE:
      {
	std::vector<double> torques = {};
	UINT16_T value;
	for (auto const motor : motors_) {
	  packet_handler_->Read2ByteTxRx(port_handler_, motor.id,
					 motor.tool->ctrl_table_["present_current"]->address,
					 &value, &error);
	  throw_control_error(error, "Error reading torque for motor " << static_cast<int>(motor.id) << " (" << motor.tool->model_name_ << ")");
	  torques.push_back(abs(double(value)*CURRENT_TO_TORQUE_RATIO_));
	}
	return torques;
      }
    default:
      throw_control_error(true, "Unknown mode: " << current_mode_);
    }
  }  
}
