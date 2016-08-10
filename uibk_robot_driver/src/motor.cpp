#include "uibk_robot_driver/motor.hpp"

using namespace ROBOTIS;                                    // Uses functions defined in ROBOTIS namespace

namespace motor_controller {

    MotorException::MotorException(const char* msg) { this->msg = msg; }

    const char* MotorException::what() const throw() {
     return msg;
    }

    void Motor::submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, int value) {

        UINT8_T dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        for(int i = 0; i < 3; ++i) {

            dxl_comm_result = packetHandler->Write4ByteTxRx(portHandler, motorId, address, value, &dxl_error);
            if(dxl_comm_result == COMM_SUCCESS)
                return;

        }

        // if not worked in once in these 3 times --> throw exception
        throw MotorException("failed to commicate with motor");

    }

    void Motor::submitPacket(ROBOTIS::PortHandler* portHandler, int motorId, int address, UINT8_T value) {

        UINT8_T dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        for(int i = 0; i < 3; ++i) {

            dxl_comm_result = packetHandler->Write1ByteTxRx(portHandler, motorId, address, value, &dxl_error);
            if(dxl_comm_result == COMM_SUCCESS)
                return;

        }

        // if not worked in once in these 3 times --> throw exception
        throw MotorException("failed to commicate with motor");

    }

    int Motor::receivePacket(ROBOTIS::PortHandler* portHandler, int motorId, int address) {

        UINT8_T dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int dxl_present_position = 0;

        for(int i = 0; i < 3; ++i) {

            dxl_comm_result = packetHandler->Read4ByteTxRx(portHandler, motorId, address, (UINT32_T*)&dxl_present_position, &dxl_error);
            if(dxl_comm_result == COMM_SUCCESS)
                return dxl_present_position;

        }

        return dxl_present_position;

    }

    void Motor::sendNextCommand(double pos) {

        if(pos < upperLimit && pos > lowerLimit){
            submitPacket(portHandler, motorId, ADDR_PRO_GOAL_POSITION, (int) (pos / M_PI * ticks_for_180_deg));
        }else
            std::cerr << "commanded position out of security limits (com: " << pos << ", lim: " << lowerLimit << ", " << upperLimit << ")" << std::endl;

    }

    Motor::Motor(std::string deviceName, int motorId, motor_type type, float protocolVersion, double lowerLimit, double upperLimit, int baudRate) {
        this->type = type;
        this->deviceName = deviceName;
        this->motorId = motorId;
        this->protocolVersion = protocolVersion;
        this->lowerLimit = lowerLimit;
        this->upperLimit = upperLimit;
        this->baudRate = baudRate;
        nextCommandSet = false;
        
        ticks_for_180_deg = (type == DYNAMIXEL_BIG_MOTOR) ? TICKS_FOR_180_DEG_BIG : TICKS_FOR_180_DEG_SMALL;
        std_stepsize = 20.0 / ticks_for_180_deg * M_PI;
        std_max_vel_limit = 5000.0 / ticks_for_180_deg * M_PI;
        
    }

    double Motor::getStepSize() { return std_stepsize; }
    double Motor::getFrequency() { return STD_FREQUENCY; }

    void Motor::initialize() {

        // Initialize PortHandler instance
        // Set the port path
        // Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = PortHandler::GetPortHandler(deviceName.c_str());

        // Initialize Packethandler instance
        // Set the protocol version
        // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        packetHandler = PacketHandler::GetPacketHandler(protocolVersion);

        // open port
        if(portHandler->OpenPort())
            std::cout << "Succeeded to open the port " << deviceName << std::endl;
        else
            throw MotorException(std::string("opening for id " + deviceName + " failed").c_str());

        // Set port baudrate
        if(portHandler->SetBaudRate(baudRate)) {
            std::cout << "Succeeded to set a baud rate of " << baudRate << std::endl;
        } else {
            std::stringstream s;
            s << "setting baud rate of " << baudRate << " failed";
            throw MotorException(s.str().c_str());
        }

        nextCommandSet = false;

    }

    void Motor::setTorqueMode(bool mode) {

        // Enable Dynamixel#1 Torque
        int torqueMode = 0;
        if(mode)
            torqueMode = 1;

        submitPacket(portHandler, motorId, ADDR_PRO_TORQUE_ENABLE, torqueMode);

    }

    void Motor::startTorqueMode() {

        setTorqueMode(true);

    }

    void Motor::stopTorqueMode() {

        setTorqueMode(false);

    }

    void Motor::setBrakes(bool brakesOpen) {

        int brakeVal = 0;
        if(brakesOpen)
            brakeVal = 4095;

        submitPacket(portHandler, motorId, 626, brakeVal);

    }

    void Motor::releaseBrakes() {

        setBrakes(true);

    }

    void Motor::closeBrakes() {

        setBrakes(false);

    }

    void Motor::shutdown() {

        closeBrakes();
        stopTorqueMode();
        portHandler->ClosePort();

    }

    int Motor::receiveState() {

        // Read Dynamixel#1 present position
        auto dxl_present_position = receivePacket(portHandler, motorId, ADDR_PRO_PRESENT_POSITION);
        return dxl_present_position;

    }

    void Motor::spinOnce() {

        stateMutex.lock();
            bool worked = false;
            for(int i = 0; i < 3 && !worked; ++i) {
                try {
                    currentState = (double) (receiveState() / ticks_for_180_deg * M_PI);
                    worked = true;
                } catch(MotorException &ex) {

                }
            }
            if(!worked)
                throw MotorException("transmission problem (check your connection)");

        stateMutex.unlock();

        commandMutex.lock();

            if(nextCommandSet) {

                nextCommandSet = false;
                sendNextCommand(nextCommand);

            }

        commandMutex.unlock();

    }

    void Motor::simplePtp(int targetPos) {

        int stdSleepTime = (int) (1.0 / STD_FREQUENCY * 1E3);

        flushNextState();
        spinOnce();

        auto currState = getCurrentState();
        int sig = ((targetPos - currState) > 0) ? 1 : -1;
        for(; fabs(currState - targetPos) > (500 / ticks_for_180_deg * M_PI); currState += sig * std_stepsize) {
            setNextState(currState);
            spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(stdSleepTime));
        }

    }

    void Motor::move(int targetPos) {

        int stdSleepTime = (int) (1.0 / STD_FREQUENCY * 1E3);

        flushNextState();
        spinOnce();

        auto currState = getCurrentState();
        int sig = ((targetPos - currState) > 0) ? 1 : -1;
      if( fabs(currState - targetPos) > (500 / ticks_for_180_deg * M_PI))
      {
            currState += sig * std_stepsize;
            setNextState(currState);
            spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(stdSleepTime));
        }

    }

    void Motor::flushNextState() {
        commandMutex.lock();
            nextCommandSet = false;
        commandMutex.unlock();
    }

    void Motor::goHome() {
        simplePtp(0);
    }

    double Motor::getCurrentState() {

        double stateBackup;
        stateMutex.lock();
            stateBackup = currentState;
        stateMutex.unlock();
        return stateBackup;

    }

    void Motor::setNextState(double state) {

        commandMutex.lock();
            nextCommandSet = true;
            nextCommand = state;
        commandMutex.unlock();

    }

    double Motor::getMaxVelLimit() {
        return std_max_vel_limit;
    }
}
