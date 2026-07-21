#include "servo.h"

int Servo::init()
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    // Open port
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    }
    else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        return -1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return -2;
    }
    return 0;
}

int Servo::init(const char *devName)
{
    initialized = true;
    portHandler = dynamixel::PortHandler::getPortHandler(devName);
    packetHandler = 
        dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    // Open port
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    }
    else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        return -1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return -2;
    }
    return 0;
}

void Servo::posContMode(int id)
{
    if(torque_enabled[id] == true) return;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, 
        ADDR_OPERATING_MODE, POSITION_CONTROL, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        printf("Succeeded enabling POSITION CONTROL Mode.\n");
    }
}

void Servo::extPosContMode(int id)
{
    if(torque_enabled[id] == true) return;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, 
        ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        printf("Succeeded enabling EXTENDED POSITION CONTROL Mode.\n");
    }
}

void Servo::enableTorque(int id)
{
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, 
        ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        torque_enabled[id] = true;
        printf("Succeeded enabling DYNAMIXEL Torque.\n");
    }
}

void Servo::disableTorque(int id)
{
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, 
        ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        torque_enabled[id] = false;
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }
}

void Servo::writePos(int id, int32_t pos)
{
    goalPos[id] = pos;
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, 
        ADDR_GOAL_POSITION, goalPos[id], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

int32_t Servo::readPos(int id)
{
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, 
        ADDR_PRESENT_POSITION, (uint32_t*)&currPos[id], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    return currPos[id];
}

void Servo::setMaxVel(int id, int32_t data)
{
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, 
        ADDR_PROFILE_VELOCITY, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}

void Servo::setMaxAcc(int id, int32_t data)
{
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, 
        ADDR_PROFILE_ACCELERATION, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
}