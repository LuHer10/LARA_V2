#ifndef SERVO_H
#define SERVO_H
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <cmath>

#include <dynamixel_sdk/dynamixel_sdk.h> // Uses DYNAMIXEL SDK library

#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE                    57600

#define ADDR_OPERATING_MODE 11
#define POSITION_CONTROL 3
#define EXTENDED_POSITION_CONTROL 4

#define ADDR_PROFILE_VELOCITY 112
#define ADDR_PROFILE_ACCELERATION 108


#define PROTOCOL_VERSION  2.0

#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

inline int32_t degToPos(float ang)
{
    float fout = (ang/360.0f)*MAXIMUM_POSITION_LIMIT;
    int32_t out = (int32_t)fout;
    return out;
}

inline int32_t radToPos(float rad)
{
    float fout = (rad/(2.0f*M_PI))*MAXIMUM_POSITION_LIMIT;
    int32_t out = (int32_t)fout;
    return out;
}

inline float posToDeg(int32_t pos)
{
    float ang = ((float)pos * 360.0f)/((float)MAXIMUM_POSITION_LIMIT);
    return ang;
}

inline float posToRad(int32_t pos)
{
    float rad = ((float)pos * (2.0f*M_PI))/((float)MAXIMUM_POSITION_LIMIT);
    return rad;
}

class Servo
{
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    bool initialized = false;

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;

    int32_t goalPos[253];
    int32_t currPos[253];

    bool torque_enabled[253] = {false};

public:
    int init();
    int init(const char *devName);

    void posContMode(int id);
    void extPosContMode(int id);

    void enableTorque(int id);
    void disableTorque(int id);
    void writePos(int id, int32_t pos);
    int32_t readPos(int id);
    void setOffset(int id, int32_t off);
    int32_t getOffset(int id);

    void setMaxVel(int id, int32_t data);
    void setMaxAcc(int id, int32_t data);

};

#endif