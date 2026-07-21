#ifndef BASE_H
#define BASE_H

#include "roboclaw.h"
#include <cmath>

#define BAUD_RATE B38400
#define DEVICE "/dev/ttyAMA0"

#define QPPR 1300 // Quadrature Pulses Per Revolution
#define WHEEL_DIAMETER 0.06F // Wheel diameter in meters
constexpr float WHEEL_RADIUS = WHEEL_DIAMETER/2.0F;
#define MAX_RPM 330 
#define LENGTH 0.323F
#define WIDTH 0.3F
#define PI 3.14159F

constexpr float LX = LENGTH/2.0F;
constexpr float LY = WIDTH/2.0F;
constexpr float LXY = LX+LY;

constexpr float QPPR_2_RAD = PI*2.0f/(QPPR);//PI*WHEEL_DIAMETER/(QPPR);
constexpr float RAD_2_QPPR = 1.0F/(QPPR_2_RAD);

struct Odometry
{
    float x;
    float y;
    float ang;
};

enum motors
{
    M_FRONT_LEFT = 0,
    M_FRONT_RIGHT,
    M_BACK_LEFT,
    M_BACK_RIGHT    
};

union int32_u
{
    uint32_t t_uint;
    int32_t t_int;
};

class Base
{
private:
    RoboClaw roboclaw{DEVICE, BAUD_RATE};

    //{FL, FR, BL, BR}
    int32_u targetVelocity[4] = {0};
    int32_u currVelocity[4] = {0};
    int32_u encoderCounts[4] = {0};

    float x, y, ang;
    float vx, vy, vang;
    

public:

    Base()
    {
        roboclaw.resetEncoders(FRONT_ADDRESS);
        roboclaw.resetEncoders(BACK_ADDRESS);
        roboclaw.M1M2Speed(FRONT_ADDRESS, 0, 0);
        roboclaw.M1M2Speed(BACK_ADDRESS, 0, 0);
        resetOdometry();
    }

    ~Base()
    {
        roboclaw.M1M2Speed(FRONT_ADDRESS, 0, 0);
        roboclaw.M1M2Speed(BACK_ADDRESS, 0, 0);
    }

    void setVelocity(float vx, float vy, float vrad);

    void FK(const float vm[4], float &vx, float &vy, float &vrad);
    void IK(const float &vx, const float &vy, const float &vrad, float vm[4]);

    Odometry getOdometry(){return Odometry{x, y, ang};}
    void resetOdometry();

    void sendSpeed();
    void readEncoders();
    void readSpeeds();


};

#endif 