#ifndef ARM_H
#define ARM_H

#include "socketcan.h"
#include "GIM6010.h"
#include "steadywin.h"
#include <cmath>
#include <thread>
#include <iostream>
#include <atomic>
#include <stdexcept>

#include "servo.h"

#define BIG_ID 11
#define MEDIUM_ID 2
#define SMALL_ID 1

#define M1_REST M_PI
#define M2_REST 0.0f
#define M3_REST 2.0f*M_PI

#define GRIP_ID 50
#define GRIPPER_OPEN  3.0f*M_PI/4.0f
#define GRIPPER_CLOSE M_PI/2.0f


#define WRIST_ID 10

inline float deg2rad(float _deg)
{
    float _rad = _deg*M_PI/180.0f;
    return _rad;
}

class Arm
{
public:

    Servo servos;

    float l1, l2, l3;

    float wristAngle = 0.0f;

    std::atomic<float> q1, q2, q3;
    std::atomic<float> x, y, th;

    std::atomic<float> m1, m2, m3;

    std::atomic<float> mBig, mMed, mSmall;

    SocketCAN* can;

    GIM6010* bigMotor;
    Steadywin* mediumMotor;
    Steadywin* smallMotor;

    std::thread controlThread;
    std::atomic<bool> stopThread{false};

    std::atomic<float> m1_offset = 0.0f;
    std::atomic<float> m2_offset = 0.0f;
    std::atomic<float> m3_offset = 0.0f;

    std::atomic<float> m1_start, m2_start, m3_start;

    // Control thread: streams m1..m3 setpoints to the motors and reads back
    // their positions into mBig/mMed/mSmall until stopThread is set.
    void motorControl();

public:
    // Reads the current motor positions, treats them as the rest pose
    // (M1_REST, M2_REST, M3_REST) and starts the control thread.
    // Throws std::runtime_error if any CAN motor does not respond.
    Arm(SocketCAN* _can, float _l1, float _l2, float _l3);
    ~Arm();

    // Motor angles <-> joint angles
    void m2q(float _m1, float _m2, float _m3,
        float& _q1, float& _q2, float& _q3);
    void q2m(float _q1, float _q2, float _q3,
        float& _m1, float& _m2, float& _m3);
    void m2q(std::atomic<float>& _m1, std::atomic<float>& _m2, std::atomic<float>& _m3,
        std::atomic<float>& _q1, std::atomic<float>& _q2, std::atomic<float>& _q3);
    void m2q();
    void q2m(std::atomic<float>& _q1, std::atomic<float>& _q2, std::atomic<float>& _q3,
        std::atomic<float>& _m1, std::atomic<float>& _m2, std::atomic<float>& _m3);
    void q2m();

    void enable();
    void disable();

    // Gripper and wrist (Dynamixel)
    void open();
    void close();
    void moveWristIncr(int dir);

    // Kinematics. IK returns non-zero if the pose is unreachable.
    int IK(float _x, float _y, float _th, float& _q1, float& _q2, float& _q3);
    int IK();
    void DK(float _q1, float _q2, float _q3, float& _x, float& _y, float& _th);
    void DK();

    void getQs(float& _q1, float& _q2, float& _q3);
    void getCoord(float& _x, float& _y, float& _th);
    void getMqs(float& _m1, float& _m2, float& _m3);

    void moveCoord(float _x, float _y, float _th);
    void moveCoordIncr(float dx, float dy, float dth);
    void moveQs_incr(float dq1, float dq2, float dq3);

    void setOffset();

    // Ramp the motor setpoints to (t1, t2, t3) in steps of `incr` rad every
    // 20 ms. Blocks until every joint has reached its target, then resyncs
    // the joint-space (q) and Cartesian (x, y, th) state.
    void rampMotorsTo(float t1, float t2, float t3, float incr);

    void home();
    void getUp();

};

#endif //ARM_H
