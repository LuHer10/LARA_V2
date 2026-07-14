#ifndef ARM_H
#define ARM_H

#include "GIM6010.h"
#include "steadywin.h"
#include <thread>

class Arm
{
private:

    SocketCAN* can;

    std::thread *arm_thread;

    GIM6010 *big;
    Steadywin *med;
    Steadywin *small;

    float l1, l2, l3;


    float qBig;
    float qMed;
    float qSmall;

    float mBig;
    float mMed;
    float mSmall;

    
    float theta;
    float px, py;
    float wx, wy;

    void writePos(float _mBig, float _mMed, float _mSmall);
    void q2m(float _qBig, float _qMed, float _qSmall, float& _mBig, float& _mMed, float& _mSmall)
    {
        _mBig = _qBig - M_PI/2.0f;
        _mMed = _qMed;
        _mSmall = _qSmall;
    }
    void m2q(float _mBig, float _mMed, float _mSmall, float& _qBig, float& _qMed, float& _qSmall)
    {
        _qBig = _mBig + M_PI/2.0f;
        _qMed = _mMed;
        _qSmall = _mSmall;
    }
    void q2m()
    {
        mBig = qBig - M_PI/2.0f;
        mMed = qMed;
        mSmall = qSmall;
    }
    void m2q()
    {
        qBig = mBig + M_PI/2.0f;
        qMed = mMed;
        qSmall = mSmall;
    }
    

public:

    Arm(SocketCAN* _can, float _l1, float _l2, float _l3):can(_can)
    {
        /*
        GIM6010 m1(can, 11);
        Steadywin m2(can, 2, GIM4310_36);
        Steadywin m3(can, 1, GIM3505_8);

        big = &m1;
        med = &m2;
        small = &m3;
        */

        big = new GIM6010(can, 11);
        med = new Steadywin(can, 2, GIM4310_36);
        small = new Steadywin(can, 1, GIM3505_8);

        small->start_motor();
        med->start_motor();
        big->start_motor();
        big->setTrapezodialMode();

        l1 = _l1;
        l2 = _l2;
        l3 = _l3;

        qBig = M_PI/2.0f;
        qMed = M_PI/2.0f;
        qSmall = M_PI/2.0f;
        
        q2m();

        DK(qBig, qMed, qSmall, px, py, theta);

    }

    ~Arm()
    {
        small->stop_motor();
        med->stop_motor();
        big->stop_motor();
    }

    void writePosTh();

    void stop()
    {
        small->stop_motor();
        med->stop_motor();
        big->stop_motor();
    }

    void setP(float p_x, float p_y)
    {
        px = p_x;
        py = p_y;
    }
    void setTheta(float th){theta = th;}
    float getPx(){return px;}
    float getPy(){return py;}
    float getTheta(){return theta;}

    //void get_qs(float &q_1, float &q_2, float &q_3){q_1 = q1; q_2 = q2; q_3 = q3;}
    void get_pos(float &p_x, float &p_y, float &th){p_x = px; p_y = py; th = theta;}

    int DK(float q_1, float q_2, float q_3, float &p_x, float &p_y, float &th);
    int IK(float p_x, float p_y, float th, float &q_1, float &q_2, float &q_3);

    int move(float p_x, float p_y, float th);
    int moveIncr(float dx, float dy, float dth);

    int move_qs(float q_1, float q_2, float q_3);

    void updateQs();


};

#endif