#ifndef ARM_H
#define ARM_H

#include "socketcan.h"
#include "GIM6010.h"
#include "steadywin.h"
#include <cmath>
#include <thread>
#include <iostream>
#include <atomic>

#define BIG_ID 11
#define MEDIUM_ID 2
#define SMALL_ID 1

class Arm
{
private:
    float l1, l2, l3;

    std::atomic<float> q1, q2, q3;
    std::atomic<float> x, y, th;

    std::atomic<float> mq1, mq2, mq3;
    std::atomic<float> mx, my, mth;

    SocketCAN* can;

    GIM6010* bigMotor;
    Steadywin* mediumMotor;
    Steadywin* smallMotor;

    std::thread controlThread;
    bool stopThread = false;

    void motorControl()
    {
        std::cout << "Motor control active\n";
        float _mx, _my, _mth;
        while(true)
        {
            if(stopThread) break;
            bigMotor->pos_ctrl_red_rad(q1.load() - (M_PI/2.0f));
            mediumMotor->pos_control_rad(q2.load());
            smallMotor->pos_control_rad(q3.load());   
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
            mq1.store(bigMotor->getPositionRad());
            mq2.store(mediumMotor->get_position_rad());
            mq3.store(smallMotor->get_position_rad());
            DK(mq1.load() + (M_PI/2.0f), mq2.load(), mq3.load(), _mx, _my, _mth);
            mx.store(_mx);
            my.store(_my);
            mth.store(_mth);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }
        std::cout << "Motor control inactive\n";

    }

public:
    Arm(SocketCAN* _can, float _l1, float _l2, float _l3):can(_can)
    {
        bigMotor = new GIM6010(can, BIG_ID);
        mediumMotor = new Steadywin(can, MEDIUM_ID, GIM4310_36);
        smallMotor = new Steadywin(can, SMALL_ID, GIM3505_8);

        bigMotor->start_motor();
        mediumMotor->start_motor();
        smallMotor->start_motor();
        bigMotor->setTrapezodialMode();

        l1 = _l1;
        l2 = _l2;
        l3 = _l3;

        x.store(l2 + l3);
        y.store(l1);
        th.store(0.0f);

        IK();

        controlThread = std::thread(&Arm::motorControl, this);

    }

    ~Arm()
    {
        stopThread = true;
        controlThread.join();
        //bigMotor->stop_motor();
        mediumMotor->stop_motor();
        smallMotor->stop_motor();
    }


    int IK(float _x, float _y, float _th, float& _q1, float& _q2, float& _q3)
    {
        float wx = _x - l3*cos(_th);
        float wy = _y - l3*sin(_th);
        float wl = sqrt(wx*wx + wy*wy);

        float q_2 = acos((l1*l1 + l2*l2 - wx*wx - wy*wy)/(2.0f * l1 * l2));
        float q_1 = atan2(wy, wx) + asin((l2 * sin(q_2))/sqrt(wx*wx + wy*wy));

        if(isnan(q_1) || isnan(q_2)) return 1;

        _q2 = q_2;
        _q1 = q_1;
        float alpha = _q1 + _q2 - M_PI;

        _q3 = M_PI - alpha + th;

        return 0;
    }

    void IK()
    {
        float _q1, _q2, _q3;
        if(IK(x.load(), y.load(), th.load(), _q1, _q2, _q3))return;
        q1.store(_q1);
        q2.store(_q2);
        q3.store(_q3);
    }

    void DK(float _q1, float _q2, float _q3, float& _x, float& _y, float& _th)
    {
        float alpha = _q1 + _q2 - M_PI;
        _x = l1 * cos(_q1) + l2 * cos(_q1 + _q2 - M_PI) + l3 * cos(_q1 + _q2 + _q3 - 2.0f*M_PI);
        _y = l1 * sin(_q1) + l2 * sin(_q1 + _q2 - M_PI) + l3 * sin(_q1 + _q2 + _q3 - 2.0f*M_PI);
        _th = _q3 - M_PI + alpha;
    }

    void DK()
    {
        float _x, _y, _th;
        DK(q1.load(), q2.load(), q3.load(), _x, _y, _th);
        x.store(_x);
        y.store(_y);
        th.store(_th);
    }

    void getQs(float& _q1, float& _q2, float& _q3)
    {
        _q1 = q1.load();
        _q2 = q2.load();
        _q3 = q3.load();
    }

    void getCoord(float& _x, float& _y, float& _th)
    {
        _x = x.load();
        _y = y.load();
        _th = th.load();
    }

    void getMqs(float& _mq1, float& _mq2, float& _mq3)
    {
        _mq1 = mq1.load();
        _mq2 = mq2.load();
        _mq3 = mq3.load();
    }

    void getMCoord(float& _mx, float& _my, float& _mth)
    {
        _mx = mx.load();
        _my = my.load();
        _mth = mth.load();
    }

    void moveCoord(float _x, float _y, float _th)
    {
        x.store(_x);
        y.store(_y);
        th.store(_th);
        IK();
    }
    
    void moveCoordIncr(float dx, float dy, float dth)
    {
        x.store(x.load() + dx);
        y.store(y.load() + dy);
        th.store(th.load() + dth);

        //x += dx;
        //y += dy;
        //th += dth;
        IK();
    }
    
};

#endif //ARM_H