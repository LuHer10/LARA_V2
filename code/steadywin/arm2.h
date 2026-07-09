#ifndef ARM2_H
#define ARM2_H

#include "socketcan.h"
#include "GIM6010.h"
#include "steadywin.h"
#include <cmath>
#include <thread>


class Arm
{
private:
    float l1, l2, l3;

    float q1, q2, q3;
    float x, y, th;

    float mq1, mq2, mq3;

    SocketCAN* can;

    GIM6010* bigMotor;
    Steadywin* mediumMotor;
    Steadywin* smallMotor;

    std::thread controlThread;
    bool stopThread = false;

    void motorControl()
    {
        while(true)
        {
            if(stopThread) break;
            bigMotor->pos_ctrl_red_rad(q1 - (M_PI/2.0f));
            mediumMotor->pos_control_rad(q2);
            smallMotor->pos_control_rad(q3);   
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
            mq1 = bigMotor->getPositionRad();
            mq2 = mediumMotor->get_position_rad();
            mq3 = smallMotor->get_position_rad();
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
        }
    }

public:
    Arm(SocketCAN* _can, float _l1, float _l2, float _l3):can(_can)
    {
        bigMotor = new GIM6010(can, 11);
        mediumMotor = new Steadywin(can, 2, GIM4310_36);
        smallMotor = new Steadywin(can, 1, GIM3505_8);

        bigMotor->start_motor();
        mediumMotor->start_motor();
        smallMotor->start_motor();
        bigMotor->setTrapezodialMode();

        l1 = _l1;
        l2 = _l2;
        l3 = _l3;

        x = l2 + l3;
        y = l1;
        th = 0.0f;

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


    void IK(float _x, float _y, float _th, float& _q1, float& _q2, float& _q3)
    {
        float wx = _x - l3*cos(_th);
        float wy = _y - l3*sin(_th);
        float wl = sqrt(wx*wx + wy*wy);

        float q_2 = acos((l1*l1 + l2*l2 - wx*wx - wy*wy)/(2.0f * l1 * l2));
        float q_1 = atan2(wy, wx) + asin((l2 * sin(q_2))/sqrt(wx*wx + wy*wy));

        if(isnan(q_1) || isnan(q_2)) return;

        _q2 = q_2;
        _q1 = q_1;
        float alpha = _q1 + _q2 - M_PI;

        _q3 = M_PI - alpha + th;
    }

    void IK()
    {
        IK(x, y, th, q1, q2, q3);
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
        DK(q1, q2, q3, x, y, th);
    }

    
}

#endif //ARM2_H