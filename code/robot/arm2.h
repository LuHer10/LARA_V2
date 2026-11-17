#ifndef ARM_H
#define ARM_H

#include "socketcan.h"
#include "GIM6010.h"
#include "steadywin.h"
#include <cmath>
#include <thread>
#include <iostream>
#include <atomic>

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

float deg2rad(float _deg)
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
    bool stopThread = false;

    std::atomic<float> m1_offset = 0.0f;
    std::atomic<float> m2_offset = 0.0f;
    std::atomic<float> m3_offset = 0.0f;

    std::atomic<float> m1_start, m2_start, m3_start;

    void motorControl()
    {
        std::cout << "Motor control active\n";
        float _mx, _my, _mth;
        while(true)
        {
            if(stopThread) break;
            
            bigMotor->pos_ctrl_red_rad(m1.load());
            mediumMotor->pos_control_rad((m2.load()));
            smallMotor->pos_control_rad(m3.load());   
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            mBig.store(bigMotor->getPositionRad());
            mMed.store(mediumMotor->get_position_rad());
            mSmall.store(smallMotor->get_position_rad());
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

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        bigMotor->setTrapezodialMode();

        l1 = _l1;
        l2 = _l2;
        l3 = _l3;

        m1.store(bigMotor->getPositionRad());
        m2.store(mediumMotor->get_position_rad());
        m3.store(smallMotor->get_position_rad());

        m1_start.store(m1.load()); //->180
        m2_start.store(m2.load()); //->  0
        m3_start.store(m3.load()); //->360
        /*
            180 = m1_start + m1_offset
            q1 = m1 + m1_offset
        */

        m1_offset.store(M1_REST - m1_start.load());
        m2_offset.store(M2_REST - m2_start.load());
        m3_offset.store(M3_REST - m3_start.load());

        //q1.store(m1.load()+m1_offset.load());
        //q2.store(m2.load()+m2_offset.load());
        //q3.store(m3.load()+m3_offset.load());

        m2q(m1, m2, m3, q1, q2, q3);

        DK();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        controlThread = std::thread(&Arm::motorControl, this);

        servos.init();
        servos.enableTorque(GRIP_ID);

    }

    ~Arm()
    {
        stopThread = true;
        controlThread.join();
        bigMotor->stop_motor();
        mediumMotor->stop_motor();
        smallMotor->stop_motor();
    }

    void m2q(float _m1, float _m2, float _m3, 
        float& _q1, float& _q2, float& _q3)
    {
        _q1 = m1_offset.load() - _m1;  //ok
        _q2 = _m2 + m2_offset.load();  //ok
        _q3 = _m3 + m3_offset.load();
    }

    void q2m(float _q1, float _q2, float _q3, 
        float& _m1, float& _m2, float& _m3)
    {
        _m1 = m1_offset.load() - _q1; //ok
        _m2 = _q2 - m2_offset.load(); //ok
        _m3 = _q3 - m3_offset.load();
    }

    void m2q(std::atomic<float>& _m1, std::atomic<float>& _m2, std::atomic<float>& _m3, 
        std::atomic<float>& _q1, std::atomic<float>& _q2, std::atomic<float>& _q3)
    {
        float m1a = _m1.load(), m2a = _m2.load(), m3a = _m3.load();
        float q1a, q2a, q3a;
        m2q(m1a, m2a, m3a, q1a, q2a, q3a);
        _q1.store(q1a);
        _q2.store(q2a);
        _q3.store(q3a);
    }
    
    void m2q()
    {
        float m1a = m1.load(), m2a = m2.load(), m3a = m3.load();
        float q1a, q2a, q3a;
        m2q(m1a, m2a, m3a, q1a, q2a, q3a);
        q1.store(q1a);
        q2.store(q2a);
        q3.store(q3a);
    }

    void q2m(std::atomic<float>& _q1, std::atomic<float>& _q2, std::atomic<float>& _q3, 
        std::atomic<float>& _m1, std::atomic<float>& _m2, std::atomic<float>& _m3)
    {
        float q1a = _q1.load(), q2a = _q2.load(), q3a = _q3.load();
        float m1a, m2a, m3a;
        q2m(q1a, q2a, q3a, m1a, m2a, m3a); 
        _m1.store(m1a);
        _m2.store(m2a);
        _m3.store(m3a);
    }
    
    void q2m()
    {
        float q1a = q1.load(), q2a = q2.load(), q3a = q3.load();
        float m1a, m2a, m3a;
        q2m(q1a, q2a, q3a, m1a, m2a, m3a);
        m1.store(m1a);
        m2.store(m2a);
        m3.store(m3a);
    }

    void enable()
    {
        bigMotor->start_motor();
        mediumMotor->start_motor();
        smallMotor->start_motor();
    }

    void disable()
    {
        bigMotor->stop_motor();
        mediumMotor->stop_motor();
        smallMotor->stop_motor();
    }

    void open(){servos.writePos(GRIP_ID, degToPos(230));}
    void close(){servos.writePos(GRIP_ID, degToPos(170));}
    void moveWristIncr(int dir)
    {
        if(dir != 0)
        {
            wristAngle += dir*0.5f;
            servos.writePos(WRIST_ID, degToPos(wristAngle));
        }
    }


    int IK(float _x, float _y, float _th, float& _q1, float& _q2, float& _q3)
    {
        float wx = _x - l3*cos(_th);
        float wy = _y - l3*sin(_th);
        float wl = sqrt(wx*wx + wy*wy);

        float q_2 = acos((l1*l1 + l2*l2 - wx*wx - wy*wy)/(2.0f * l1 * l2));
        float q_1 = atan2(wy, wx) + asin((l2 * sin(q_2))/wl);

        if(isnan(q_1) || isnan(q_2)) return 1;

        _q2 = q_2;
        _q1 = q_1;
        float alpha = _q1 + _q2 - M_PI;

        _q3 = M_PI - alpha + th;

        return 0;
    }

    int IK()
    {
        float _q1, _q2, _q3;
        if(IK(x.load(), y.load(), th.load(), _q1, _q2, _q3))return 1;
        q1.store(_q1);
        q2.store(_q2);
        q3.store(_q3);
        return 0;
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

    void getMqs(float& _m1, float& _m2, float& _m3)
    {
        _m1 = m1.load();
        _m2 = m2.load();
        _m3 = m3.load();
    }

    void moveCoord(float _x, float _y, float _th)
    {
        x.store(_x);
        y.store(_y);
        th.store(_th);
        IK();
        q2m();
    }
    
    void moveCoordIncr(float dx, float dy, float dth)
    {
        float newX = x.load() + dx;
        float newY = y.load() + dy;
        float newTh = th.load() + dth;
        float _q1, _q2, _q3;

        if(IK(newX, newY, newTh, _q1, _q2, _q3)) return;

        x.store(newX);
        y.store(newY);
        th.store(newTh);

        //x += dx;
        //y += dy;
        //th += dth;
        IK();
        q2m();
    }

    void moveQs_incr(float dq1, float dq2, float dq3)
    {
        q1.store(q1.load() + dq1);
        q2.store(q2.load() + dq2);
        q3.store(q3.load() + dq3);
        q2m();
        DK();
    }

    void setOffset()
    {
        //mBigRaw.store(bigMotor->getPositionRad() + (M_PI/2.0f));
        //mMedRaw.store(-mediumMotor->get_position_rad());
        //mSmaRaw.store(smallMotor->get_position_rad());
        //bigOffset.store(mBigRaw.load() - deg2rad(90.0f));
        //mediumOffset.store(mMedRaw.load() - deg2rad(90.0f));
        //smallOffset.store(mSmaRaw.load() - deg2rad(180.0f));
    }

    void home()
    {
        float m1diff = m1.load() - m1_start.load();
        float m2diff = m2.load() - m2_start.load();
        float m3diff = m3.load() - m3_start.load();

        bool condition = (abs(m1diff)>0.1f) && (abs(m2diff)>0.1f) && (abs(m3diff)>0.1f);
        
        float incr = 0.05f;

        while(condition)
        {
            if(m1diff>0) m1.store(m1.load()-incr);
            else if(m1diff<0) m1.store(m1.load()+incr);

            if(m2diff>0) m2.store(m2.load()-incr);
            else if(m2diff<0) m2.store(m2.load()+incr);
            
            if(m3diff>0) m3.store(m3.load()-incr);
            else if(m3diff<0) m3.store(m3.load()+incr);
        
            m1diff = m1.load() - m1_start.load();
            m2diff = m2.load() - m2_start.load();
            m3diff = m3.load() - m3_start.load();

            condition = (abs(m1diff)>0.1f) && (abs(m2diff)>0.1f) && (abs(m3diff)>0.1f);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));

        }
    }

    void getUp()
    {

        float q1_target = M_PI/2.0f;
        float q2_target = M_PI/2.0f;
        float q3_target = M_PI;

        float m1_target, m2_target, m3_target;

        q2m(q1_target, q2_target, q3_target, m1_target, m2_target, m3_target);

        float m1diff = m1.load() - m1_target;
        float m2diff = m2.load() - m2_target;
        float m3diff = m3.load() - m3_target;

        float incr = 0.01f;
        bool condition = (abs(m1diff)>2.0f*incr) && (abs(m2diff)>2.0f*incr) && (abs(m3diff)>2.0f*incr);

        while(condition)
        {
            if(m1diff>0) m1.store(m1.load()-incr);
            else if(m1diff<0) m1.store(m1.load()+incr);

            if(m2diff>0) m2.store(m2.load()-incr);
            else if(m2diff<0) m2.store(m2.load()+incr);
            
            if(m3diff>0) m3.store(m3.load()-incr);
            else if(m3diff<0) m3.store(m3.load()+incr);
        
            m1diff = m1.load() - m1_target;
            m2diff = m2.load() - m2_target;
            m3diff = m3.load() - m3_target;

            condition = (abs(m1diff)>2.0f*incr) && (abs(m2diff)>2.0f*incr) && (abs(m3diff)>2.0f*incr);

            std::this_thread::sleep_for(std::chrono::milliseconds(20));

        }
    }

    
    
};

#endif //ARM_H