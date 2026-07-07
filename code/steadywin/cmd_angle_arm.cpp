#include "socketcan.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "GIM6010.h"
#include "steadywin.h"
#include <stdio.h>
#include <cstdlib>

//float angle = 0.0f;

std::mutex mtx;

float m1pos;

struct Motors
{
    GIM6010   *m1;
    Steadywin *m2;
    Steadywin *m3;
};

struct Angles
{
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
};

float l1 = 0.4f;
float l2 = 0.33f;
float l3 = 0.1f;

float x, y;
float theta = 0.0f;
float q1, q2, q3;
float mq1, mq2, mq3;

int IK(float p_x, float p_y, float th, float &q_1, float &q_2, float &q_3)
{
    float w_x = p_x - l3 * cos(th);
    float w_y = p_y - l3 * sin(th);

    float wl = sqrt(w_x*w_x + w_y*w_y);
    //if(wl >= (l1+l2+l3 - MARGIN)) return -1;

    q_2 = acos((l1*l1 + l2*l2 - w_x*w_x - w_y*w_y)/(2.0f * l1 * l2));
    q_1 = atan2(w_y, w_x) + asin((l2 * sin(q_2))/sqrt(w_x*w_x + w_y*w_y));
    
    float alpha = q_1;

    q_3 = M_PI - alpha + th;

    int err = 0; //test_qs(q_1, q_2, q_3);
    if(std::isnan(q_1) || std::isnan(q_2) || std::isnan(q_3)) return -1;
    return err;

    q_1 = q_1*180.0f/M_PI;
    q_2 = q_2*180.0f/M_PI;
    q_3 = q_3*180.0f/M_PI;
}

int DK(float q_1, float q_2, float q_3, float &p_x, float &p_y, float &th)
{
    int err = 0;//test_qs(q_1, q_2, q_3);
    //if(err == -1) return -1;
    float alpha = q_1;
    p_x = l1 * cos(q_1) + l2 * cos(q_1 + q_2 - M_PI) + l3 * cos(q_1 + q_3 - M_PI);
    p_y = l1 * sin(q_1) + l2 * sin(q_1 + q_2 - M_PI) + l3 * sin(q_1 + q_3 - M_PI);
    th = q_3 - M_PI + alpha;
    return 0;
}

void send_position_deg_smooth(Motors& m, Angles& ang, float& step_size)
{
    const int delta_time = 10; // Time in milliseconds
    float current_angle2 = m.m2->get_position_deg(); // Get the current position in degrees
    float current_angle3 = m.m3->get_position_deg(); // Get the current position in degrees
    float step = step_size * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached

        if(ang.q2 > current_angle2) {
            current_angle2 += step; // Increment the angle
        } else if(ang.q2 < current_angle2){
            current_angle2 -= step; // Decrement the angle
        }

        if(ang.q3 > current_angle3) {
            current_angle3 += step; // Increment the angle
        } else if(ang.q3 < current_angle3){
            current_angle3 -= step; // Decrement the angle
        }

        float rads = ang.q1 * M_PI / 180.0f;

        m.m1->pos_ctrl_red_rad(rads);
        m.m2->pos_control_deg(current_angle2);
        m.m3->pos_control_deg(current_angle3);

        float n;
        m.m1->getEncoderEstimates_deg(mq1, n);
        mq2 = m.m2->get_position_deg();
        mq3 = m.m3->get_position_deg();


        std::this_thread::sleep_for(std::chrono::milliseconds(delta_time));
        //current_angle = m.get_position_deg(); // Get the current position in degrees
    }
}


void send_position_deg(GIM6010& m3, float& ang)
{
    while (true) {
        //mtx.lock();
        float rads = ang * M_PI / 180.0f;
        float deg_vel = 90.0f;
        //send_position_deg_smooth(m1, ang, deg_vel);
        //send_position_deg_smooth(m2, ang, deg_vel);
        m3.pos_ctrl_red_rad(rads);
        //mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


int main() {
    SocketCAN can("can0");

    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }

    can.setNonBlocking(true);

    GIM6010 m1(&can, 11);
    Steadywin m2(&can, 2, GIM4310_36);
    Steadywin m3(&can, 1, GIM3505_8);

    Motors motors;
    Angles angls;

    motors.m1 = &m1;
    motors.m2 = &m2;
    motors.m3 = &m3;
    

    motors.m1->start_motor();
    motors.m2->start_motor();
    motors.m3->start_motor();
    motors.m1->setTrapezodialMode();

    float angle = 0.0f;

    //printf("Target Angle: ");
    //scanf("%f", &angle);
        
    float deg_vel = 180.0f;
    float rad_vel = deg_vel*M_PI/180.0f;

    //std::thread pos_thread1(send_position_deg_smooth, std::ref(motors), std::ref(angls), std::ref(deg_vel));

    x = l2 + l3;
    y = l1;
    theta = 0.0f;

    IK(x, y, theta, q1, q2, q3);

    std::cout << "x: " << x << ", y: " << y << ", th: " << theta
        << ", q1: " << q1 << ", q2: " << q2 << ", q3: " << q3 << "\n";   

    /*
    while (true) {
        std::cout << "mq1: " << mq1 << ", mq2: " << mq2 << ", mq3: " << mq3 << "\n"; 

        scanf("%f", &angle);
        if(angle == 1.0f) break;
        angls.q1 = angle;
        //m1.start_motor();
        //m1.pos_control_deg(angle);
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //m1.stop_motor();
    }
        */

    //pos_thread1.join();
    //pos_thread2.join();
    //pos_thread3.join();
    motors.m1->stop_motor();
    motors.m2->stop_motor();
    motors.m3->stop_motor();

    return 0;

}
