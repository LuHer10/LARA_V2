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

float m3pos;

struct Motors
{
    Steadywin *m1;
    Steadywin *m2;
    GIM6010 *m3;
};

void send_position_deg_smooth(Motors& m, float& ang, float& step_size)
{
    const int delta_time = 10; // Time in milliseconds
    float current_angle1 = m.m1->get_position_deg(); // Get the current position in degrees
    float current_angle2 = m.m2->get_position_deg(); // Get the current position in degrees
    float step = step_size * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached

        if(ang > current_angle1) {
            current_angle1 += step; // Increment the angle
        } else if(ang < current_angle1){
            current_angle1 -= step; // Decrement the angle
        }

        if(ang > current_angle2) {
            current_angle2 += step; // Increment the angle
        } else if(ang < current_angle2){
            current_angle2 -= step; // Decrement the angle
        }

        float rads = ang * M_PI / 180.0f;

        m.m1->pos_control_deg(current_angle1);
        m.m2->pos_control_deg(current_angle2);
        m.m3->pos_ctrl_red_rad(rads);

        float n;
        m.m3->getEncoderEstimates_deg(m3pos, n);


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

    Steadywin m1(&can, 1, GIM3505_8);
    Steadywin m2(&can, 2, GIM4310_36);
    GIM6010 m3(&can, 11);

    Motors motors;
    motors.m1 = &m1;
    motors.m2 = &m2;
    motors.m3 = &m3;
    

    motors.m1->start_motor();
    motors.m2->start_motor();
    motors.m3->start_motor();
    motors.m3->setTrapezodialMode();

    float angle = 0.0f;

    //printf("Target Angle: ");
    //scanf("%f", &angle);
        
    float deg_vel = 180.0f;
    float rad_vel = deg_vel*M_PI/180.0f;

    std::thread pos_thread1(send_position_deg_smooth, std::ref(motors), std::ref(angle), std::ref(deg_vel));

    while (true) {
        printf("Target Angle: ");
        scanf("%f", &angle);
        if(angle == 1.0f) break;
        printf("Read pos: %f \n", m3pos);
        //m1.start_motor();
        //m1.pos_control_deg(angle);
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //m1.stop_motor();
    }

    //pos_thread1.join();
    //pos_thread2.join();
    //pos_thread3.join();
    motors.m1->stop_motor();
    motors.m2->stop_motor();
    motors.m3->stop_motor();

    return 0;

}
