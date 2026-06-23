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

void send_position_deg_smooth(Steadywin& m, float& ang, float& step_size)
{
    const int delta_time = 10; // Time in milliseconds
    mtx.lock();
    float current_angle = m.get_position_deg(); // Get the current position in degrees
    mtx.unlock();
    float step = step_size * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached
        //if(abs(ang - current_angle) > 1*step)
        //{
            if(ang > current_angle) {
                current_angle += step; // Increment the angle
            } else if(ang < current_angle){
                current_angle -= step; // Decrement the angle
            }
            mtx.lock();
            m.pos_control_deg(current_angle);
            mtx.unlock();
        //}

        std::this_thread::sleep_for(std::chrono::milliseconds(delta_time));
        //current_angle = m.get_position_deg(); // Get the current position in degrees
    }
}

void send_position_rad_smooth(Steadywin& m, float& ang, float& step_size)
{
    float degs = ang*180.0f/M_PI;
    float rad_steps = step_size*180.0f/M_PI;

    const int delta_time = 100; // Time in milliseconds
    mtx.lock();
    float current_angle = m.get_position_deg(); // Get the current position in degrees
    mtx.unlock();
    float step = rad_steps * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached
        degs = ang*180.0f/M_PI;
        rad_steps = step_size*180.0f/M_PI;
        if(abs(degs - current_angle) > 1*step)
        {
            if(degs > current_angle) {
                current_angle += step; // Increment the angle
            } else {
                current_angle -= step; // Decrement the angle
            }
            mtx.lock();
            m.pos_control_deg(current_angle);
            mtx.unlock();
        }
        else {
            mtx.lock();
            m.pos_control_deg(degs); // Set to target angle if close enough
            current_angle = m.get_position_deg();
            mtx.unlock();
        }        
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
    

    m1.start_motor();
    m2.start_motor();
    m3.start_motor();
    m3.setTrapezodialMode();

    float angle = 0.0f;

    //printf("Target Angle: ");
    //scanf("%f", &angle);
        
    float deg_vel = 180.0f;
    float rad_vel = deg_vel*M_PI/180.0f;
    //std::thread pos_thread(send_position_deg, std::ref(m1), std::ref(angle));
    //std::thread pos_thread3(send_position_deg, std::ref(m3), std::ref(angle));
    std::thread pos_thread1(send_position_deg_smooth, std::ref(m1), std::ref(angle), std::ref(deg_vel));
    std::thread pos_thread2(send_position_deg_smooth, std::ref(m2), std::ref(angle), std::ref(deg_vel));
    //std::thread pos_thread(send_position_rad_smooth, std::ref(m1), std::ref(angle), std::ref(rad_vel));
    //std::thread pos_thread(&Steadywin::move_smooth_deg, &m1, std::ref(angle), std::ref(deg_vel));

    //std::system("get_pos");

    while (true) {
        printf("Target Angle: ");
        scanf("%f", &angle);
        if(angle == 2305) break;
        //m1.start_motor();
        //m1.pos_control_deg(angle);
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //m1.stop_motor();
    }

    //pos_thread1.join();
    //pos_thread2.join();
    //pos_thread3.join();
    m1.stop_motor();
    m2.stop_motor();
    m3.stop_motor();

    return 0;

}
