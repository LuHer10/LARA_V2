#include "socketcan.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "steadywin.h"
#include <stdio.h>

//float angle = 0.0f;

void send_position_deg(Steadywin& m, float& ang)
{
    while (true) {
        m.pos_control_deg(ang);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void send_position_deg_smooth(Steadywin& m, float& ang, float step_size/* deg/s */)
{
    const int delta_time = 100; // Time in milliseconds
    float current_angle = m.get_position_deg(); // Get the current position in degrees
    float step = step_size * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached
        if(abs(ang - current_angle) > 1*step)
        {
            if(ang > current_angle) {
                current_angle += step; // Increment the angle
            } else {
                current_angle -= step; // Decrement the angle
            }
            m.pos_control_deg(current_angle);
        }
        else {
            m.pos_control_deg(ang); // Set to target angle if close enough
            current_angle = m.get_position_deg();
        }        
        std::this_thread::sleep_for(std::chrono::milliseconds(delta_time));
        //current_angle = m.get_position_deg(); // Get the current position in degrees
    }
}

void send_position_rad_smooth(Steadywin& m, float& ang, float step_size/* rad/s */)
{
    float degs = ang*180.0f/3.1415926f;
    float rad_steps = step_size*180.0f/3.1415926f;

    const int delta_time = 100; // Time in milliseconds
    float current_angle = m.get_position_deg(); // Get the current position in degrees
    float step = rad_steps * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached
        degs = ang*180.0f/3.1415926f;
        if(abs(degs - current_angle) > 1*step)
        {
            if(degs > current_angle) {
                current_angle += step; // Increment the angle
            } else {
                current_angle -= step; // Decrement the angle
            }
            m.pos_control_deg(current_angle);
        }
        else {
            m.pos_control_deg(degs); // Set to target angle if close enough
            current_angle = m.get_position_deg();
        }        
        std::this_thread::sleep_for(std::chrono::milliseconds(delta_time));
        //current_angle = m.get_position_deg(); // Get the current position in degrees
    }
}



int main() {
    SocketCAN can("can0");

    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }

    can.setNonBlocking(true);

    Steadywin m1(&can, 0x01);


    m1.start_motor();
    float angle = 0.0f;

    printf("Target Angle: ");
    scanf("%f", &angle);
        
    //std::thread pos_thread(send_position, std::ref(m1), std::ref(angle));
    std::thread pos_thread(send_position_deg_smooth, std::ref(m1), std::ref(angle), 90.0f);
    //std::thread pos_thread(send_position_rad_smooth, std::ref(m1), std::ref(angle), 3.1415926f*0.5f);

    while (true) {
        printf("Target Angle: ");
        scanf("%f", &angle);
        //m1.start_motor();
        //m1.pos_control_deg(angle);
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //m1.stop_motor();
    }

    pos_thread.join();
    m1.stop_motor();

    return 0;

}
