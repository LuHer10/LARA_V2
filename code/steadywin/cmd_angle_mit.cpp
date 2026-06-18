#include "socketcan.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "GIM6010.h"
#include <stdio.h>

//float angle = 0.0f;

std::mutex mtx;

void send_position_deg(GIM6010& m, float& ang)
{
    while (true) {
        //mtx.lock();
        float rads = ang * M_PI / 180.0f;
        m.MITControl(rads, 5.0f, 0.0f, 15.0f, 1.2f);
        //mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
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


    m1.start_motor();
    m1.setMITMode();

    float angle = 0.0f;

    //printf("Target Angle: ");
    //scanf("%f", &angle);
        
    float deg_vel = 90.0f;
    float rad_vel = deg_vel*M_PI/180.0f;
    std::thread pos_thread(send_position_deg, std::ref(m1), std::ref(angle));
    //std::thread pos_thread(send_position_deg_smooth, std::ref(m1), std::ref(angle), std::ref(deg_vel));
    //std::thread pos_thread(send_position_rad_smooth, std::ref(m1), std::ref(angle), std::ref(rad_vel));
    //std::thread pos_thread(&Steadywin::move_smooth_deg, &m1, std::ref(angle), std::ref(deg_vel));

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
