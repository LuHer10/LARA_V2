#include "socketcan.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "GIM6010.h"
//#include "steadywin.h"
#include <stdio.h>

/*
kp kd gains:
{  5.0f, 0.5f };
{ 10.0f, 0.8f };
{ 15.0f, 1.2f };  //equilibrado
{ 25.0f, 1.6f };
{ 35.0f, 1.8f };
*/

int main() {
    SocketCAN can("can0");

    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }

    can.setNonBlocking(true);

    float kp = 15.0f;
    float kd = 1.2f;

    GIM6010 m1(&can, 11);
    m1.start_motor();
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));

    m1.setTrapezodialMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    m1.pos_control_rev(0.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    m1.pos_control_rev(48.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    m1.stop_motor();
    /**/
    
    GIM6010 m1(&can, 11);
    m1.start_motor();
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));

    m1.setMITMode();
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));

    m1.MITControl(0.0f, 10.0f, 0.0f, kp, kd);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    m1.MITControl(100.0f, 10.0f, 0.0f, kp, kd);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    m1.stop_motor();
    
    return 0;
}

