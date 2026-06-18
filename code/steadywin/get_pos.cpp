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

    float pos, vel;

    GIM6010 m1(&can, 11);
    
    while (true) {
        m1.getEncoderEstimates(pos, vel);
        std::cout << "pos: " << pos << " vel: " << vel << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    
    return 0;
}

