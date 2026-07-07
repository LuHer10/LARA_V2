#include "socketcan.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "arm.h"
#include <stdio.h>
#include <cstdlib>

//float angle = 0.0f;


int main() {
    SocketCAN can("can0");

    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }

    can.setNonBlocking(true);

    Arm arm(&can, 0.4f, 0.33f, 0.033f);

    std::thread pos_thread1(&Arm::writePosTh, &arm);

    char command;

    while (true) {
        printf("WASD: ");
        scanf("%c", &command);
        if(command == 'd')
            arm.moveIncr(0.01f, 0.0f, 0.0f);
        if(command == 'a')
            arm.moveIncr(0.01f, 0.0f, 0.0f);
        if(command == 'x')
        {
            arm.stop();
            break;
        }
        //m1.start_motor();
        //m1.pos_control_deg(angle);
        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //m1.stop_motor();
    }

    arm.stop();
    

    return 0;

}
