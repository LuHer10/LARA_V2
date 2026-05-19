#include "socketcan.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "steadywin.h"
#include <stdio.h>

//float angle = 0.0f;

void send_position(Steadywin& m, float& ang)
{
    while (true) {
        m.pos_control_deg(ang);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
        
    std::thread pos_thread(send_position, std::ref(m1), std::ref(angle));

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
