#include "socketcan.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "GIM6010.h"
//#include "steadywin.h"
#include <stdio.h>

int main() {
    SocketCAN can("can0");

    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }

    can.setNonBlocking(true);


    /*    
    fl32u8 pos;
    pos.fl = 0.0f;
    struct can_frame recv_frame;
    while (true) {
        struct can_frame frame;
        frame.can_id = (11 << 5) | 0x00C;
        frame.can_dlc = 8;
        frame.data[0] = pos.u8[0];
        frame.data[1] = pos.u8[1];
        frame.data[2] = pos.u8[2];
        frame.data[3] = pos.u8[3];
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        if (can.sendFrame(frame)) {
            std::cout << "Frame sent\n";
            break;
        }
    }
    */
    

    /* */
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
    /*
    GIM6010 m1(&can, 11);
    m1.start_motor();
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));

    m1.setMITMode();
    //std::this_thread::sleep_for(std::chrono::milliseconds(10));

    m1.MITControl(0.0f, 10.0f, 0.0f, 20.0f, 1.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    m1.MITControl(100.0f, 10.0f, 0.0f, 20.0f, 1.0f);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
    m1.stop_motor();
    */
    return 0;
}

