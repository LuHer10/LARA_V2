#include "socketcan.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdint.h>
#include <string.h>
#include "steadywin.h"
#include <stdio.h>

int main() {
    SocketCAN can("can0");

    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }

    can.setNonBlocking(true);

    SocketCAN *pcan = &can;

    /*float rpms = 120.0f;
    uint8_t* rpms_int;
    rpms_int = (uint8_t*)(&rpms);
    */
    
    //fl32u8 rpms;
    //rpms.fl =  0.0f; 

    //std::memcpy(rpms_int, &rpms);

    /*
    struct can_frame frame;
    frame.can_id = 0x02;
    frame.can_dlc = 8;
    frame.data[0] = 0x84;
    frame.data[1] = 0x00;
    frame.data[2] = 0x17;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can.sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;

    while (true) {
        if (can.receiveFrame(recv_frame)) {
            std::cout << "Received ID: " << std::hex << recv_frame.can_id << "\n";
            break;
        } else {
            // No data → do other work
            //std::cout << "Error\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    int32_t result;
    result = ((int32_t)recv_frame.data[7] << 24)
            + ((int32_t)recv_frame.data[6] << 16)
            + ((int32_t)recv_frame.data[5] << 8)
            + ((int32_t)recv_frame.data[4]);
    std::cout << "Result: " << (int)result << "\n";
    
    */
    
    int32_t data;// = 10;
    int retval;

    Steadywin m1(&can, 0x01);

    //retval = m1.mod_config(0x02, 384);
    //printf("%d\n", data);
    //retval = m1.retr_config(0x17, data);
    //printf("%d\n", data);
    
    m1.start_motor();
    //m1.speed_control(0.0f, 0);
    m1.pos_control(1.0f, 10000);
    m1.get_gear_ratio();
    m1.get_torq_const();
    m1.get_fault();
    retval = m1.stop_motor();
    //printf("%d\n", retval);
   

    return 0;
}

