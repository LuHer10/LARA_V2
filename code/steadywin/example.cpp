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

    //SocketCAN *pcan = &can;

    float rpms = 120.0f;
    uint8_t* rpms_int;
    rpms_int = (uint8_t*)(&rpms);
    
    
    //fl32u8 rpms;
    //rpms.fl =  0.0f; 

    //std::memcpy(rpms_int, &rpms);

    float position;
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
    
/*
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
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        pos.fl += 3.1415926f/8.0f;

    pos.u8[0] = recv_frame.data[4];
    pos.u8[1] = recv_frame.data[5];
    pos.u8[2] = recv_frame.data[6];
    pos.u8[3] = recv_frame.data[7];
    position = pos.fl;
    std::cout << "Position: " << position << "\n";

    
    int32_t result;
    result = ((int32_t)recv_frame.data[7] << 24)
            + ((int32_t)recv_frame.data[6] << 16)
            + ((int32_t)recv_frame.data[5] << 8)
            + ((int32_t)recv_frame.data[4]);
    std::cout << "Result: " << (int)result << "\n";

    }

    pos.u8[0] = recv_frame.data[4];
    pos.u8[1] = recv_frame.data[5];
    pos.u8[2] = recv_frame.data[6];
    pos.u8[3] = recv_frame.data[7];
    position = pos.fl;
    std::cout << "Position: " << position << "\n";

    
    int32_t result;
    result = ((int32_t)recv_frame.data[7] << 24)
            + ((int32_t)recv_frame.data[6] << 16)
            + ((int32_t)recv_frame.data[5] << 8)
            + ((int32_t)recv_frame.data[4]);
    std::cout << "Result: " << (int)result << "\n";
    */
    
    /*
    int32_t data;// = 10;
    int retval;

    Steadywin m1(&can, 0x01);

    //retval = m1.mod_config(MAX_SPEED, 250);
    //printf("%d\n", data);
    //retval = m1.retr_config(0x17, data);
    //printf("%d\n", data);
    
    //m1.reset_config();

    float angle = 0.0f;

    m1.start_motor();
    //m1.speed_control(0.0f, 100);
    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    for(int i = 0; i < 16; i++){
        m1.pos_control_deg(angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        angle += 22.5f;
    }
    //m1.stop_motor();
    //m1.pos_control_deg(angle);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    //m1.start_motor();    
    for(int i = 0; i < 16; i++){
        m1.pos_control_deg(angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        angle -= 22.5f;
    }
    m1.stop_motor();

    
    //m1.pos_control_deg(0.0f);
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //m1.pos_control(0.0f, 0);
    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    //m1.pos_control(0.0f, 0);
    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    //float pos = m1.get_position();
    //std::cout << "Position: " << pos << "\n";
    //for(int i = 0x00; i < 0x19; i++){
    //m1.retr_config(0x18, data);
    //std::cout << "CAN_BAUD_RATE: " << data << "\n";//}
    //m1.get_gear_ratio();
    //m1.get_torq_const();
    //m1.stop_control();
    //m1.get_fault();
    //retval = m1.stop_motor();
    //printf("%d\n", retval);
   */

    return 0;
}

