#include "steadywin.h"

int Steadywin::reset_config()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RESET_CONFIG;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }
    
    return  retval;    
}

int32_t Steadywin::get_gear_ratio()
{
    
    retr_config(GEAR_RATIO, gear_ratio);
    std::cout << "Gear ratio is: " << gear_ratio << ":1\n";
    return gear_ratio;
}

int Steadywin::get_fault()
{
    
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = GET_FAULT;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;
    int faul;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    faul = recv_frame.data[2]; 

    std::cout << "Fault no.: " << faul << "\n";
    
    return faul;    
}

float Steadywin::get_torq_const()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_CONFIG;
    frame.data[1] = 0x01;
    frame.data[2] = 0x03;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[3];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    fl32u8 res;
    res.u8[0] = recv_frame.data[4];
    res.u8[1] = recv_frame.data[5];
    res.u8[2] = recv_frame.data[6];
    res.u8[3] = recv_frame.data[7];

    torq_const = res.fl;
    std::cout << "Torque constant is: " << torq_const << " N.m/A\n";
    
    return torq_const;    
}

int Steadywin::mod_config(int confid, int32_t data)
{
    i32u8 _data;
    _data.i32 = data;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = MODIFY_CONFIG;
    frame.data[1] = 0x00;
    frame.data[2] = confid;
    frame.data[3] = 0x00;
    frame.data[4] = _data.u8[0];
    frame.data[5] = _data.u8[1];
    frame.data[6] = _data.u8[2];
    frame.data[7] = _data.u8[3];

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[3];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }
    
    return  retval;
}

int Steadywin::retr_config(uint8_t confid, int32_t& data)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_CONFIG;
    frame.data[1] = 0x00;
    frame.data[2] = confid;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[3];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    data = ((int32_t)recv_frame.data[7] << 24)
            + ((int32_t)recv_frame.data[6] << 16)
            + ((int32_t)recv_frame.data[5] << 8)
            + ((int32_t)recv_frame.data[4]);

    
    return  retval;
    
}

int Steadywin::start_motor()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = START_MOTOR;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    
    return  retval;
}

int Steadywin::stop_motor()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = STOP_MOTOR;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    
    return  retval;

}

int Steadywin::speed_control(float _speed, uint32_t duration)
{

    fl32u8 rpms;
    rpms.fl = _speed;

    u32u8 dur;
    dur.u32  = duration;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = SPEED_CONTROL;
    frame.data[1] = rpms.u8[0];
    frame.data[2] = rpms.u8[1];
    frame.data[3] = rpms.u8[2];
    frame.data[4] = rpms.u8[3];
    frame.data[5] = dur.u8[0];
    frame.data[6] = dur.u8[1];
    frame.data[7] = dur.u8[2];

    

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    temp = recv_frame.data[2];

    int32_t pos_int = recv_frame.data[3]
                    + (recv_frame.data[4] << 8);
    pos = pos_int * 25.0f / 65535.0f - 12.5f;

    int32_t speed_int = (recv_frame.data[5] << 4)
                    +  ((recv_frame.data[6] & 0xf0) >> 4);
    speed = speed_int * 130.0f / 4095.0f - 65.0f;

    int32_t torque_int = recv_frame.data[7]
                    + ((recv_frame.data[6] & 0x0f) << 8);
    torque = torque_int * (450.0f * torq_const * gear_ratio)/4095.0f 
                - 225.0f * torq_const * gear_ratio;

    
    return  retval;
    
}

int Steadywin::pos_control(float _pos, uint32_t duration)
{

    fl32u8 ang;
    ang.fl = _pos;

    u32u8 dur;
    dur.u32  = duration;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = POSITION_CONTROL;
    frame.data[1] = ang.u8[0];
    frame.data[2] = ang.u8[1];
    frame.data[3] = ang.u8[2];
    frame.data[4] = ang.u8[3];
    frame.data[5] = dur.u8[0];
    frame.data[6] = dur.u8[1];
    frame.data[7] = dur.u8[2];

    if (can->sendFrame(frame)) {
        std::cout << "Frame sent\n";
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            std::cout << "Return value: " << retval << "\n";
            break;
        }
    }

    temp = recv_frame.data[2];

    int32_t pos_int = recv_frame.data[3]
                    + (recv_frame.data[4] << 8);
    pos = pos_int * 25.0f / 65535.0f - 12.5f;
    std::cout << "pos_int: " << pos_int << "\n";

    int32_t speed_int = (recv_frame.data[5] << 4)
                    +  ((recv_frame.data[6] & 0xf0) >> 4);
    speed = speed_int * 130.0f / 4095.0f - 65.0f;

    int32_t torque_int = recv_frame.data[7]
                    + ((recv_frame.data[6] & 0x0f) << 8);
    torque = torque_int * (450.0f * torq_const * gear_ratio)/4095.0f 
                - 225.0f * torq_const * gear_ratio;

    
    return  retval;
    
}

