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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }
    
    return  retval;    
}

int Steadywin::refresh_config()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = REFRESH_CONFIG;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            #ifdef DEBUG
            std::cout << "Received frame\n";
            #endif
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    return  retval;
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[3];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[3];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    data = ((int32_t)recv_frame.data[7] << 24)
            + ((int32_t)recv_frame.data[6] << 16)
            + ((int32_t)recv_frame.data[5] << 8)
            + ((int32_t)recv_frame.data[4]);

    
    return  retval;
    
}

int32_t Steadywin::get_gear_ratio()
{
    
    retr_config(GEAR_RATIO, gear_ratio);
    std::cout << "Gear ratio is: " << gear_ratio << ":1\n";
    return gear_ratio;
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[3];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    
    return  retval;

}

int Steadywin::torque_control(float _torque, uint32_t duration)
{

    fl32u8 trq;
    trq.fl = _torque;
    u32u8 dur;
    dur.u32  = duration;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = TORQUE_CONTROL;
    frame.data[1] = trq.u8[0];
    frame.data[2] = trq.u8[1];
    frame.data[3] = trq.u8[2];
    frame.data[4] = trq.u8[3];
    frame.data[5] = dur.u8[0];
    frame.data[6] = dur.u8[1];
    frame.data[7] = dur.u8[2];

    

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
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

int Steadywin::pos_control_rad(float _pos, uint32_t duration)
{

    fl32u8 ang;
    ang.fl = _pos - (79.7f*M_PI/180.0f); 
    u32u8 dur;
    dur.u32  = duration;

    //std::cout << std::hex << (int)ang.u8[0] << " " << (int)ang.u8[1] << " " << (int)ang.u8[2] << " " << (int)ang.u8[3] << "\n";
    //std::cout << (float)ang.fl << "\n";

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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    temp = recv_frame.data[2];

    int32_t pos_int = recv_frame.data[3]
                    + (recv_frame.data[4] << 8);
    pos_int = pos_int - 44276;
    //pos = ((pos_int * 25.0f / 65535.0f) - 12.5f) - 4.38563f;
    pos = pos_int * 25.0f / 65535.0f;
    
    #ifdef DEBUG
    std::cout << "pos_int: " << pos_int << "\n";
    std::cout << "pos (RAD): " << pos << "\n";
    std::cout << "pos (DEG): " << (pos * 180.0f / M_PI) << "\n";
    #endif

    int32_t speed_int = (recv_frame.data[5] << 4)
                    +  ((recv_frame.data[6] & 0xf0) >> 4);
    speed = speed_int * 130.0f / 4095.0f - 65.0f;

    int32_t torque_int = recv_frame.data[7]
                    + ((recv_frame.data[6] & 0x0f) << 8);
    torque = torque_int * (450.0f * torq_const * gear_ratio)/4095.0f 
                - 225.0f * torq_const * gear_ratio;

    
    return  retval;
    
}

int Steadywin::pos_control_deg(float _pos, uint32_t duration)
{

    float ang;
    ang = _pos * M_PI / 180.0f; // Convert degrees to radians

    int retval = pos_control_rad(ang, duration);
    return  retval;
    
}

int Steadywin::stop_control()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = STOP_CONTROL;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    
    return  retval;
}

float Steadywin::get_position_rad()
{
    fl32u8 ang;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_INDICATOR;
    frame.data[1] = ANGLE_OUTPUT_MEC;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    ang.u8[0] = recv_frame.data[4];
    ang.u8[1] = recv_frame.data[5];
    ang.u8[2] = recv_frame.data[6];
    ang.u8[3] = recv_frame.data[7];

    float current_angle = (float)ang.fl;

    current_angle = (current_angle*180.0f/M_PI) + 590040.0f;

    pos_ind_deg = current_angle;
    pos_ind_rad = current_angle * M_PI / 180.0f;


    return pos_ind_rad;
}

float Steadywin::get_position_deg()
{
    fl32u8 ang;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_INDICATOR;
    frame.data[1] = ANGLE_OUTPUT_MEC;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    ang.u8[0] = recv_frame.data[4];
    ang.u8[1] = recv_frame.data[5];
    ang.u8[2] = recv_frame.data[6];
    ang.u8[3] = recv_frame.data[7];

    float current_angle = (float)ang.fl;

    current_angle = (current_angle*180.0f/M_PI) + 590040.0f;

    pos_ind_deg = current_angle;
    pos_ind_rad = current_angle * M_PI / 180.0f;

    return pos_ind_deg;
}

float Steadywin::get_speed_rpm()
{
    fl32u8 speed;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_INDICATOR;
    frame.data[1] = OUTPUT_SPEED;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    speed.u8[0] = recv_frame.data[4];
    speed.u8[1] = recv_frame.data[5];
    speed.u8[2] = recv_frame.data[6];
    speed.u8[3] = recv_frame.data[7];

    float current_speed = (float)speed.fl;

    current_speed = (current_speed*60.0f/M_PI) + 590040.0f;

    speed_ind_rpms = current_speed;

    return speed_ind_rpms;
}

float Steadywin::get_speed_rad()
{
    float curr_speed;
    curr_speed = get_speed_rpm();
    curr_speed = (curr_speed / 60.0f) * 2.0f * M_PI;
    speed_ind_rad = curr_speed;
    return speed_ind_rad;
}

float Steadywin::get_speed_deg()
{
    float curr_speed;
    curr_speed = get_speed_rpm();
    curr_speed = (curr_speed / 60.0f) * 360.0f;
    speed_ind_deg = curr_speed;
    return speed_ind_deg;
}

int Steadywin::mod_param(int param_id, uint32_t value)
{
    u32u8 val;
    val.u32 = value;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = MODIFY_PARAM;
    frame.data[1] = param_id;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = val.u8[0];
    frame.data[5] = val.u8[1];
    frame.data[6] = val.u8[2];
    frame.data[7] = val.u8[3];

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    return  retval;
}

int Steadywin::retr_param(int param_id, uint32_t& value)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_PARAM;
    frame.data[1] = param_id;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    value = ((uint32_t)recv_frame.data[7] << 24)
            + ((uint32_t)recv_frame.data[6] << 16)
            + ((uint32_t)recv_frame.data[5] << 8)
            + ((uint32_t)recv_frame.data[4]);

    
    return  retval;    
}

int Steadywin::get_version()
{
    int ver;
    u32u8 _ver;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = GET_VERSION;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    _ver.u8[0] = recv_frame.data[4];
    _ver.u8[1] = recv_frame.data[5];
    _ver.u8[2] = recv_frame.data[6];
    _ver.u8[3] = recv_frame.data[7];

    ver = _ver.u32;

    std::cout << "Firmware version: " << ver << "\n";

    return  ver;    
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
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;
    int faul;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    faul = recv_frame.data[2]; 

    std::cout << "Fault no.: " << faul << "\n";
    
    return faul;    
}

int Steadywin::ack_fault()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = ACK_FAULT;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[1];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    
    return  retval;    
}

int Steadywin::retr_indicator(int ind_id, float& value)
{
    fl32u8 val;

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = RETRIEVE_INDICATOR;
    frame.data[1] = ind_id;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    val.u8[0] = recv_frame.data[4];
    val.u8[1] = recv_frame.data[5];
    val.u8[2] = recv_frame.data[6];
    val.u8[3] = recv_frame.data[7];

    value = val.fl;

    
    return  retval;    
}

int Steadywin::calibrate(int calib_type)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = CALIBRATE;
    frame.data[1] = calib_type;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;
    int retval;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            retval  = recv_frame.data[2];
            #ifdef DEBUG
            std::cout << "Return value: " << retval << "\n";
            #endif
            break;
        }
    }

    
    return  retval;    
}

void Steadywin::update_firmware()
{
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = UPDATE_FIRMWARE;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

}