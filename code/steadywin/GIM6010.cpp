#include "GIM6010.h"

void GIM6010::setAxisState(uint8_t state)
{
    // Implementation for setting the axis state

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_SET_AXIS_STATE_CMD;
    frame.can_dlc = 8;
    frame.data[0] = state;
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

void GIM6010::setControllerMode(uint8_t control_mode, uint8_t input_mode)
{
    // Implementation for setting the controller mode
    struct can_frame frame;
    frame.can_id = (id << 5) | MW_SET_CONTROLLER_MODE_CMD;
    frame.can_dlc = 8;
    frame.data[0] = control_mode;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = input_mode;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }
}

void GIM6010::setTrapezodialMode()
{
    setControllerMode(MW_POS_CTRL, MW_TRAPEZODIAL);
}

void GIM6010::setMITMode()
{
    setControllerMode(MW_POS_CTRL, MW_MIT);
}

void GIM6010::pos_control_rev(float revs, int16_t vel, int16_t torq)
{
    // Implementation for position control in revolutions

    fl32u8 _revs;
    _revs.fl = revs;
    i32u8 _vel;
    _vel.i32 = (int32_t)vel;
    i32u8 _torq;
    _torq.i32 = (int32_t)torq;

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_SET_INPUT_POS_CMD;
    frame.can_dlc = 8;
    frame.data[0] = _revs.u8[0];
    frame.data[1] = _revs.u8[1];
    frame.data[2] = _revs.u8[2];
    frame.data[3] = _revs.u8[3];
    frame.data[4] = _vel.u8[0];
    frame.data[5] = _vel.u8[1];
    frame.data[6] = _torq.u8[0];
    frame.data[7] = _torq.u8[1];

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }
}

void GIM6010::pos_control_rad(float rads, int16_t vel, int16_t torq)
{
    float revs = rads / (2.0f * M_PI);
    float _vel = vel / (2.0f * M_PI);
    pos_control_rev(revs, _vel, torq);
}



void GIM6010::vel_control(float vel, float torq)
{
    fl32u8 _vel;
    _vel.fl = vel;
    fl32u8 _torq;
    _torq.fl = torq;

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_SET_INPUT_VEL_CMD;
    frame.can_dlc = 8;
    frame.data[0] = _vel.u8[0];
    frame.data[1] = _vel.u8[1];
    frame.data[2] = _vel.u8[2];
    frame.data[3] = _vel.u8[3];
    frame.data[4] = _torq.u8[0];
    frame.data[5] = _torq.u8[1];
    frame.data[6] = _torq.u8[2];
    frame.data[7] = _torq.u8[3];

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }
}

void GIM6010::torq_control(float torq)
{
    fl32u8 _torq;
    _torq.fl = torq;

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_SET_INPUT_VEL_CMD;
    frame.can_dlc = 8;
    frame.data[0] = _torq.u8[0];
    frame.data[1] = _torq.u8[1];
    frame.data[2] = _torq.u8[2];
    frame.data[3] = _torq.u8[3];
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

void GIM6010::pos_ctrl_red_rev(float revs, int16_t vel = 0, int16_t torq = 0)
{
    float _revs = revs * gear_ratio;
    int16_t _vel = vel * gear_ratio;
    int16_t _torq = torq / gear_ratio;
    pos_control_rev(_revs, _vel, _torq);
}

void GIM6010::pos_ctrl_red_rad(float rads, int16_t vel = 0, int16_t torq = 0)
{
    float revs = rads / (2.0f * M_PI);
    int16_t _vel = vel / (2.0f * M_PI);
    pos_ctrl_red_rev(revs, _vel, torq);
}


void GIM6010::MITControl(float pos, float vel, float torq, float kp, float kd)
{
    // Implementation for MIT control
    int16_t pos_int = (int16_t)((pos + 12.5f) * 65535.0f / 25.0f);
    int16_t vel_int = (int16_t)((vel + 65.0f) * 4095.0f / 130.0f);
    int16_t torq_int = (int16_t)((torq + 50.0f) * 4095.0f / 100.0f);
    int16_t kp_int = (int16_t)(kp * 4095.0f / 500.0f);
    int16_t kd_int = (int16_t)(kd * 4095.0f / 5.0f);

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_MIT_CONTROL_CMD;
    frame.can_dlc = 8;
    frame.data[0] = pos_int >> 8;
    frame.data[1] = pos_int & 0xFF;
    frame.data[2] = vel_int >> 4;
    frame.data[3] = ((vel_int & 0xF) << 4) + (kp_int >> 8);
    frame.data[4] = kp_int & 0xFF;
    frame.data[5] = kd_int >> 4;
    frame.data[6] = ((kd_int & 0xF) << 4) + (torq_int >> 8);
    frame.data[7] = torq_int & 0xFF;

    if (can->sendFrame(frame)) {
        #ifdef DEBUG
        std::cout << "Frame sent\n";
        #endif
    }

    struct can_frame recv_frame;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            break;
        }
    }

    float pos_out = ((float)(recv_frame.data[1] << 8 | recv_frame.data[2]) * 25.0f / 65535) - 12.5f;
    float vel_out = ((float)(recv_frame.data[3] << 4 | recv_frame.data[4] >> 4) * 130.0f / 4095.0f) - 65.0f;
    float torq_out = ((float)(((recv_frame.data[4] & 0xF) << 8) | recv_frame.data[5]) * 100.0f / 4095.0f) - 50.0f;
    
    this->pos = pos_out;
    this->vel = vel_out;
    this->torq = torq_out;
}

void GIM6010::start_motor()
{
    // Implementation for starting the motor
    setAxisState(MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
}

void GIM6010::stop_motor()
{
    // Implementation for stopping the motor
    setAxisState(MW_AXIS_STATE_IDLE);
}

void GIM6010::getEncoderEstimates(float& _pos, float& _vel)
{
    // Implementation for getting encoder estimates

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_GET_ENCODER_ESTIMATES_CMD;
    frame.can_dlc = 8;
    frame.data[0] = 0x00;
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

    // Wait for response and parse the received data to get pos and vel
    struct can_frame recv_frame;

    while (true) {
        if (can->receiveFrame(recv_frame)) {
            std::cout << "Received ID: " << (recv_frame.can_id >> 5) << "\n";
            break;
        }
    }

    //float pos_out = _pos.fl;
    float vel_out;// = _vel.fl;

    float pos_out;
    memcpy(&pos_out, &recv_frame.data[0], sizeof(float));
    memcpy(&vel_out, &recv_frame.data[4], sizeof(float));
    
    std::cout << "Position: " << pos_out << ", Velocity: " << vel_out << "\n";
    _pos = pos_out;
    _vel = vel_out;
}
