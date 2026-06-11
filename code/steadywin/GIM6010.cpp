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
