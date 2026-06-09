#include "GIM6010.h"

void GIM6010::setAxisState(MW_MOTER_STATE state)
{
    // Implementation for setting the axis state

    uint8_t state_bytes[4];

    memcpy(state_bytes, &state, 4);

    struct can_frame frame;
    frame.can_id = (id << 5) | MW_SET_AXIS_STATE_CMD;
    frame.can_dlc = 8;
    frame.data[0] = state_bytes[0];
    frame.data[1] = state_bytes[1];
    frame.data[2] = state_bytes[2];
    frame.data[3] = state_bytes[3];
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

void GIM6010::MITControl(float pos, float vel, float torq)
{
    // Implementation for MIT control
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
