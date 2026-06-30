#include "arm.h"

void Arm::writePos(float _qBig, float _qMed, float _qSmall)
{
    small->pos_control_rad(_qSmall);
    med->pos_control_rad(_qSmall);
    big->pos_ctrl_red_rad(_qBig);

    updateQs();
}

void Arm::updateQs()
{
    float v;
    big->getEncoderEstimates_rad(qBig, v);
    qMed = med->get_position_rad();
    qSmall = small->get_position_rad();

}

int Arm::DK(float q_1, float q_2, float q_3, float &p_x, float &p_y, float &th)
{
    
}

int Arm::IK(float p_x, float p_y, float th, float &q_1, float &q_2, float &q_3)
{

}

