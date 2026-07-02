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
    big->getEncoderEstimates_rad(mBig, v);
    mMed = med->get_position_rad();
    mSmall = small->get_position_rad();

}

int Arm::DK(float q_1, float q_2, float q_3, float &p_x, float &p_y, float &th)
{
    int err = 0;//test_qs(q_1, q_2, q_3);
    //if(err == -1) return -1;
    float alpha = q_1;
    p_x = l1 * cos(q_1) + l2 * cos(q_1 + q_2 - M_PI) + l3 * cos(q_1 + q_3 - M_PI);
    p_y = l1 * sin(q_1) + l2 * sin(q_1 + q_2 - M_PI) + l3 * sin(q_1 + q_3 - M_PI);
    th = q_3 - M_PI + alpha;
    return 0;
}

int Arm::IK(float p_x, float p_y, float th, float &q_1, float &q_2, float &q_3)
{
    float w_x = p_x - l3 * cos(th);
    float w_y = p_y - l3 * sin(th);

    float wl = sqrt(wx*wx + wy*wy);
    //if(wl >= (l1+l2+l3 - MARGIN)) return -1;

    q_2 = acos((l1*l1 + l2*l2 - w_x*w_x - w_y*w_y)/(2.0f * l1 * l2));
    q_1 = atan2(w_y, w_x) + asin((l2 * sin(q_2))/sqrt(w_x*w_x + w_y*w_y));
    
    float alpha = q_1;

    q_3 = M_PI - alpha + theta;

    int err = 0; //test_qs(q_1, q_2, q_3);
    if(std::isnan(q_1) || std::isnan(q_2) || std::isnan(q_3)) return -1;
    return err;
}

