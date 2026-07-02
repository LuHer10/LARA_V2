#include "arm.h"

void Arm::writePos(float _mBig, float _mMed, float _mSmall)
{
    small->pos_control_rad(_mSmall);
    med->pos_control_rad(_mSmall);
    big->pos_ctrl_red_rad(_mBig);

    updateQs();
}

void Arm::writePosTh()
{
    float deg_vel = 180.0f;
    float step_size = deg_vel*M_PI/180.0f;
    const int delta_time = 10; // Time in milliseconds
    float current_angle1 = med->get_position_deg(); // Get the current position in degrees
    float current_angle2 = small->get_position_deg(); // Get the current position in degrees
    float step = step_size * ((float)delta_time/1000.0f);//(rpm / 60.0f) * (delta_time / 1000.0f);  // Adjust the step size as needed

    while (true) { // Loop until the target angle is reached

        if(mMed > current_angle1) {
            current_angle1 += step; // Increment the angle
        } else if(mMed < current_angle1){
            current_angle1 -= step; // Decrement the angle
        }

        if(mSmall > current_angle2) {
            current_angle2 += step; // Increment the angle
        } else if(mSmall < current_angle2){
            current_angle2 -= step; // Decrement the angle
        }

        med->pos_control_rad(current_angle1);
        small->pos_control_rad(current_angle2);
        big->pos_ctrl_red_rad(mBig);


        std::this_thread::sleep_for(std::chrono::milliseconds(delta_time));
        //current_angle = m.get_position_deg(); // Get the current position in degrees
        updateQs();
    }

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

int Arm::move(float p_x, float p_y, float th)
{
    float q_1, q_2, q_3;
    float m_1, m_2, m_3;
    if(IK(p_x, p_y, th, q_1, q_2, q_3)) return -1;
    q2m(q_1, q_2, q_3, m_1, m_2, m_3);
    if(abs(m_1) >= (80.0f*M_PI/180.0f)) return -1;

    px = p_x;
    py = p_y;
    theta = th;

    qBig = q_1;
    qMed = q_2;
    qSmall = q_3;

    mBig = m_1;
    mMed = m_2;
    mSmall = m_3;

    //writePos(mBig, mMed, mSmall);
}

int Arm::moveIncr(float dx, float dy, float dth)
{
    float p_x = px + dx;
    float p_y = py + dy;
    float th = theta + dth;

    float q_1, q_2, q_3;
    float m_1, m_2, m_3;
    if(IK(p_x, p_y, th, q_1, q_2, q_3)) return -1;
    q2m(q_1, q_2, q_3, m_1, m_2, m_3);
    if(abs(m_1) >= (80.0f*M_PI/180.0f)) return -1;

    px += dx;
    py += dy;
    theta += dth;

    qBig = q_1;
    qMed = q_2;
    qSmall = q_3;

    mBig = m_1;
    mMed = m_2;
    mSmall = m_3;

    //writePos(mBig, mMed, mSmall);

}
