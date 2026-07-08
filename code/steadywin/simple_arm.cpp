#include <iostream>
#include <math.h>
#include "steadywin.h"
#include "GIM6010.h"

float l1 = 0.4f;
float l2 = 0.33f;
float l3 = 0.1f;

float x, y, th;
float q1, q2, q3;
float m1, m2, m3;

float s(float ang)
{
    return sin(ang);
}

float c(float ang)
{
    return cos(ang);
}

void IK()
{
    float wx = x - l3*c(th);
    float wy = y - l3*s(th);
    float wl = sqrt(wx*wx + wy*wy);

    float q_2 = acos((l1*l1 + l2*l2 - wx*wx - wy*wy)/(2.0f * l1 * l2));
    float q_1 = atan2(wy, wx) + asin((l2 * sin(q2))/sqrt(wx*wx + wy*wy));

    if(isnan(q_1) || isnan(q_2)) return;

    q2 = q_2;
    q1 = q_1;
    float alpha = q1 + q2 - M_PI;

    q3 = M_PI - alpha + th;

}

void DK()
{
    float alpha = q1 + q2 - M_PI;
    x = l1 * c(q1) + l2 * c(q1 + q2 - M_PI) + l3 * c(q1 + q2 + q3 - 2.0f*M_PI);
    y = l1 * s(q1) + l2 * s(q1 + q2 - M_PI) + l3 * s(q1 + q2 + q3 - 2.0f*M_PI);
    th = q3 - M_PI + alpha;
}

int main()
{
    //Can initialization
    SocketCAN can("can0");
    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }
    can.setNonBlocking(true);
    
    //Motor initialization
    GIM6010 big(&can, 11);
    Steadywin med(&can, 2, GIM4310_36);
    Steadywin small(&can, 1, GIM3505_8);

    big.start_motor();
    med.start_motor();
    small.start_motor();
    big.setTrapezodialMode();

    x = l2 + l3;
    y = l1;
    th = 0.0f;

    IK();

    //Move motors to initial position
    big.pos_ctrl_red_rad(q1 - (M_PI/2.0f));
    med.pos_control_rad(q2);
    small.pos_control_rad(q3);

    std::cout << "x: " << x << ", y: " << y << ", th: " << th 
            << ", q1: " << q1*180.0f/M_PI 
            << ", q2: " << q2*180.0f/M_PI  
            << ", q3: " << q3*180.0f/M_PI << "\n";

    while(true)
    {
        char dir;
        scanf("%c", &dir);
        if(dir == 'd')
        {
            x += 0.02;
        }
        if(dir == 'a')
        {
            x -= 0.02;
        }
        if(dir == 'w')
        {
            y += 0.02;
        }
        if(dir == 's')
        {
            y -= 0.02;
        }
        if(dir == 'x') break;

        IK();

        //Move motors
        big.pos_ctrl_red_rad(q1 - (M_PI/2.0f));
        med.pos_control_rad(q2);
        small.pos_control_rad(q3);

        DK();

        std::cout << "x: " << x << ", y: " << y << ", th: " << th 
                << ", q1: " << q1*180.0f/M_PI 
                << ", q2: " << q2*180.0f/M_PI  
                << ", q3: " << q3*180.0f/M_PI << "\n";

    }

    return 0;
}