#include <iostream>
#include <math.h>
#include <thread>
#include "steadywin.h"
#include "GIM6010.h"
#include "arm.h"




int main()
{

    float l1 = 0.4f;
    float l2 = 0.33f;
    float l3 = 0.1f;

    float x, y, th;
    float q1, q2, q3;
    float mx, my, mth;
    float mq1, mq2, mq3;

    char dir;

    //Can initialization
    SocketCAN can("can0");
    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }
    can.setNonBlocking(true);
    
    //Motor initialization
    Arm arm(&can, l1, l2, l3);

    arm.getCoord(x, y, th);
    arm.getQs(q1, q2, q3);

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

        //IK();

        //Move motors
        //big.pos_ctrl_red_rad(q1 - (M_PI/2.0f));
        //med.pos_control_rad(q2);
        //small.pos_control_rad(q3);

        //DK();

        arm.getMCoord(mx, my, mth);
        arm.getMqs(mq1, mq2, mq3);
        arm.moveCoord(x, y, th);


        std::cout << "x: " << mx << ", y: " << my << ", th: " << mth 
                << ", q1: " << (mq1 + (M_PI/2.0f))*180.0f/M_PI 
                << ", q2: " << mq2*180.0f/M_PI  
                << ", q3: " << mq3*180.0f/M_PI << "\n";

    }

    dir = 'x';


    return 0;
}