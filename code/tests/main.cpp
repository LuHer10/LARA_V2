#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cmath>

#include "../robot/base.h"
#include "../robot/network.h"
//#include "../robot/arm.h"


int main() {

    /*SocketCAN can("can0");
    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }
    can.setNonBlocking(true);
*/
    Network net;
    
    float x_left = 0.0F;
    float y_left = 0.0F;
    float x_right = 0.0F;
    float y_right = 0.0F;

    float q1, q2, q3;
    float arm_x, arm_y, arm_th;

    int8_t d1, d2, d3;

    int8_t mode;
    int8_t grip = 0;
    int8_t omni = 0;

    int manual = 0;
    int pre_manual = manual;

    float q2off = 0;
    float q3off = 0;

    float pre_x_left, pre_y_left;
    float pre_x_right, pre_y_right;

    bool changed = false;

    Odometry odo;

    Base base;

    float l1 = 0.4f;
    float l2 = 0.33f;
    float l3 = 0.1f;

    //Arm arm(&can, l1, l2, l3);
    //arm.getCoord(arm_x, arm_y, arm_th);
    //arm.getQs(q1, q2, q3);
    
    
    base.readEncoders();
    odo = base.getOdometry();

    //arm.open();

    while (1) {

        pre_x_left = x_left;
        pre_y_left = y_left;
        pre_x_right = x_right;
        pre_y_right = y_right;

        //net.receiveJoysticks(x_left, y_left, x_right, y_right, mode);
        net.receiveJoysticks(x_left, y_left, x_right, y_right,
                             d1, d2, d3,
                             mode);

        pre_manual = manual;

        manual = mode >> 2;
        //omni = (mode >> 1) & 0x01;

        base.readEncoders();

        odo = base.getOdometry();

       net.sendOdometry(odo.x, odo.y, odo.ang);

      //  arm.getQs(q1, q2, q3);
        //arm.DK(q1, q2, q3, arm_x, arm_y, arm_th);
        //arm.getCoord(arm_x, arm_y, arm_th);

        changed = (pre_x_left != x_left)
                || (pre_y_left != y_left)
                || (pre_x_right != x_right)
                || (pre_y_right != y_right);

        if(changed)
        {
 //           if(omni)
  //          {
                base.setVelocity(x_left*5.0f, y_left*5.0f, -x_right*5.0f);
                base.sendSpeed();
   //         }
    //        else
    //        {
//                base.setVelocity(0.0f , y_left*5.0f, -x_left*5.0f);
//                base.sendSpeed();
    //        }
            
        }
/*
        if(!omni)
        {
            if(!manual)
            {
                if(x_right || y_right){}
                    //arm.moveCoordIncr(x_right*0.0001f, y_right*0.0001f, 0.0f);
            }
            else {
                //arm.moveQs_incr(0.01f*d1, 0.01f*d2, 0.01*d3);
            }
        }
*/
        if(pre_manual != manual && !manual)
        {
            //arm.setOffset();
            //arm.home();
        }

        grip = mode & 0x01;

        if(mode & 0x01)
        {
            //arm.close();
        }
        else
        {
            //arm.open();
        }

        printf("%f, %f, %f      ", q1*180.0f/M_PI, q2*180.0f/M_PI, q3*180.0f/M_PI);
        printf("%f, %f, %f, %d     ", x_left, arm_y, arm_th, manual);
        printf("%d, %d, %d  \n", d1, d2, d3);

        //printf("%f, %f, %f, %f, %d \n", x_left, y_left, x_right, y_right, grip);

        usleep(1000);
    
    }
    return 0;
}
