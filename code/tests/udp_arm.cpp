#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

//#include "../robot/base.h"
#include "../robot/network.h"
#include "../robot/arm.h"


int main() {

    SocketCAN can("can0");
    if (!can.open()) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }
    can.setNonBlocking(true);

    Network net;
    
    float x_left = 0.0F;
    float y_left = 0.0F;
    float x_right = 0.0F;
    float y_right = 0.0F;

    float q1, q2, q3;
    float arm_x, arm_y, arm_th;

    int8_t mode;
    int8_t grip = 0;

    int manual = 0;
    int pre_manual = manual;

    float q2off = 0;
    float q3off = 0;

    float pre_x_left, pre_y_left;
    float pre_x_right, pre_y_right;

    bool changed = false;

    //Odometry odo;

    //Base base;

    float l1 = 0.4f;
    float l2 = 0.33f;
    float l3 = 0.1f;

    Arm arm(l1, l2, l3);
    arm.getCoord(arm_x, arm_y, arm_th);
    arm.getQs(q1, q2, q3);
    
    
    //base.readEncoders();
    //odo = base.getOdometry();

    //arm.open();

    while (1) {

        pre_x_left = x_left;
        pre_y_left = y_left;
        pre_x_right = x_right;
        pre_y_right = y_right;

        net.receiveJoysticks(x_left, y_left, x_right, y_right, mode);

        pre_manual = manual;

        manual = mode >> 2;

        //base.readEncoders();

        //odo = base.getOdometry();

        //net.sendOdometry(odo.x, odo.y, odo.ang);

        arm.getQs(q1, q2, q3);
        arm.DK(q1, q2, q3, arm_x, arm_y, arm_th);
        arm.getCoord(arm_x, arm_y, arm_th);

        changed = (pre_x_left != x_left)
                || (pre_y_left != y_left)
                || (pre_x_right != x_right)
                || (pre_y_right != y_right);

        if(changed)
        {
            //if(arm_x > 0.7f || arm_x < 0.3f)
            //{
                //base.setVelocity(x_left*5.0f, y_left*5.0f, 0/*-x_right*/);
                //base.sendSpeed();
                //arm.moveIncr(0.0f, y_right*0.1f, 0.0f);
            //}
            //else
            //{
                //arm.moveIncr(y_left*0.1f, y_right*0.1f, 0.0f);
            //}

            
        }

        if(x_right || y_right)
        {
            if(manual == 0)
                arm.moveCoordIncr(x_right*0.005f, y_right*0.005f, 0.0f);
            if(manual == 1)
            {
                //arm.move_q1(PI/2.0f);
                //arm.moveIncr_ms(0.0f, x_left, y_right);
            }
            
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

        /*
        if(pre_manual != manual && !manual)
        {
            arm.setOffset();
            arm.move_qs(PI/2.0f, PI, PI);
        }
        */

        printf("%f, %f, %f      ", q1*180.0f/PI, q2*180.0f/PI, q3*180.0f/PI);
        printf("%f, %f, %f, %d  \n", arm_x, arm_y, arm_th, mode);

        //printf("%f, %f, %f, %f, %d", x_left, y_left, x_right, y_right, grip);

        usleep(1000);  // 10 Hz loop
    
    }
    return 0;
}
