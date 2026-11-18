#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "../robot/base.h"
#include "../robot/network.h"
#include "../robot/arm2.h"

// Joystick full-scale speeds. Base::setVelocity takes SI units (m/s, rad/s);
// these reproduce the speeds the robot had before the IK unit fix
// (5 and 10 "wheel rad/s" units => ~0.32 m/s and ~0.64 rad/s).
constexpr float MAX_LIN_SPEED = 5.0f  * WHEEL_RADIUS;  // m/s
constexpr float MAX_ANG_SPEED = 10.0f * WHEEL_RADIUS;  // rad/s


int main() {

    SocketCAN can("can0");
    bool canOk = can.open();
    if (!canOk) {
        std::cerr << "Failed to open CAN: running base only, arm disabled\n";
    } else {
        can.setNonBlocking(true);
    }

    Network net;
    
    float x_left = 0.0F;
    float y_left = 0.0F;
    float x_right = 0.0F;
    float y_right = 0.0F;

    float q1 = 0, q2 = 0, q3 = 0;
    float arm_x = 0, arm_y = 0, arm_th = 0;

    // estas variables no se lo que son
    int8_t d1 = 0, d2 = 0, d3 = 0, d4 = 0;

    int8_t mode = 0;
    int8_t grip = 0;
    int8_t omni = 0;
    int8_t emergency = 0;
    int8_t pre_emer = emergency;

    int home = 0;
    int pre_home = home;

    int manual = 0;

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

    std::unique_ptr<Arm> arm;
    if (canOk) {
        try {
            arm = std::make_unique<Arm>(&can, l1, l2, l3);
            arm->getCoord(arm_x, arm_y, arm_th);
            arm->getQs(q1, q2, q3);
        } catch (const std::exception& e) {
            std::cerr << e.what() << ": running base only, arm disabled\n";
        }
    }
    
    std::cout << "YUJU1" << std::endl;
    base.readEncoders();
    std::cout << "YUJU2" << std::endl;
    odo = base.getOdometry();

    //arm.open();
    //arm.getUp();

    while (1) {

        pre_x_left = x_left;
        pre_y_left = y_left;
        pre_x_right = x_right;
        pre_y_right = y_right;

        //net.receiveJoysticks(x_left, y_left, x_right, y_right, mode);
        net.receiveJoysticks(x_left, y_left, x_right, y_right,
                             d1, d2, d3, d4,
                             mode);

        pre_home = home;
        pre_emer = emergency;

        home = (mode >> 2) & 0x01;
        omni = (mode >> 1) & 0x01;
        emergency = (mode >> 4) & 0x01;

        base.readEncoders();

        odo = base.getOdometry();

        net.sendOdometry(odo.x, odo.y, odo.ang);

        if (arm) {
            arm->getQs(q1, q2, q3);
            arm->getCoord(arm_x, arm_y, arm_th);
        }

        changed = (pre_x_left != x_left)
                || (pre_y_left != y_left)
                || (pre_x_right != x_right)
                || (pre_y_right != y_right);

        //changed = 1;
        //x_left = 0;
        //printf("Xdir Ydir Rotx Roty :%f %f %f %f\n", x_left, y_left, x_right, y_right);

        if(pre_home != home && home)
        {
            //arm.getUp();
        }

        if(pre_home != home && !home)
        {
            //arm.home();
        }


        if(emergency)
        {
            if(emergency != pre_emer && arm)
            {
                arm->disable();
            }
            // Actually command zero speed every cycle while the E-stop is held
            base.setVelocity(0.0f, 0.0f, 0.0f);
            base.sendSpeed();
            usleep(1000);
            continue;
        }
        if(!emergency)
        {
            if(emergency != pre_emer && arm)
            {
                arm->enable();
            }
        }

        if(changed)
        {
            if(omni)
            {
                // left y right se refiere a los indicadores de la app de Lucas 
                // X en la ap es el eje horizontal
                // Y en la ap es el eje vertical
                base.setVelocity(x_left*MAX_LIN_SPEED, y_left*MAX_LIN_SPEED, -x_right*MAX_ANG_SPEED);
                //base.setVelocity(0.0f, 0.0f, 0.0f);
                base.sendSpeed();
                
            }
            else
            {
                base.setVelocity(0.0f , y_left*MAX_LIN_SPEED, -x_left*MAX_ANG_SPEED);
                base.sendSpeed();
            }
            
        }

        //arm.moveCoordIncr(x_right*0.001f, y_right*0.001f, 0.0f);
        if (arm) arm->moveQs_incr(0.01f*d1, 0.01f*d2, 0.01f*d3);

        //if(!omni)
        //{
            if(!manual)
            {
                if(x_right || y_right){}
                    //arm.moveCoordIncr(x_right*0.0001f, y_right*0.0001f, 0.0f);
            }
            else {
                //arm.moveQs_incr(0.01f*d1, 0.01f*d2, 0.01*d3);
            }
        //}

        grip = mode & 0x01;

        if(grip)
        {
            //arm.open();
        }
        else
        {
            //arm.close();
            
        }

        //arm.moveWristIncr(d4);

        printf("Brazo motores: %f, %f, %f \n", q1*180.0f/M_PI, q2*180.0f/M_PI, q3*180.0f/M_PI);
        if (arm) {
            printf("motor start: %f, %f, %f \n", arm->m1_start.load()*180.0f/M_PI, arm->m2_start.load()*180.0f/M_PI, arm->m3_start.load()*180.0f/M_PI);
            printf("motor offset: %f, %f, %f \n", arm->m1_offset.load()*180.0f/M_PI, arm->m2_offset.load()*180.0f/M_PI, arm->m3_offset.load()*180.0f/M_PI);
        }
        //printf("Brazo X Y theta: %f, %f, %f, %d \n", x_left, arm_y, arm_th, manual);
        //printf("Esto??: %d, %d, %d  \n", d1, d2, d3);

        //printf("%f, %f, %f, %f, %d \n", x_left, y_left, x_right, y_right, grip);

        usleep(1000);
    
    }
    return 0;
}
