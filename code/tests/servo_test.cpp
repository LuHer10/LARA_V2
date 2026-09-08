#include "../robot/servo.h"
#include <unistd.h>
#include <stdio.h>

int main()
{
    Servo servos;
    servos.init();

    //servos.extPosContMode(11);
    //servos.extPosContMode(20);
    //servos.extPosContMode(30);
    //servos.extPosContMode(40);
    //int pos11 = servos.readPos(11);
    //int pos30 = servos.readPos(30);
    //int pos40 = servos.readPos(40);



    //printf("q1: %d, q2: %d, q3: %d\n", pos11, pos40, pos30);
    //servos.enableTorque(11);
    //servos.enableTorque(30);
    //servos.enableTorque(40);
    servos.enableTorque(1);

    //servos.writePos(11, degToPos(90));
    //servos.writePos(40, 3500); //6700
    //servos.writePos(30, -500); //1655 -200
    
    
    servos.writePos(1, degToPos(90));

    sleep(2);

    servos.writePos(1, degToPos(135));
    
    sleep(2);

    ///servos.writePos(11, degToPos(90));
    //servos.writePos(40, degToPos(60));
    //servos.writePos(30, degToPos(90));



    //sleep(20);


    //servos.disableTorque(11);
    //servos.disableTorque(30);
    //servos.disableTorque(40);
    //servos.disableTorque(50);

}
