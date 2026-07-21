#include "base.h"

inline void Base::FK(const float vm[4], float &vx, float &vy, float &vrad)
{
    vx   = WHEEL_RADIUS * (-vm[0] + vm[1] + vm[2] - vm[3])/4.0F;
    vy   = WHEEL_RADIUS * ( vm[0] + vm[1] + vm[2] + vm[3])/4.0F;
    vrad = WHEEL_RADIUS * (-vm[0] + vm[1] - vm[2] + vm[3])/(4.0F*LXY);
}

inline void Base::IK(const float &vx, const float &vy, const float &vrad, float vm[4])
{
    vm[0] = ( vx + vy - vrad*LXY) /* WHEEL_RADIUS*/;
    vm[1] = (-vx + vy + vrad*LXY) /* WHEEL_RADIUS*/;
    vm[2] = (-vx + vy - vrad*LXY) /* WHEEL_RADIUS*/;
    vm[3] = ( vx + vy + vrad*LXY) /* WHEEL_RADIUS*/;
}

void Base::setVelocity(float vx, float vy, float vrad)
{
    float vm[4];
    IK(vx, vy, vrad, vm);
    for(int i = 0; i < 4; i++)
    {
        vm[i] *= RAD_2_QPPR;
        targetVelocity[i].t_int = (int32_t)vm[i];
    }
}

void Base::sendSpeed()
{
    roboclaw.M1M2Speed(FRONT_ADDRESS, targetVelocity[M_FRONT_LEFT].t_int, targetVelocity[M_FRONT_RIGHT].t_int);
    roboclaw.M1M2Speed(BACK_ADDRESS, targetVelocity[M_BACK_LEFT].t_int, targetVelocity[M_BACK_RIGHT].t_int);
}

void Base::readEncoders()
{
    int32_u encs[4];
    float angs[4] = {0.0F};
    bool front, back;
    front = roboclaw.readEncoders(FRONT_ADDRESS, encs[M_FRONT_LEFT].t_uint, encs[M_FRONT_RIGHT].t_uint);
    back  = roboclaw.readEncoders( BACK_ADDRESS, encs[M_BACK_LEFT].t_uint,  encs[M_BACK_RIGHT].t_uint);

    if(front && back)
    {
        for(int i = 0; i < 4; i++)
        {
            angs[i] = encs[i].t_int - encoderCounts[i].t_int;
            encoderCounts[i].t_int = encs[i].t_int;
            angs[i] *= QPPR_2_RAD;
        }

        float d_xr, d_yr, d_ang;
        FK(angs, d_xr, d_yr, d_ang);

        x += (-d_xr * cos(ang + d_ang/2) + d_yr * sin(ang + d_ang/2));
        y +=  (d_xr * sin(ang + d_ang/2) + d_yr * cos(ang + d_ang/2));
        ang += d_ang;

    }
}

void Base::readSpeeds()
{
    bool front, back;
    float vangs[4] = {0.0F};

    front = roboclaw.readSpeeds(FRONT_ADDRESS, currVelocity[M_FRONT_LEFT].t_int, currVelocity[M_FRONT_RIGHT].t_int);
    back  = roboclaw.readSpeeds( BACK_ADDRESS, currVelocity[M_BACK_LEFT].t_int,  currVelocity[M_BACK_RIGHT].t_int);
    if(front && back)
    {
        for(int i = 0; i < 4; i++)
            vangs[i] = currVelocity[i].t_int * QPPR_2_RAD;
        
        FK(vangs, vx, vy, vang);
    }    
}

void Base::resetOdometry()
{
    for(int i = 0; i < 4; i++)
        encoderCounts[i].t_int = 0;
    x = y = ang = 0.0F;
    roboclaw.resetEncoders(FRONT_ADDRESS);
    roboclaw.resetEncoders(BACK_ADDRESS);

}