#include "socketcan.h"
#include <stdint.h>
#include <iostream>


typedef enum
{
    RESET_CONFIG       = 0x81,
    REFRESH_CONFIG     = 0x82,
    MODIFY_CONFIG      = 0x83,
    RETRIEVE_CONFIG    = 0x84,

    START_MOTOR        = 0x91,
    STOP_MOTOR         = 0x92,
    TORQUE_CONTROL     = 0x93,
    SPEED_CONTROL      = 0x94,
    POSITION_CONTROL   = 0x95,
    PTS_CONTROL        = 0x96,
    STOP_CONTROL       = 0x97,

    MODIFY_PARAM       = 0xA1,
    RETRIEVE_PARAM     = 0xA2,

    GET_VERSION        = 0xB1,
    GET_FAULT          = 0xB2,
    ACK_FAULT          = 0xB3,
    RETRIEVE_INDICATOR = 0xB4,

    UPDATE_FIRMWARE = 0xC1
} COMMANDS;

typedef enum
{
    POLE_PAIRS           = 0x00,
    RATED_CURRENT        = 0x01,
    MAX_SPEED            = 0x02,
    RATED_VOLTAGE        = 0x06,
    PWM_FREQ             = 0x07,
    DEF_KP_CURRENT       = 0x08,
    DEF_KI_CURRENT       = 0x09,
    DEF_KP_SPEED         = 0X0C,
    DEF_KI_SPEED         = 0x0D,
    DEF_KP_POSITION      = 0x0E,
    DEF_KI_POSITION      = 0x0F,
    DEF_KD_POSITION      = 0x10,
    GEAR_RATIO           = 0x11,
    CAN_ID               = 0x12,
    HOST_CAN_ID          = 0x13,
    ZERO_POSITION        = 0x14,
    POWER_OFF_POS        = 0x15,
    OVER_VOLTAGE_THRESH  = 0x16,
    UNDER_VOLTAGE_THRESH = 0x17,
    CAN_BAUD_RATE        = 0x18,
    DEF_KP_FLUX          = 0x19,
    DEF_KI_FLUX          = 0x1A,
    OVER_TEMP_THRESH     = 0x20,
    CAN_PROTOCOL         = 0x1C
} CONF_ID;

typedef enum
{
    BUS_VOLTAGE       = 0X00,
    DRIVER_BOARD_TEMP = 0X01,
    MOTOR_TEMP        = 0X02,
    POWER             = 0X03,
    I_A               = 0X04,
    I_B               = 0X05,
    I_C               = 0X06,
    I_ALPHA           = 0X07,
    I_BETA            = 0X08,
    I_Q               = 0x09,
    I_D               = 0X0A,
    TARGET_IQ         = 0X0B,
    TARGET_ID         = 0X0C,
    V_Q               = 0X0D,
    V_D               = 0X0E,
    V_ALPHA           = 0X0F,
    V_BETA            = 0X10,
    ANGLE_ROTOR_ELEC  = 0X11,
    ANGLE_ROTOR_MEC   = 0X12,
    ANGLE_OUTPUT_MEC  = 0X13,
    OUTPUT_SPEED      = 0X14,
    OUTPUT_POWER      = 0X15
} IND_ID;

union fl32u8
{
    float fl;
    uint8_t u8[4];
};

union fl32i8
{
    float fl;
    int8_t i8[4];
};

union u32u8
{
    uint32_t u32;
    uint8_t u8[4];
};

union i32i8
{
    int32_t i32;
    int8_t i8[4];
};  

union i32u8
{
    int32_t i32;
    uint8_t u8[4];
};

class Steadywin
{
private:
    SocketCAN* can;
    int id;

    int8_t temp;
    float pos;
    float speed;
    float torque;
    float torq_const;
    int32_t gear_ratio;

public:

    Steadywin(SocketCAN* _can, int _id):can(_can)
    {
        //can = _can;
        id = _id;
    }

    float get_torq_const();
    int32_t get_gear_ratio();

    int reset_config();
    int refresh_config();
    int mod_config(int confid, int32_t data);
    int retr_config(uint8_t confid, int32_t& data);

    int start_motor();
    int stop_motor();
    int torque_control(float _torque, uint32_t duration = 0);
    int speed_control(float _speed, uint32_t duration = 0);
    int pos_control(float _pos, uint32_t duration = 0);
    int pos_control_deg(float _pos, uint32_t duration = 0);
    int stop_control();

    float get_position();

    int get_fault();




};

