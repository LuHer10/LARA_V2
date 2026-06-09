#ifndef GIM6010_H
#define GIM6010_H

#include "socketcan.h"
#include <stdint.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <math.h>

    typedef enum {
        MW_HEARTBEAT_CMD = 0x001,                   //!<@brief 电机心跳周期数据
        MW_ESTOP_CMD = 0x002,                   //!<@brief 紧急停止
        MW_GET_ERROR_CMD = 0x003,                   //!<@brief 获取错误
        MW_RXSDO_CMD = 0x004,                   //!<@brief 接收SDO
        MW_TXSDO_CMD = 0x005,                   //!<@brief 发送SDO
        MW_SET_AXIS_NODE_ID_CMD = 0x006,                   //!<@brief 设置电机节点ID
        MW_SET_AXIS_STATE_CMD = 0x007,                   //!<@brief 设置电机状态
        MW_MIT_CONTROL_CMD = 0x008,                   //!<@brief MIT控制
        MW_GET_ENCODER_ESTIMATES_CMD = 0x009,                   //!<@brief 获取编码器估计值
        MW_GET_ENCODER_COUNT_CMD = 0x00A,                   //!<@brief 获取编码器计数
        MW_SET_CONTROLLER_MODE_CMD = 0x00B,                   //!<@brief 设置控制模式
        MW_SET_INPUT_POS_CMD = 0x00C,                   //!<@brief 设置输入位置
        MW_SET_INPUT_VEL_CMD = 0x00D,                   //!<@brief 设置输入速度
        MW_SET_INPUT_TORQUE_CMD = 0x00E,                   //!<@brief 设置输入力矩
        MW_SET_LIMITS_CMD = 0x00F,                   //!<@brief 设置速度电流限制
        MW_START_ANTICOGGING_CMD = 0x010,                   //!<@brief 进行力矩纹波校准
        MW_SET_TRAJ_VEL_LIMIT_CMD = 0x011,                   //!<@brief 滑行速度最大值
        MW_SET_TRAJ_ACCEL_LIMIT_CMD = 0x012,                   //!<@brief 滑行加（减）速度最大值
        MW_SET_TRAJ_INERTIA_CMD = 0x013,                   //!<@brief 惯性系数
        MW_GET_IQ_CMD = 0x014,                   //!<@brief 获取电机相电流配置    
        MW_REBOOT_CMD = 0x016,                   //!<@brief 重启
        MW_GET_BUS_VOLTAGE_CURRENT_CMD = 0x017,                   //!<@brief 获取总线电压电流
        MW_CLEAR_ERRORS_CMD = 0x018,                   //!<@brief 清除错误
        MW_SET_LINEAR_COUNT_CMD = 0x019,                   //!<@brief 设置编码器计数
        MW_SET_POS_GAIN_CMD = 0x01A,                   //!<@brief 设置位置增益
        MW_SET_VEL_GAIN_CMD = 0x01B,                   //!<@brief 设置速度增益
        MW_GET_TORQUES_CMD = 0x01C,                   //!<@brief 获取力矩
        MW_GET_POWERS_CMD = 0x01D,                   //!<@brief 获取功率
        MW_DISABLE_CAN_CMD = 0x01E,                   //!<@brief 禁用CAN
        MW_SAVE_CONFIGURATION_CMD = 0x01F                    //!<@brief 保存配置
    } MW_CMD_ID;

    typedef enum {
        MW_AXIS_STATE_UNDEFINED = 0x0,    
        MW_AXIS_STATE_IDLE = 0x1,   
        MW_AXIS_STATE_STARTUP_SEQUENCE = 0x2,    
        MW_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 0x3,    
        MW_AXIS_STATE_MOTOR_CALIBRATION = 0x4,    
        MW_AXIS_STATE_ENCODER_INDEX_SEARCH = 0x6,    
        MW_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 0x7,    
        MW_AXIS_STATE_CLOSED_LOOP_CONTROL = 0x8,    
        MW_AXIS_STATE_LOCKIN_SPIN = 0x9,    
        MW_AXIS_STATE_ENCODER_DIR_FIND = 0xA,    
        MW_AXIS_STATE_HOMING = 0xB,    
        MW_AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 0xC,   
        MW_AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 0XD,    
        MW_AXIS_STATE_ANTICOGGING_CALIBRATION = 0XE   
    } MW_MOTER_STATE;


class GIM6010
{
private:
    SocketCAN* can;
    uint8_t id;

    float pos;
    float vel;
    float torq;
public:
    GIM6010(SocketCAN* _can, uint8_t _id):can(_can)
    {
        id = _id;
    }

    void setAxisState(MW_MOTER_STATE state);
    void MITControl(float pos, float vel, float torq);

    void start_motor();
    void stop_motor();

};

#endif