/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: 零控(LingKong) MF9025 电机驱动
 *   继承 Motor_Base，速度模式(0xA2)控制，CAN 通信
 */
#ifndef _MOTOR_LINGKONG_H_
#define _MOTOR_LINGKONG_H_

#include "motor_base.h"
#include "motor_def.h"
#include <stdint.h>

/* 零控电机命令字节 */
typedef enum
{
    LK_CMD_MOTOR_CLOSE     = 0x80, /* 电机关闭 */
    LK_CMD_MOTOR_STOP      = 0x81, /* 电机停止 */
    LK_CMD_MOTOR_RUN       = 0x88, /* 电机运行 */
    LK_CMD_GET_STATUS      = 0x9C, /* 读取状态2 */
    LK_CMD_SET_TORQUE      = 0xA1, /* 转矩闭环控制 */
    LK_CMD_SET_SPEED       = 0xA2, /* 速度闭环控制 */
} LK_Motor_Cmd_e;

/* 零控电机测量数据 */
typedef struct
{
    uint8_t  id;                        /* 电机ID */
    int8_t   temperature;               /* 温度 (℃) */
    int16_t  current_raw;               /* 转矩电流原始值 (-2048~2048) */
    uint16_t encoder_pos;               /* 编码器位置 (0~65535) */
    float    last_single_round_angle;   /* 上一次单圈角度 (rad) */
    int32_t  total_round;               /* 总圈数 */
} LK_Motor_Measure_s;

/* LK_Motor_t — 继承 Motor_Base */
typedef struct
{
    Motor_Base          base;    /* [必须首字段] 公共基类 */
    LK_Motor_Measure_s  measure; /* 零控测量数据 */
} LK_Motor_t;

/**
 * @brief 零控电机初始化
 * @param config 电机初始化配置 (transport 必须为 MOTOR_TRANSPORT_CAN)
 * @return LK_Motor_t 指针, 失败返回 NULL
 */
LK_Motor_t *Motor_LK_Init(Motor_Init_Config_s *config);

/**
 * @brief 零控电机使能 (发送 0x88)
 */
void Motor_LK_Start(LK_Motor_t *motor);

/**
 * @brief 零控电机停止 (发送 0x80)
 */
void Motor_LK_Stop(LK_Motor_t *motor);

/**
 * @brief 设置参考速度 (rad/s)
 */
void Motor_LK_SetRef(LK_Motor_t *motor, float ref);

#endif /* _MOTOR_LINGKONG_H_ */
