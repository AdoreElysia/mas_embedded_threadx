/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: Target 机器人定义
 *   1× DM6220 (顶部旋转, MIT模式, 软件速度环)
 *   2× MF9025 (左右轮子, 速度模式, 电机内部闭环)
 *   PS2 遥控器 (软件 SPI, C板可配置IO)
 */
#ifndef _TARGET_DEF_H_
#define _TARGET_DEF_H_

#include <stdint.h>

/* ========== 机械参数 ========== */
#define WHEEL_RADIUS        0.08f           /* 轮半径 8cm (bache 代码验证) */
#define MAX_WHEEL_SPEED_MPS 1.0f            /* 最大轮速 1 m/s */
#define MAX_WHEEL_SPEED_RAD (MAX_WHEEL_SPEED_MPS / WHEEL_RADIUS) /* 12.5 rad/s */

/* ========== DM6220 转速档位 ========== */
#define PI                  3.14159265359f
#define DM6220_SPEED_LOW    (0.4f * 2.0f * PI)   /* 0.4 转/秒 → rad/s */
#define DM6220_SPEED_HIGH   (0.8f * 2.0f * PI)   /* 0.8 转/秒 → rad/s */

/* ========== CAN 总线分配 ========== */
/* 全部挂在 CAN1 上, ID 不冲突 */
#define TARGET_CAN_BUS      BSP_CAN_HANDLE1

/* DM6220 (达妙, MIT模式) */
#define DM6220_CAN_TX_ID    0x01
#define DM6220_CAN_RX_ID    0x11   /* Master ID > CAN ID */

/* MF9025 (零控, 速度模式) */
#define MF9025_LEFT_ID      1       /* CAN ID: 0x141 */
#define MF9025_RIGHT_ID     2       /* CAN ID: 0x142 */

/* ========== 离线检测超时 ========== */
#define MOTOR_OFFLINE_TIMEOUT_MS  200

#endif /* _TARGET_DEF_H_ */
