/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-14 16:11:04
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 16:11:07
 * @FilePath: /mas_embedded_threadx/apps/sentry/gimbal_board/robot_func/robot_func.h
 * @Description:
 */
#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "sentry_def.h"

/**
 * @brief 计算相对于对齐角度的偏差，返回弧度制
 * @param getyawangle 当前编码器值 (0 - 8191)
 * @return 偏差弧度值 (范围 -PI 到 PI)
 */
float CalcOffsetAngle(float getyawangle);

/**
 * @brief 根据遥控器输入设置底盘、云台和发射机构的控制命令
 * @param Chassis_Ctrl 底盘控制命令结构体指针
 * @param Shoot_Ctrl 发射机构控制命令结构体指针
 * @param Gimbal_Ctrl 云台控制命令结构体指针
 */
void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif // _ROBOT_FUNC_H_
