/*
 * robot_control.c
 *
 * F103C8T6 最小化单板控制入口
 * 仅包含 REMOTE + MOTOR + OFFLINE
 */

#include "app_init.h"
#include "module_remote.h"
#include "module_motor.h"

void Robot_Control_Init(void)
{
    LOG_I("F103 single board control init");
}

void Robot_Control_Loop(void)
{
    /* 遥控器数据 */
    const Remote_t *remote = Module_REMOTE_get();
    if (remote == NULL)
    {
        return;
    }

    /* 基础遥控控制：左摇杆 Y 轴控制电机转速 */
    Motor_Set_Current(0, (int16_t)(remote->ch[REMOTE_CH_LY] * 0.5f));
}

void Robot_Control_Exit(void)
{
    Motor_Set_Current(0, 0);
}

