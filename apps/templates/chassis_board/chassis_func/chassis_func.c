/*
 * @Author: AdoreElysia w2006825@qq.com
 * @Date: 2026-06-05 21:07:40
 * @LastEditors: AdoreElysia w2006825@qq.com
 * @LastEditTime: 2026-07-11 17:32:13
 * @FilePath: \mas_embedded_threadx\apps\templates\chassis_board\chassis_func\chassis_func.c
 * @Description: 
 */
/*
 * @Description: 底盘功能实现模板 (底盘板)
 *
 * TODO: 实现底盘控制逻辑
 */

#include "chassis_func.h"

void chassis_init(void)
{
    /* TODO: 初始化底盘电机 */
}

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    (void)chassis_cmd;

    /* TODO: 实现底盘控制
     *   - 根据 chassis_cmd->vx / vy / wz 计算电机速度
     *   - 根据 chassis_cmd->chassis_mode 切换控制模式
     *   - 跟随模式: 使用 chassis_cmd->offset_angle 做 yaw 闭环
     */
}
