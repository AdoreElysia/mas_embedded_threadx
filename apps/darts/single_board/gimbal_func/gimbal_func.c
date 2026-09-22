/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-15 20:36:42
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-22 22:25:52
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Description: 云台功能实现模板
 *
 * TODO: 实现你的云台控制逻辑
 *   1. 初始化 gimbal yaw/pitch 电机 (CAN)
 *   2. 每周期根据 gimbal_cmd 计算 PID 输出
 *   3. 回传当前 yaw 编码器值
 */

#include "gimbal_func.h"
#include "module_bmi088.h"
#include "module_ins.h"
#include "motor_dji.h"
#include "motor_damiao.h"
#include "motor_zdt.h"
#include "user_lib.h"

static DJI_Motor_t *inside_motor = NULL ;
static UART_Motor_t *yaw_motor = NULL ;

void gimbal_init(void)
{
    /* TODO: 初始化云台电机 */
}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    (void)gimbal_cmd;
    (void)yaw_ecd;

    /* TODO: 实现云台控制
     *   - 根据 gimbal_cmd->yaw / gimbal_cmd->pitch 计算目标位置
     *   - 根据 gimbal_cmd->gimbal_mode 切换控制模式
     *   - 读取 yaw 编码器 → *yaw_ecd
     *   - 调用电机 CAN 发送命令
     */
}
