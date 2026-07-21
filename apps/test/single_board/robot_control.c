/*
 * robot_control.c
 *
 * F103C8T6 最小化单板测试
 * 仅初始化达妙电机 DM4310 + 注册离线检测
 * 不含遥控器/PS2 相关逻辑
 */

#include "app_init.h"
#include "module_motor.h"
#include "motor_damiao.h"

#define LOG_TAG "Robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static DM_Motor_t *test_motor = NULL;

void robot_control_init(void)
{
    Motor_Init_Config_s cfg = {
        .motor_init_info.motor_type = DM4310,
        .motor_init_info.gear_ratio = 1.0f,
        .motor_init_info.torque_constant = 0.0f,
        .motor_init_info.max_torque = 10.0f,
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can = {
            .hcan = BSP_CAN_HANDLE1,
            .tx_id = 0x001,
            .rx_id = 0x011,
        },
        .setting_init_config = {
            .loop_type = SPEED_LOOP,
            .enableflag = 0,
            .algorithm_type = CONTROL_PID,
            .motor_reverse_flag = 0,
            .feedback_reverse_flag = 0,
            .angle_feedback_source = 0,
            .speed_feedback_source = 0,
        },
        .controller_init_config = {
            .speed_PID = {
                .Kp = 0.5f,
                .Ki = 0.05f,
                .Kd = 0.0f,
                .MaxOut = 10.0f,
                .DeadBand = 0.0f,
                .IntegralLimit = 3.0f,
                .Improve = PID_Integral_Limit,
            },
            .angle_PID = {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 0.0f,
                .DeadBand = 0.0f,
                .IntegralLimit = 0.0f,
                .Improve = 0,
            },
        },
        .offline_init_config = {
            .name = "test_dm",
            .timeout_ms = 100,
            .beep_times = 1,
            .enable = 1,
        },
    };

    test_motor = Motor_DM_Init(&cfg, DM_MIT_MODE);
    if (test_motor != NULL)
    {
        /* 清除错误并使能 */
        Motor_DM_Cmd(test_motor, DM_CMD_CLEAR_ERROR);
        Motor_DM_Cmd(test_motor, DM_CMD_MOTOR_START);
        Motor_DM_Start(test_motor);
        /* 初始零转矩，电机保持位置 */
        Motor_DM_SetRef(test_motor, 0.0f);
        LOG_I("DM4310 test motor initialized on CAN1, ID=0x001");
    }
    else
    {
        LOG_E("DM4310 test motor init FAILED");
    }
}

void robot_control_loop(void)
{
    /* 占位：后续可添加遥控器/PS2 控制 */
}

void robot_control_exit(void)
{
    if (test_motor != NULL)
    {
        Motor_DM_SetRef(test_motor, 0.0f);
        Motor_DM_Stop(test_motor);
        Motor_DM_Cmd(test_motor, DM_CMD_MOTOR_STOP);
        LOG_I("DM4310 test motor stopped");
    }
}
