/*
 * robot_control.c
 *
 * F103C8T6 最小化单板控制入口
 * 仅包含 REMOTE + MOTOR + OFFLINE
 */

#include "app_init.h"
#include "module_remote.h"
#include "module_motor.h"
#include "motor_dji.h"

#define LOG_TAG "Robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static DJI_Motor_t *test_motor = NULL;

void Robot_Control_Init(void)
{
    Motor_Init_Config_s cfg = {
        .motor_type = M3508,
        .transport  = MOTOR_TRANSPORT_CAN,
        .transport_config.can = {
            .hcan = BSP_CAN_HANDLE1,
            .tx_id = 0x200,
            .rx_id = 0x200,
        },
        .setting_init_config = {
            .loop_type  = SPEED_LOOP,
            .enableflag = 1,
        },
        .controller_init_config = {
            .speed_PID = {
                .Kp = 5.0f,
                .Ki = 0.5f,
                .Kd = 0.0f,
                .MaxOut = 16384.0f,
                .IntegralLimit = 5000.0f,
                .Improve = PID_Integral_Limit,
            },
        },
    };
    test_motor = Motor_DJI_Init(&cfg);

    LOG_I("F103 single board control init");
}

void Robot_Control_Loop(void)
{
    Remote_Data_t *remote = Module_Remote_get_data();
    if (remote == NULL)
    {
        return;
    }

    /* 左摇杆 Y 轴 (ch2) 控制电机转速 */
    int16_t ch_val = Module_Remote_get_channel(2);
    float speed_ref = (float)ch_val / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS) * 1000.0f;

    if (test_motor)
    {
        Motor_DJI_SetRef(test_motor, speed_ref);
    }
    Motor_DJI_Flush();
}

void Robot_Control_Exit(void)
{
    if (test_motor)
    {
        Motor_DJI_SetRef(test_motor, 0.0f);
    }
    Motor_DJI_Flush();
}
