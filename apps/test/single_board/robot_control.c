/*
 * robot_control.c
 *
 * F103C8T6 单板测试
 *   云台: 达妙电机 DM4310 (仅初始化)
 *   底盘: 留空
 *   发射: 无
 *   遥控器: 暂不接入
 */

#include "robot_control.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "test_def.h"
#include "gimbal_func.h"
#include "chassis_func.h"
#include "robot_func.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static Gimbal_Ctrl_Cmd_t  gimbal_cmd;
static Chassis_Ctrl_Cmd_t chassis_cmd;
static uint16_t           yaw_ecd;

static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        static uint32_t _can_chk = 0;
        if (++_can_chk % 500 == 0) {
            CAN_TypeDef *cx = CAN1;
            LOG_I("CAN TSR=0x%08lx ESR=0x%08lx REC=%lu TEC=%lu",
                  cx->TSR, cx->ESR,
                  (cx->ESR >> 16) & 0xFF, (cx->ESR >> 24) & 0xFF);
        }
        /* 遥控器控制输入 (暂不接入) */
        // RemoteControlSet(&chassis_cmd, NULL, &gimbal_cmd);

        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);
        /* 底盘控制 */
        chassis_func(&chassis_cmd);

        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

    gimbal_init();
    chassis_init();

    status = tx_thread_create(&robot_control_thread, "robot_control_thread",
                              robot_control_task, 0,
                              robot_control_thread_stack, 1024,
                              30, 30, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_thread create failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
