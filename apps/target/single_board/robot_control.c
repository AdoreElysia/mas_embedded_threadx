/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: Target 机器人主控任务
 *
 * 电机配置:
 *   - DM6220: MIT模式, 软件速度环 (PID 在 MCU 侧算, 发力矩)
 *   - MF9025 ×2: 速度模式 (0xA2), 电机内部闭环, MCU 只发速度给定
 *
 * 遥控映射 (PS2):
 *   左摇杆 Y (LY): 前进/后退 → 两轮同向 (左+右+)
 *   右摇杆 X (RX): 旋转 → 两轮反向 (左+右- 或 左-右+)
 *   TRIANGLE 按钮: 切换 DM6220 档位 (0.4↔0.8 转/秒)
 *   R1 按钮: 切换 6220 ON/OFF
 *   L1 按钮: 切换 9025 ON/OFF
 *
 * 控制循环: 2ms, PS2 每 10ms 轮询一次 (5次分频)
 */

#include "robot_control.h"
#include "target_def.h"
#include "module_ps2.h"
#include "module_motor.h"
#include "module_offline.h"
#include "motor_damiao.h"
#include "motor_lingkong.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "bsp_dwt.h"

#define LOG_TAG "target_robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== 线程定义 ========== */
static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_stack[2048];

/* ========== 电机实例 ========== */
static DM_Motor_t *motor_6220 = NULL;
static LK_Motor_t *motor_9025_l = NULL;
static LK_Motor_t *motor_9025_r = NULL;

/* ========== 控制状态 ========== */
static volatile uint8_t mode_6220_on  = 0;  /* 6220 使能开关 */
static volatile uint8_t mode_9025_on  = 0;  /* 9025 使能开关 */
static volatile uint8_t speed_level   = 0;  /* 6220 档位: 0=低速 0.4, 1=高速 0.8 */

/* ========== PS2 轮询分频 ========== */
static uint8_t ps2_poll_counter = 0;

/* ========== PS2 按钮回调 ========== */

/* TRIANGLE (bit 12): 切换 6220 档位 */
static void cb_toggle_6220_speed(void)
{
    speed_level ^= 1;
    LOG_I("6220 speed level: %d (%.1f rev/s)", speed_level, speed_level ? 0.8f : 0.4f);
}

/* R1 (bit 11): 切换 6220 ON/OFF */
static void cb_toggle_6220(void)
{
    mode_6220_on ^= 1;
    if (mode_6220_on)
    {
        Motor_DM_Start(motor_6220);
        LOG_I("6220 ON");
    }
    else
    {
        Motor_DM_Stop(motor_6220);
        LOG_I("6220 OFF");
    }
}

/* L1 (bit 10): 切换 9025 ON/OFF */
static void cb_toggle_9025(void)
{
    mode_9025_on ^= 1;
    if (mode_9025_on)
    {
        Motor_LK_Start(motor_9025_l);
        Motor_LK_Start(motor_9025_r);
        LOG_I("9025 ON");
    }
    else
    {
        Motor_LK_Stop(motor_9025_l);
        Motor_LK_Stop(motor_9025_r);
        LOG_I("9025 OFF");
    }
}

/* ========== 主控任务 ========== */
static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        /* ── PS2 轮询 (每 10ms, 5次分频) ── */
        if (++ps2_poll_counter >= 5)
        {
            ps2_poll_counter = 0;
            Module_PS2_Poll();
        }

        /* ── 安全检查: 离线则全部停 ── */
        if (!Module_PS2_IsOnline())
        {
            /* 遥控器离线: 所有电机清零 */
            if (motor_6220)
                Motor_DM_SetRef(motor_6220, 0.0f);
            if (motor_9025_l)
                Motor_LK_SetRef(motor_9025_l, 0.0f);
            if (motor_9025_r)
                Motor_LK_SetRef(motor_9025_r, 0.0f);
            tx_thread_sleep(2);
            continue;
        }

        PS2_Handle_t *ps2 = Module_PS2_GetHandle();

        /* ── 9025 轮子控制 ── */
        if (mode_9025_on && motor_9025_l && motor_9025_r)
        {
            /*
             * 左摇杆 Y (LY): 前进/后退 → 两轮同向
             * 右摇杆 X (RX): 旋转 → 两轮反向
             *
             * 映射: (摇杆值 - 128) / 128 → [-1, +1]
             * 再乘以 MAX_WHEEL_SPEED_RAD → rad/s
             */
            float forward = (float)((int16_t)ps2->rx_ly - PS2_JOYSTICK_CENTER) / 128.0f;
            float turn    = (float)((int16_t)ps2->rx_rx - PS2_JOYSTICK_CENTER) / 128.0f;

            /* 差速驱动: 左轮 = forward + turn, 右轮 = forward - turn */
            float left_speed  = (forward + turn) * MAX_WHEEL_SPEED_RAD;
            float right_speed = (forward - turn) * MAX_WHEEL_SPEED_RAD;

            /* 限幅 */
            if (left_speed > MAX_WHEEL_SPEED_RAD)  left_speed = MAX_WHEEL_SPEED_RAD;
            if (left_speed < -MAX_WHEEL_SPEED_RAD) left_speed = -MAX_WHEEL_SPEED_RAD;
            if (right_speed > MAX_WHEEL_SPEED_RAD)  right_speed = MAX_WHEEL_SPEED_RAD;
            if (right_speed < -MAX_WHEEL_SPEED_RAD) right_speed = -MAX_WHEEL_SPEED_RAD;

            Motor_LK_SetRef(motor_9025_l, left_speed);
            Motor_LK_SetRef(motor_9025_r, right_speed);
        }
        else
        {
            if (motor_9025_l) Motor_LK_SetRef(motor_9025_l, 0.0f);
            if (motor_9025_r) Motor_LK_SetRef(motor_9025_r, 0.0f);
        }

        /* ── 6220 顶部旋转控制 ── */
        if (mode_6220_on && motor_6220)
        {
            float dm_speed = speed_level ? DM6220_SPEED_HIGH : DM6220_SPEED_LOW;
            Motor_DM_SetRef(motor_6220, dm_speed);
        }
        else
        {
            if (motor_6220) Motor_DM_SetRef(motor_6220, 0.0f);
        }

        tx_thread_sleep(2);
    }
}

/* ========== 初始化 ========== */
void robot_control_init(void)
{
    UINT status;

    /* ── 初始化 PS2 模块 ── */
    Module_PS2_Init();

    /* ── 绑定 PS2 按钮回调 ── */
    Module_PS2_BindButton(12, PS2_TRIGGER_EDGE_PRESSED, cb_toggle_6220_speed); /* TRIANGLE */
    Module_PS2_BindButton(11, PS2_TRIGGER_EDGE_PRESSED, cb_toggle_6220);       /* R1 */
    Module_PS2_BindButton(10, PS2_TRIGGER_EDGE_PRESSED, cb_toggle_9025);       /* L1 */

    /* ── 注册 DM6220 (达妙, MIT模式, 软件速度环) ── */
    Motor_Init_Config_s dm6220_cfg = {0};

    dm6220_cfg.motor_init_info.motor_type       = DM6220;
    dm6220_cfg.motor_init_info.gear_ratio       = 1.0f;
    dm6220_cfg.motor_init_info.torque_constant  = 0.0f;  /* TODO: 待测量 */
    dm6220_cfg.motor_init_info.max_torque       = 10.0f;

    dm6220_cfg.setting_init_config.loop_type             = SPEED_LOOP;
    dm6220_cfg.setting_init_config.algorithm_type        = CONTROL_PID;
    dm6220_cfg.setting_init_config.enableflag            = 0;  /* 初始禁用, 由按钮开启 */
    dm6220_cfg.setting_init_config.motor_reverse_flag    = 0;
    dm6220_cfg.setting_init_config.feedback_reverse_flag = 0;
    dm6220_cfg.setting_init_config.angle_feedback_source = 0;
    dm6220_cfg.setting_init_config.speed_feedback_source = 0;

    /* PID 参数全 0, 用户自行整定 */
    dm6220_cfg.controller_init_config.speed_PID.Kp = 0.0f;
    dm6220_cfg.controller_init_config.speed_PID.Ki = 0.0f;
    dm6220_cfg.controller_init_config.speed_PID.Kd = 0.0f;
    dm6220_cfg.controller_init_config.speed_PID.MaxOut = 10.0f;
    dm6220_cfg.controller_init_config.speed_PID.DeadBand = 0.0f;
    dm6220_cfg.controller_init_config.speed_PID.Improve = PID_IMPROVE_NONE;

    dm6220_cfg.controller_init_config.angle_PID.Kp = 0.0f;
    dm6220_cfg.controller_init_config.angle_PID.Ki = 0.0f;
    dm6220_cfg.controller_init_config.angle_PID.Kd = 0.0f;
    dm6220_cfg.controller_init_config.angle_PID.MaxOut = 0.0f;
    dm6220_cfg.controller_init_config.angle_PID.DeadBand = 0.0f;
    dm6220_cfg.controller_init_config.angle_PID.Improve = PID_IMPROVE_NONE;

    dm6220_cfg.controller_init_config.other_angle_feedback_ptr = NULL;
    dm6220_cfg.controller_init_config.other_speed_feedback_ptr = NULL;

    dm6220_cfg.offline_init_config.name       = "dm6220";
    dm6220_cfg.offline_init_config.timeout_ms = MOTOR_OFFLINE_TIMEOUT_MS;
    dm6220_cfg.offline_init_config.beep_times = 3;
    dm6220_cfg.offline_init_config.enable     = 1;

    dm6220_cfg.transport = MOTOR_TRANSPORT_CAN;
    dm6220_cfg.transport_config.can.hcan      = TARGET_CAN_BUS;
    dm6220_cfg.transport_config.can.tx_id     = DM6220_CAN_TX_ID;
    dm6220_cfg.transport_config.can.rx_id     = DM6220_CAN_RX_ID;
    dm6220_cfg.transport_config.can.rx_callback = NULL;
    dm6220_cfg.transport_config.can.user_arg    = NULL;

    motor_6220 = Motor_DM_Init(&dm6220_cfg, DM_MIT_MODE);
    if (motor_6220 == NULL)
    {
        LOG_E("DM6220 init failed!");
    }

    /* ── 注册 MF9025 左轮 (零控, 速度模式) ── */
    Motor_Init_Config_s lk_left_cfg = {0};

    lk_left_cfg.motor_init_info.motor_type       = MF9025;
    lk_left_cfg.motor_init_info.gear_ratio       = 1.0f;
    lk_left_cfg.motor_init_info.torque_constant  = 0.32f;
    lk_left_cfg.motor_init_info.max_torque       = 2.42f;

    lk_left_cfg.setting_init_config.loop_type             = SPEED_LOOP;
    lk_left_cfg.setting_init_config.algorithm_type        = CONTROL_PID;
    lk_left_cfg.setting_init_config.enableflag            = 0;
    lk_left_cfg.setting_init_config.motor_reverse_flag    = 0;
    lk_left_cfg.setting_init_config.feedback_reverse_flag = 0;
    lk_left_cfg.setting_init_config.angle_feedback_source = 0;
    lk_left_cfg.setting_init_config.speed_feedback_source = 0;

    lk_left_cfg.controller_init_config.speed_PID.Kp = 0.0f;
    lk_left_cfg.controller_init_config.speed_PID.Ki = 0.0f;
    lk_left_cfg.controller_init_config.speed_PID.Kd = 0.0f;
    lk_left_cfg.controller_init_config.speed_PID.MaxOut = 0.0f;
    lk_left_cfg.controller_init_config.speed_PID.DeadBand = 0.0f;
    lk_left_cfg.controller_init_config.speed_PID.Improve = PID_IMPROVE_NONE;

    lk_left_cfg.controller_init_config.angle_PID.Kp = 0.0f;
    lk_left_cfg.controller_init_config.angle_PID.Ki = 0.0f;
    lk_left_cfg.controller_init_config.angle_PID.Kd = 0.0f;
    lk_left_cfg.controller_init_config.angle_PID.MaxOut = 0.0f;
    lk_left_cfg.controller_init_config.angle_PID.DeadBand = 0.0f;
    lk_left_cfg.controller_init_config.angle_PID.Improve = PID_IMPROVE_NONE;

    lk_left_cfg.controller_init_config.other_angle_feedback_ptr = NULL;
    lk_left_cfg.controller_init_config.other_speed_feedback_ptr = NULL;

    lk_left_cfg.offline_init_config.name       = "mf9025_l";
    lk_left_cfg.offline_init_config.timeout_ms = MOTOR_OFFLINE_TIMEOUT_MS;
    lk_left_cfg.offline_init_config.beep_times = 3;
    lk_left_cfg.offline_init_config.enable     = 1;

    lk_left_cfg.transport = MOTOR_TRANSPORT_CAN;
    lk_left_cfg.transport_config.can.hcan      = TARGET_CAN_BUS;
    lk_left_cfg.transport_config.can.tx_id     = 0x140 + MF9025_LEFT_ID;
    lk_left_cfg.transport_config.can.rx_id     = 0x140 + MF9025_LEFT_ID;
    lk_left_cfg.transport_config.can.rx_callback = NULL;
    lk_left_cfg.transport_config.can.user_arg    = NULL;

    motor_9025_l = Motor_LK_Init(&lk_left_cfg);
    if (motor_9025_l == NULL)
    {
        LOG_E("MF9025 left init failed!");
    }

    /* ── 注册 MF9025 右轮 (零控, 速度模式) ── */
    Motor_Init_Config_s lk_right_cfg = lk_left_cfg; /* 复制左轮配置 */

    lk_right_cfg.offline_init_config.name       = "mf9025_r";
    lk_right_cfg.transport_config.can.tx_id     = 0x140 + MF9025_RIGHT_ID;
    lk_right_cfg.transport_config.can.rx_id     = 0x140 + MF9025_RIGHT_ID;

    motor_9025_r = Motor_LK_Init(&lk_right_cfg);
    if (motor_9025_r == NULL)
    {
        LOG_E("MF9025 right init failed!");
    }

    /* ── 创建控制线程 ── */
    status = tx_thread_create(&robot_control_thread, "target_robot", robot_control_task, 0,
                              robot_control_stack, 2048, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("Failed to create robot_control_thread!");
        return;
    }

    LOG_I("Target robot control initialized");
}
