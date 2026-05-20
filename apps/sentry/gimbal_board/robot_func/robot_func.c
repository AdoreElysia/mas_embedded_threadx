#include "robot_func.h"
#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "sentry_def.h"
#include "user_lib.h"

float CalcOffsetAngle(float getyawangle)
{
    static float offset_ecd;

    // 定义编码器范围常量
    const float ECD_MAX  = 8191.0f; // 编码器最大值
    const float ECD_HALF = 4095.5f; // 编码器中点 (对应180度)

    // 计算原始偏差 (单位: 编码器值)
    offset_ecd = getyawangle - YAW_CHASSIS_ALIGN_ECD;

    // 归一化处理 (使其在 -4095.5 到 4095.5 之间)，即将偏差映射到最近的路径，处理过零点问题
    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return offset_ecd * (2.0f * 3.14159265354f) / ECD_MAX;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    /* 双离线：全部安全 */
    if (state == 0)
    {
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Shoot_Ctrl->shoot_mode     = shoot_off;
        Shoot_Ctrl->friction_mode  = friction_off;
        Shoot_Ctrl->load_mode      = load_stop;
        memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
        return;
    }

    /* RC 在线 */
    if (state & 0x01)
    {
        /* 摇杆 → 底盘速度 (通用) */
        Chassis_Ctrl->vx = 0.004f * (float)Module_Remote_get_channel(2);
        Chassis_Ctrl->vy = -0.004f * (float)Module_Remote_get_channel(1);

#if (REMOTE_SOURCE == 1)
        {
            int16_t ch6 = Module_Remote_get_channel(6);
            if (ch6 == SBUS_CHX_UP)
            {
                Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
                Gimbal_Ctrl->yaw -= 0.001f * (float)Module_Remote_get_channel(4);
                Gimbal_Ctrl->pitch += 0.001f * (float)Module_Remote_get_channel(3);
                VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
            }
            else if (ch6 == SBUS_CHX_BIAS)
            {
                Gimbal_Ctrl->gimbal_mode   = gimbal_auto_mode;
                Chassis_Ctrl->chassis_mode = chassis_automode;
            }
            else if (ch6 == SBUS_CHX_DOWN)
            {
                Gimbal_Ctrl->gimbal_mode  = gimbal_zero_force;
                Shoot_Ctrl->shoot_mode    = shoot_off;
                Shoot_Ctrl->friction_mode = friction_off;
                Shoot_Ctrl->load_mode     = load_stop;
            }

            // 发射机构控制部分（分2种遥控器）
            int16_t ch7 = Module_Remote_get_channel(7);
            if (ch7 == SBUS_CHX_UP)
            {
                Shoot_Ctrl->shoot_mode    = shoot_off;
                Shoot_Ctrl->friction_mode = friction_off;
                Shoot_Ctrl->load_mode     = load_stop;
            }
            else if (ch7 == SBUS_CHX_BIAS)
            {
                Shoot_Ctrl->shoot_mode    = shoot_off;
                Shoot_Ctrl->friction_mode = friction_on;
            }
            else if (ch7 == SBUS_CHX_DOWN)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_on;
                if (ch7 == SBUS_CHX_BIAS)
                    Shoot_Ctrl->load_mode = load_1_bullet;
                else if (ch7 == SBUS_CHX_DOWN)
                    Shoot_Ctrl->load_mode = load_burstfire;
            }

            int16_t ch8 = Module_Remote_get_channel(8);
            if (ch8 == SBUS_CHX_UP)
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
            else if (ch8 == SBUS_CHX_BIAS)
                Chassis_Ctrl->chassis_mode = chassis_rotate;
            else if (ch8 == SBUS_CHX_DOWN)
                Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;
        }

#elif (REMOTE_SOURCE == 2)
        /* DT7 */
        {
            dt7_custom_t *dt7_custom = Module_Remote_get_dt7_custom();
            if (dt7_custom == NULL)
            {
                Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
                Chassis_Ctrl->chassis_mode = chassis_zero_force;
                Shoot_Ctrl->shoot_mode     = shoot_off;
                Shoot_Ctrl->friction_mode  = friction_off;
                Shoot_Ctrl->load_mode      = load_stop;
                memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
                return;
            }

            if (dt7_custom->sw1 == DT7_SW_DOWN)
            {
                Gimbal_Ctrl->gimbal_mode   = gimbal_auto_mode;
                Chassis_Ctrl->chassis_mode = chassis_automode;
                Shoot_Ctrl->shoot_mode     = shoot_on;
                Shoot_Ctrl->friction_mode  = friction_on;
            }
            else
            {
                Gimbal_Ctrl->gimbal_mode  = gimbal_gyro_mode;
                Shoot_Ctrl->shoot_mode    = shoot_off;
                Shoot_Ctrl->friction_mode = friction_off;
            }

            Gimbal_Ctrl->yaw -= 0.001f * (float)(Module_Remote_get_channel(3));
            Gimbal_Ctrl->pitch += 0.001f * (float)(Module_Remote_get_channel(4));
            VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);

            // 发射机构控制部分
            if (dt7_custom->sw2 == DT7_SW_UP)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_off;
                Shoot_Ctrl->load_mode     = load_stop;

                int16_t wheel_value = dt7_custom->wheel;
                if (wheel_value == 0)
                {
                    Shoot_Ctrl->load_mode = load_stop;
                }
                else if (wheel_value > 0)
                {
                    Shoot_Ctrl->load_mode = load_reverse;
                }
                else if (wheel_value < 0)
                {
                    Shoot_Ctrl->load_mode = load_burstfire;
                }
            }
            else if (dt7_custom->sw1 == DT7_SW_MID)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_off;
            }

            // 底盘控制部分
            if (dt7_custom->sw2 == DT7_SW_UP)
            {
                Chassis_Ctrl->chassis_mode = chassis_rotate;
            }
            else if (dt7_custom->sw2 == DT7_SW_MID)
            {
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
            }
            else if (dt7_custom->sw2 == DT7_SW_DOWN)
            {
                Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;
            }
        }
#endif
    }

    /* VT 在线 (可覆盖 RC)*/
    if (state & 0x02)
    {
#if (REMOTE_VT_SOURCE == 2)
        static uint8_t trigger_state      = 0; // 0: 关闭状态, 1: 开启状态
        static uint8_t chassis_mode_state = 0; // 0: FOLLOW_GIMBAL_YAW, 1: ROTATE, 2: ROTATE_REVERSE

        vt03_custom_t *vt03 = Module_Remote_get_vt03_custom();
        if (vt03 == NULL)
        {
            Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
            Shoot_Ctrl->shoot_mode     = shoot_off;
            Shoot_Ctrl->friction_mode  = friction_off;
            Shoot_Ctrl->load_mode      = load_stop;
            memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
            return;
        }

        // 云台控制部分
        uint8_t switch_pos = vt03->switch_pos;
        if (switch_pos == 0)
        {
            Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
            Gimbal_Ctrl->yaw -= 0.001f * (float)(Module_Remote_get_channel(4));
            Gimbal_Ctrl->pitch += 0.001f * (float)(Module_Remote_get_channel(3));
            VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
        }
        else if (switch_pos == 1)
        {
            Gimbal_Ctrl->gimbal_mode   = gimbal_auto_mode;
            Chassis_Ctrl->chassis_mode = chassis_automode;
        }

        // 发射机构控制部分
        // 只有当按钮从松开(0)变为按下(1)时才切换状态
        static uint8_t last_trigger_state = 0; // 存储上一次的trigger状态
        uint8_t        current_trigger    = vt03->trigger;
        if (current_trigger == 1 && last_trigger_state == 0)
        {
            trigger_state = !trigger_state; // 切换状态

            if (trigger_state)
            { // 开启状态
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_on;

                int16_t wheel_value = vt03->wheel;
                if (wheel_value == 0)
                {
                    Shoot_Ctrl->load_mode = load_stop;
                }
                else if (wheel_value > 0)
                {
                    Shoot_Ctrl->load_mode = load_reverse;
                }
                else if (wheel_value < 0)
                {
                    Shoot_Ctrl->load_mode = load_burstfire;
                }
            }
            else
            { // 关闭状态
                Shoot_Ctrl->shoot_mode    = shoot_off;
                Shoot_Ctrl->friction_mode = friction_off;
                Shoot_Ctrl->load_mode     = load_stop;
            }
        }

        // 底盘控制部分
        static uint8_t last_custom_right_state = 0; // 存储上一次的custom_right状态
        uint8_t        current_custom_right    = vt03->custom_right;
        if (current_custom_right == 1 && last_custom_right_state == 0)
        {
            chassis_mode_state = (chassis_mode_state + 1) % 3; // 循环切换三种模式

            switch (chassis_mode_state)
            {
            case 0:
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
                break;
            case 1:
                Chassis_Ctrl->chassis_mode = chassis_rotate;
                break;
            case 2:
                Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;
                break;
            }
        }

        // 更新上一次的按键状态
        last_trigger_state      = current_trigger;
        last_custom_right_state = current_custom_right;
#endif
    }
}
