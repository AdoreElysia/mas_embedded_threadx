/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-15 20:36:42
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-22 22:42:10
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\shoot_func\shoot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Description: 发射机构功能实现模板
 *
 * TODO: 实现发射逻辑
 *   1. 初始化摩擦轮电机、拨盘电机
 *   2. 根据 shoot_cmd 控制摩擦轮开关和拨盘动作
 */

#include "shoot_func.h"

#define LOG_TAG "app_shoot_func"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"
#include "module_offline.h"
#include "motor_dji.h"
#include "motor_damiao.h"
#include "motor_servo.h"
#include "shoot_mode.h"
#include "tim.h"
#include "user_lib.h"

static DJI_Motor_t *left_friction_motor;
static DJI_Motor_t *right_friction_motor;
static DM_Motor_t  *reload_motor;
static Servo_Motor_t *trigger = NULL;

static float    home_l, home_r;   /* 上电零点 = 待机位 (转子侧 rad) */
static uint16_t startup_cnt;      /* 上电先等过 offline 检测窗口, 再要求两轴有反馈 */
static bool     inited;           /* 上电零点已记, 两轴已使能过一次 */
static bool     spring_on = true; /* 回程簧力被机械结构接管后置 false */


void shoot_init(void)
{
    Motor_Init_Config_s friction_config = {
        .offline_init_config =
            {
                .name       = "3508_1", // 设备名称
                .timeout_ms = 100,      // 超时时间
                .beep_times = 1,        // 蜂鸣次数
                .enable     = 1,        // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan = BSP_CAN_HANDLE1,
            },
        .controller_init_config = 
            { 
                .lqr_init =
                    {
                        .K         = {FRICTION_LQR_K0, FRICTION_LQR_K1},
                        .state_dim = 2,
                    },
            },
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = ANGLE_AND_SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 19, .max_torque = 6, .torque_constant = 0.016f},
    };
    // 左摩擦轮
    friction_config.transport_config.can.tx_id = 1;
    left_friction_motor= Motor_DJI_Init(&friction_config);
    if (left_friction_motor == NULL)
    {
        LOG_E("friction_l init failed");
        return;
    }
    // 右摩擦轮
    friction_config.transport_config.can.tx_id     = 4; // 右摩擦轮,改txid和方向就行
    friction_config.offline_init_config.name       = "3508_2";
    friction_config.offline_init_config.beep_times = 4;
    right_friction_motor= Motor_DJI_Init(&friction_config);
    if (right_friction_motor == NULL)
    {
        LOG_E("friction_r init failed");
        return;
    }

    Motor_Init_Config_s servo_config = {
        .offline_init_config =
            {
                .name       = "trigger",
                .timeout_ms = 100,
                .beep_times = 0,
                .enable     = 0,
            },
        .transport = MOTOR_TRANSPORT_PWM,
        .transport_config.pwm =
            {
                .htim    = &htim8,
                .Channel = TIM_CHANNEL_1,
                .dutyx10 = 125,
                .Mode    = PWM_MODE_BLOCKING,
            },
        .controller_init_config = {0},
        .setting_init_config =
            {
                .loop_type      = ANGLE_LOOP,
                .algorithm_type = CONTROL_PID,
                .enableflag     = 0,
            },
        .motor_init_info =
            {
                .motor_type = SERVO_GENERIC,
                .gear_ratio = 1.0f,
            },
    };
    trigger = Motor_Servo_Init(&servo_config, 50, 250, 0, 90);
    if (trigger == NULL)
    {
        LOG_E("trigger init failed");
        return;
    }

    Motor_SetRef((Motor_Base *)trigger, TRIGGER_FIRE_DEG);
    Motor_Start((Motor_Base *)trigger);

    /* 3508 在 shoot_func() 里等首帧反馈、记下待机位零点之后才使能 */

    LOG_I("darts shoot init success");
}

/* 本轴行程 (输出侧度): 正 = 去程方向 (右轴镜像, 去程为负) */
static float friction_x_deg(const DJI_Motor_t *motor, float home)
{
    return (motor->base.measure.total_angle - home) * 180.0f / (PI * motor->base.info.gear_ratio);
}

/* 从待机位出发: 使能两轴 + 恢复拉簧前馈。ref 先回 home, 使能瞬间误差为 0 (同上电) */
static void friction_axis_engage(void)
{
    Motor_SetRef((Motor_Base *)left_friction_motor, home_l);
    Motor_SetRef((Motor_Base *)right_friction_motor, home_r);
    Motor_Start((Motor_Base *)left_friction_motor);
    Motor_Start((Motor_Base *)right_friction_motor);
    spring_on = true;
}

/* 拉簧前馈: 簧力始终反抗位移, 两轴同式 ff = K·x, 不按轴取反;
 * 回程簧力被机械结构接管后 spring_on = false, 前馈归零 */
static void friction_spring_compensate(void)
{
    float k = spring_on ? FRICTION_FF_K : 0.0f;

    left_friction_motor->base.controller.feedforward_torque  = k * friction_x_deg(left_friction_motor, home_l);
    right_friction_motor->base.controller.feedforward_torque = k * friction_x_deg(right_friction_motor, home_r);
}

/* 回程到位后泄力: 调它的时机由时序决定 (回程步里每周期调, 到位即生效) */
static void friction_release_at_home(void)
{
    if (!shoot_mode_3508_is_arrived(left_friction_motor, home_l) ||
        !shoot_mode_3508_is_arrived(right_friction_motor, home_r))
        return;

    Motor_Stop((Motor_Base *)left_friction_motor);
    Motor_Stop((Motor_Base *)right_friction_motor);
}

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    (void)shoot_cmd;

    /* 上电: 两轴都收到过反馈才能记零点, 否则使能瞬间会朝编码器绝对零位窜。
     * 注: offline 设备注册后头 100ms 恒报 ONLINE, 所以先等过 100ms 窗口 (75 × 2ms = 150ms) 再判状态 */
    if (!inited)
    {
        if (++startup_cnt < 75) return;

        if (Module_Offline_get_device_status(left_friction_motor->base.offline_dev) != STATE_ONLINE ||
            Module_Offline_get_device_status(right_friction_motor->base.offline_dev) != STATE_ONLINE)
            return;

        home_l = left_friction_motor->base.measure.total_angle;
        home_r = right_friction_motor->base.measure.total_angle;
        friction_axis_engage();
        inited = true;
    }

    friction_spring_compensate();

    /* TODO: 时序
     *   出发:   待机位先 friction_axis_engage(), 再给目标 = home ± 行程
     *           (行程(rad) = FRICTION_TRAVEL_DEG × PI/180 × gear_ratio, 右轴乘 FRICTION_RIGHT_SIGN)
     *   工作位: 保持目标, 前馈持续发力
     *   回程:   目标退回 home; |行程| < FRICTION_TAKEOVER_DEG 时 spring_on = false (簧力已被机械结构接管)
     *   到位:   回程步里每周期调 friction_release_at_home(), 到位即泄力, 下次出发前要重新 friction_axis_engage()
     *   扳机:   TRIGGER_READY_DEG 上膛 → 延时 → TRIGGER_FIRE_DEG 开火 (照 E 的时序)
     */
}
