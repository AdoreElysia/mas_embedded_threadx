/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: 零控(LingKong) MF9025 电机驱动实现
 *   速度模式(0xA2)控制，电机内部闭环，MCU 只发速度给定
 */
#include "motor_lingkong.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_def.h"
#include "module_offline.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG "motor_lk"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* dps ↔ rad/s 转换 */
#define RAD_TO_DEG  (57.2957795131f)
#define DEG_TO_RAD  (0.01745329252f)

/* 编码器单圈范围 (16 bit) */
#define LK_ENCODER_MAX  65535.0f
#define LK_ENCODER_HALF 32768.0f
#define LK_ENCODER_RANGE (2.0f * 3.14159265359f) /* 2π rad */

/* CAN 接收回调 */
static void lk_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len < 8 || !dev->user_arg) return;
    LK_Motor_t *motor = (LK_Motor_t *)dev->user_arg;

    /* 解析反馈帧 (0x9C 或控制命令回复) */
    motor->measure.id          = data[0];
    motor->measure.temperature = (int8_t)data[1];
    motor->measure.current_raw = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));

    /* 速度: int16, 小端, 1 dps/LSB → 转换为 rad/s */
    int16_t speed_dps = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    motor->base.measure.speed_rad = (float)speed_dps * DEG_TO_RAD;

    /* 编码器位置: uint16, 小端 */
    uint16_t enc_pos = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
    motor->measure.encoder_pos = enc_pos;

    /* 单圈角度 (rad) */
    float current_angle = (float)enc_pos / LK_ENCODER_MAX * LK_ENCODER_RANGE;
    motor->base.measure.single_round_angle = current_angle;

    /* 多圈角度追踪 */
    float diff = current_angle - motor->measure.last_single_round_angle;
    if (diff < -LK_ENCODER_HALF / LK_ENCODER_MAX * LK_ENCODER_RANGE)
    {
        diff += LK_ENCODER_RANGE;
        motor->measure.total_round++;
    }
    else if (diff > LK_ENCODER_HALF / LK_ENCODER_MAX * LK_ENCODER_RANGE)
    {
        diff -= LK_ENCODER_RANGE;
        motor->measure.total_round--;
    }
    motor->base.measure.total_angle += diff;
    motor->measure.last_single_round_angle = current_angle;

    /* 转矩电流 → 力矩 (MF9025: 转矩常数 0.32 N·m/A, 分辨率 33/4096 A/LSB) */
    float current_amp = (float)motor->measure.current_raw * (33.0f / 4096.0f);
    motor->base.measure.torque_nm = current_amp * 0.32f;

    /* 更新离线检测 */
    Module_Offline_device_update(motor->base.offline_dev);
}

/* 发送命令帧 (使能/失能/停止) */
static void lk_send_cmd(LK_Motor_t *motor, LK_Motor_Cmd_e cmd)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan = bus->hcan;
    msg.id   = can_dev->tx_id;
    msg.len  = 8;

    msg.data[0] = (uint8_t)cmd;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    BSP_CAN_SendMessage(&msg);
}

/* 发送速度控制帧 (0xA2) */
static void lk_send_speed(LK_Motor_t *motor, float speed_rad)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan = bus->hcan;
    msg.id   = can_dev->tx_id;
    msg.len  = 8;

    /* rad/s → dps, int16 小端 */
    int16_t speed_dps = (int16_t)(speed_rad * RAD_TO_DEG);

    msg.data[0] = LK_CMD_SET_SPEED;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = (uint8_t)(speed_dps & 0xFF);
    msg.data[5] = (uint8_t)((speed_dps >> 8) & 0xFF);
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    BSP_CAN_SendMessage(&msg);
}

/* ControlAndSend 回调 (由 Motor_UpdateAll 调用) */
static void lk_ControlAndSend(Motor_Base *base)
{
    LK_Motor_t *motor = MOTOR_GET_DERIVED(base, LK_Motor_t);

    if (Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE || base->setting.enableflag == 0)
    {
        base->controller.output = 0;
        base->controller.ref    = 0;
        lk_send_speed(motor, 0.0f);
        return;
    }

    /* 速度模式: 直接把 ref(rad/s) 作为速度给定发给电机 */
    base->controller.output = base->controller.ref;
    lk_send_speed(motor, base->controller.ref);
}

/* 对外函数 */
LK_Motor_t *Motor_LK_Init(Motor_Init_Config_s *config)
{
    LK_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(LK_Motor_t), TX_NO_WAIT);
    if (motor == NULL)
    {
        LOG_E("Failed to allocate memory for LK motor");
        return NULL;
    }
    memset(motor, 0, sizeof(LK_Motor_t));

    /* 初始化基类字段 */
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;

    /* 注册 CAN 设备 */
    Can_Device *can_dev = BSP_CAN_Device_Init(&config->transport_config.can);
    if (can_dev == NULL)
    {
        LOG_E("Failed to initialize CAN device for LK motor");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->base.transport_dev = can_dev;

    /* 设置 CAN 接收回调 */
    can_dev->rx_callback = lk_can_rx_callback;
    can_dev->user_arg    = motor;

    /* 初始化控制器 (PID 参数全 0, 用户自行整定) */
    if (motor->base.setting.algorithm_type == CONTROL_PID)
    {
        PIDInit(&motor->base.controller.speed_PID, &config->controller_init_config.speed_PID);
        PIDInit(&motor->base.controller.angle_PID, &config->controller_init_config.angle_PID);
    }
    motor->base.controller.other_angle_feedback_ptr = config->controller_init_config.other_angle_feedback_ptr;
    motor->base.controller.other_speed_feedback_ptr = config->controller_init_config.other_speed_feedback_ptr;

    /* 离线检测 */
    motor->base.offline_dev = Module_Offline_register(&config->offline_init_config);

    /* 发送使能命令 */
    lk_send_cmd(motor, LK_CMD_MOTOR_RUN);
    BSP_DWT_Delay(0.0002f); /* 200us 间隔 */

    /* 注册到全局链表 */
    motor->base.ControlAndSend = lk_ControlAndSend;
    Motor_Register(&motor->base);

    LOG_I("LK motor initialized (type=%d)", motor->base.info.motor_type);

    return motor;
}

void Motor_LK_Start(LK_Motor_t *motor)
{
    if (motor == NULL) return;
    motor->base.setting.enableflag = 1;
    lk_send_cmd(motor, LK_CMD_MOTOR_RUN);
}

void Motor_LK_Stop(LK_Motor_t *motor)
{
    if (motor == NULL) return;
    motor->base.setting.enableflag = 0;
    motor->base.controller.ref = 0;
    lk_send_cmd(motor, LK_CMD_MOTOR_CLOSE);
}

void Motor_LK_SetRef(LK_Motor_t *motor, float ref)
{
    if (motor == NULL) return;
    motor->base.controller.ref = ref;
}
