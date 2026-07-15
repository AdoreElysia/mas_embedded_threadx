/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: PS2 遥控手柄底层驱动 (软件 SPI)
 *   引脚映射到 C板可配置 IO:
 *     CS  → PB12 (SPI2_CS)
 *     CLK → PB13 (SPI2_CLK)
 *     DI  → PB14 (SPI2_MISO, 手柄→MCU)
 *     DO  → PB15 (SPI2_MOSI, MCU→手柄)
 */
#ifndef _PS2_H_
#define _PS2_H_

#include <stdint.h>

/* ========== 按键位定义 (bit 0-15, 按下=1) ========== */
#define PS2_KEY_SELECT    (1 << 0)
#define PS2_KEY_L3        (1 << 1)
#define PS2_KEY_R3        (1 << 2)
#define PS2_KEY_START     (1 << 3)
#define PS2_KEY_UP        (1 << 4)
#define PS2_KEY_RIGHT     (1 << 5)
#define PS2_KEY_DOWN      (1 << 6)
#define PS2_KEY_LEFT      (1 << 7)
#define PS2_KEY_L2        (1 << 8)
#define PS2_KEY_R2        (1 << 9)
#define PS2_KEY_L1        (1 << 10)
#define PS2_KEY_R1        (1 << 11)
#define PS2_KEY_TRIANGLE  (1 << 12)
#define PS2_KEY_CIRCLE    (1 << 13)
#define PS2_KEY_CROSS     (1 << 14)
#define PS2_KEY_SQUARE    (1 << 15)

/* 摇杆数据索引 */
#define PS2_RX_INDEX   5   /* 右摇杆 X */
#define PS2_RY_INDEX   6   /* 右摇杆 Y */
#define PS2_LX_INDEX   7   /* 左摇杆 X */
#define PS2_LY_INDEX   8   /* 左摇杆 Y */

#define PS2_JOYSTICK_CENTER  128

/* ========== 触发模式 ========== */
typedef enum
{
    PS2_TRIGGER_EDGE_PRESSED  = 0, /* 0→1 下降沿(按下) */
    PS2_TRIGGER_EDGE_RELEASED = 1, /* 1→0 上升沿(释放) */
    PS2_TRIGGER_HOLD          = 2, /* 持续按住 */
} PS2_TriggerMode_e;

/* ========== 按钮回调配置 ========== */
typedef struct
{
    PS2_TriggerMode_e mode;
    void (*callback)(void);
} PS2_BtnEventConf_t;

/* ========== PS2 句柄 ========== */
typedef struct PS2_Handle
{
    /* 收发缓冲 */
    uint8_t tx_buf[9];
    uint8_t rx_buf[9];

    /* 状态数据 */
    uint16_t buttons;       /* 16位按键状态 (按下=1) */
    uint8_t  rx_lx;         /* 左摇杆 X (0-255, 128=中心) */
    uint8_t  rx_ly;         /* 左摇杆 Y */
    uint8_t  rx_rx;         /* 右摇杆 X */
    uint8_t  rx_ry;         /* 右摇杆 Y */

    /* 振动电机 */
    uint8_t  motor1_val;    /* 右侧小震动 */
    uint8_t  motor2_val;    /* 左侧大震动 */

    /* 连接状态 */
    uint8_t  is_connected;  /* 1=通信正常, 0=丢失 */

    /* 轮询函数指针 */
    void (*poll_routine)(struct PS2_Handle *handle);

    /* 接收回调 */
    void (*rx_callback)(struct PS2_Handle *handle);
} PS2_Handle_t;

/* ========== API ========== */

/**
 * @brief 初始化 PS2 硬件 (GPIO + 配置模式)
 */
void PS2_Init(PS2_Handle_t *handle);

/**
 * @brief 轮询收发 (软件 SPI, 9 字节)
 */
void PS2_Poll_Routine(PS2_Handle_t *handle);

/**
 * @brief 注册接收回调
 */
void PS2_RegisterRxCallback(PS2_Handle_t *handle, void (*callback)(PS2_Handle_t *));

/**
 * @brief 设置振动
 */
void PS2_SetVibration(PS2_Handle_t *handle, uint8_t motor1, uint8_t motor2);

#endif /* _PS2_H_ */
