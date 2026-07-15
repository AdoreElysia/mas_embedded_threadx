/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: PS2 遥控手柄底层驱动实现 (软件 SPI)
 *   使用 BSP_DWT_Delay 做微秒级延时, 替代原始空循环
 *   引脚: CS=PB12, CLK=PB13, DI=PB14, DO=PB15
 */
#include "ps2.h"
#include "bsp_dwt.h"
#include <string.h>

/* ========== 寄存器级 GPIO 操作 (C板可配置IO) ========== */
#define PS2_CS_H()  (GPIOB->BSRR = GPIO_PIN_12)
#define PS2_CS_L()  (GPIOB->BRR  = GPIO_PIN_12)
#define PS2_CLK_H() (GPIOB->BSRR = GPIO_PIN_13)
#define PS2_CLK_L() (GPIOB->BRR  = GPIO_PIN_13)
#define PS2_DO_H()  (GPIOB->BSRR = GPIO_PIN_15)
#define PS2_DO_L()  (GPIOB->BRR  = GPIO_PIN_15)
#define PS2_DI()    ((GPIOB->IDR & GPIO_PIN_14) != 0)

/* ========== 软件 SPI 单字节收发 ========== */
static uint8_t PS2_Cmd(uint8_t CMD)
{
    uint8_t byte_rx = 0;

    for (uint16_t ref = 0x01; ref < 0x0100; ref <<= 1)
    {
        if (ref & CMD)
            PS2_DO_H();
        else
            PS2_DO_L();

        PS2_CLK_H();
        BSP_DWT_Delay(0.000005f); /* 5us */
        PS2_CLK_L();
        BSP_DWT_Delay(0.000005f); /* 5us */
        PS2_CLK_H();

        if (PS2_DI())
            byte_rx |= ref;
    }
    BSP_DWT_Delay(0.000016f); /* 16us 字间隔 */
    return byte_rx;
}

/* ========== 内部配置命令 ========== */
static void PS2_ShortPoll(void)
{
    PS2_CS_L();
    BSP_DWT_Delay(0.000016f);
    PS2_Cmd(0x01);
    PS2_Cmd(0x42);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_CS_H();
    BSP_DWT_Delay(0.000016f);
}

static void PS2_EnterConfig(void)
{
    PS2_CS_L();
    BSP_DWT_Delay(0.000016f);
    PS2_Cmd(0x01);
    PS2_Cmd(0x43);
    PS2_Cmd(0x00);
    PS2_Cmd(0x01);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_CS_H();
    BSP_DWT_Delay(0.000016f);
}

static void PS2_TurnOnAnalogMode(void)
{
    PS2_CS_L();
    BSP_DWT_Delay(0.000016f);
    PS2_Cmd(0x01);
    PS2_Cmd(0x44);
    PS2_Cmd(0x00);
    PS2_Cmd(0x01); /* 0x01: 模拟模式 */
    PS2_Cmd(0x03); /* 锁存, 防止 MODE 键切换 */
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_CS_H();
    BSP_DWT_Delay(0.000016f);
}

static void PS2_ExitConfig(void)
{
    PS2_CS_L();
    BSP_DWT_Delay(0.000016f);
    PS2_Cmd(0x01);
    PS2_Cmd(0x43);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_CS_H();
    BSP_DWT_Delay(0.000016f);
}

/* ========== 对外 API ========== */

void PS2_Init(PS2_Handle_t *handle)
{
    if (handle == NULL) return;

    memset(handle, 0, sizeof(PS2_Handle_t));

    /* GPIO 初始化: PB12(CS), PB13(CLK), PB15(DO) 推挽输出; PB14(DI) 输入上拉 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 输出引脚: CS, CLK, DO */
    GPIO_InitStruct.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 输入引脚: DI */
    GPIO_InitStruct.Pin  = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 默认空闲状态 */
    PS2_CS_H();
    PS2_CLK_H();
    BSP_DWT_Delay(0.01f); /* 10ms 上电等待 */

    /* PS2 配置序列: 进入模拟模式 */
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_EnterConfig();
    PS2_TurnOnAnalogMode();
    PS2_ExitConfig();

    /* 初始化发送缓冲 */
    handle->tx_buf[0] = 0x01;  /* 起始命令 */
    handle->tx_buf[1] = 0x42;  /* 请求数据 */
    handle->tx_buf[2] = 0x00;
    handle->tx_buf[3] = 0x00;  /* Motor 1 */
    handle->tx_buf[4] = 0x00;  /* Motor 2 */
    handle->tx_buf[5] = 0x00;
    handle->tx_buf[6] = 0x00;
    handle->tx_buf[7] = 0x00;
    handle->tx_buf[8] = 0x00;

    /* 摇杆默认中心值 */
    handle->rx_rx = PS2_JOYSTICK_CENTER;
    handle->rx_ry = PS2_JOYSTICK_CENTER;
    handle->rx_lx = PS2_JOYSTICK_CENTER;
    handle->rx_ly = PS2_JOYSTICK_CENTER;

    handle->poll_routine = PS2_Poll_Routine;
    handle->rx_callback  = NULL;
}

void PS2_RegisterRxCallback(PS2_Handle_t *handle, void (*callback)(PS2_Handle_t *))
{
    if (handle)
        handle->rx_callback = callback;
}

void PS2_SetVibration(PS2_Handle_t *handle, uint8_t motor1, uint8_t motor2)
{
    if (handle)
    {
        handle->motor1_val = motor1;
        handle->motor2_val = motor2;
    }
}

void PS2_Poll_Routine(PS2_Handle_t *handle)
{
    if (handle == NULL) return;

    handle->tx_buf[3] = handle->motor1_val;
    handle->tx_buf[4] = handle->motor2_val;

    /* CS 拉低, 开始通信 */
    PS2_CS_L();
    BSP_DWT_Delay(0.000016f);

    /* 9 字节全双工收发 */
    for (uint8_t i = 0; i < 9; i++)
    {
        handle->rx_buf[i] = PS2_Cmd(handle->tx_buf[i]);
    }

    /* CS 拉高, 结束通信 */
    PS2_CS_H();
    BSP_DWT_Delay(0.000016f);

    /* 校验应答字节 */
    if (handle->rx_buf[1] == 0x5A || handle->rx_buf[1] == 0x73 || handle->rx_buf[1] == 0x41)
    {
        handle->is_connected = 1;

        /* 按键解析 (按下=0, 取反后按下=1) */
        uint16_t key_raw = ((uint16_t)handle->rx_buf[4] << 8) | handle->rx_buf[3];
        handle->buttons = (~key_raw) & 0xFFFF;

        /* 摇杆值 */
        handle->rx_rx = handle->rx_buf[5];
        handle->rx_ry = handle->rx_buf[6];
        handle->rx_lx = handle->rx_buf[7];
        handle->rx_ly = handle->rx_buf[8];

        if (handle->rx_callback != NULL)
            handle->rx_callback(handle);
    }
    else
    {
        handle->is_connected = 0;
        handle->buttons = 0;
        handle->rx_rx = PS2_JOYSTICK_CENTER;
        handle->rx_ry = PS2_JOYSTICK_CENTER;
        handle->rx_lx = PS2_JOYSTICK_CENTER;
        handle->rx_ly = PS2_JOYSTICK_CENTER;
    }
}
