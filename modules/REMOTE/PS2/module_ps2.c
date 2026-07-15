/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: PS2 模块层实现
 *   离线检测逻辑: is_connected=1 且 4 个摇杆值不全为 128 时才更新心跳
 */
#include "module_ps2.h"
#include "bsp_dwt.h"
#include <string.h>

#define LOG_TAG "module_ps2"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 全局 PS2 句柄 */
PS2_Handle_t g_ps2_handle;

/* 离线检测设备 */
static Offline_Device *g_ps2_offline = NULL;

/* 按钮回调表 (16 个 bit) */
static PS2_BtnEventConf_t g_btn_callbacks[16];

/* 上一帧按键状态 (用于边沿检测) */
static uint16_t last_buttons_state = 0;

/* 离线检测配置 */
#ifndef PS2_OFFLINE_TIMEOUT_MS
#define PS2_OFFLINE_TIMEOUT_MS 200
#endif

/**
 * @brief 接收回调: 按钮事件分发
 */
static void ps2_rx_complete_callback(PS2_Handle_t *handle)
{
    if (handle == NULL) return;

    /* 离线检测: 只有摇杆值不全为 128 时才更新心跳 */
    if (handle->rx_lx != PS2_JOYSTICK_CENTER || handle->rx_ly != PS2_JOYSTICK_CENTER ||
        handle->rx_rx != PS2_JOYSTICK_CENTER || handle->rx_ry != PS2_JOYSTICK_CENTER)
    {
        if (g_ps2_offline != NULL)
            Module_Offline_device_update(g_ps2_offline);
    }

    /* 按钮事件分发 */
    uint16_t current_state = handle->buttons;

    for (uint8_t i = 0; i < 16; i++)
    {
        if (g_btn_callbacks[i].callback != NULL)
        {
            uint8_t curr_bit = (current_state >> i) & 0x01;
            uint8_t last_bit = (last_buttons_state >> i) & 0x01;

            switch (g_btn_callbacks[i].mode)
            {
            case PS2_TRIGGER_EDGE_PRESSED:
                if (last_bit == 0 && curr_bit == 1)
                    g_btn_callbacks[i].callback();
                break;

            case PS2_TRIGGER_EDGE_RELEASED:
                if (last_bit == 1 && curr_bit == 0)
                    g_btn_callbacks[i].callback();
                break;

            case PS2_TRIGGER_HOLD:
                if (curr_bit == 1)
                    g_btn_callbacks[i].callback();
                break;
            }
        }
    }

    last_buttons_state = current_state;
}

void Module_PS2_Init(void)
{
    /* 初始化 PS2 硬件 */
    PS2_Init(&g_ps2_handle);

    /* 注册接收回调 */
    PS2_RegisterRxCallback(&g_ps2_handle, ps2_rx_complete_callback);

    /* 注册离线检测 */
    Offline_Init_config_t offline_config = {
        .name       = "ps2_joystick",
        .timeout_ms = PS2_OFFLINE_TIMEOUT_MS,
        .beep_times = 3,
        .enable     = 1,
    };
    g_ps2_offline = Module_Offline_register(&offline_config);

    LOG_I("PS2 module initialized");
}

void Module_PS2_Poll(void)
{
    if (g_ps2_handle.poll_routine != NULL)
    {
        g_ps2_handle.poll_routine(&g_ps2_handle);
    }

    /* 离线安全处理: 清零数据 */
    if (g_ps2_offline != NULL && Module_Offline_get_device_status(g_ps2_offline) == STATE_OFFLINE)
    {
        g_ps2_handle.buttons = 0;
        last_buttons_state   = 0;
        g_ps2_handle.rx_rx   = PS2_JOYSTICK_CENTER;
        g_ps2_handle.rx_ry   = PS2_JOYSTICK_CENTER;
        g_ps2_handle.rx_lx   = PS2_JOYSTICK_CENTER;
        g_ps2_handle.rx_ly   = PS2_JOYSTICK_CENTER;
    }
}

PS2_Handle_t *Module_PS2_GetHandle(void)
{
    return &g_ps2_handle;
}

uint8_t Module_PS2_IsOnline(void)
{
    if (g_ps2_offline == NULL)
        return 0;
    return Module_Offline_get_device_status(g_ps2_offline) == STATE_ONLINE ? 1 : 0;
}

void Module_PS2_BindButton(uint8_t bit_index, PS2_TriggerMode_e mode, PS2_ButtonCallback_t cb)
{
    if (bit_index < 16)
    {
        g_btn_callbacks[bit_index].mode     = mode;
        g_btn_callbacks[bit_index].callback = cb;
    }
}
