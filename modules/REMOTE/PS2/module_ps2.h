/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: PS2 模块层接口
 *   封装 PS2 底层驱动, 提供离线检测、按钮回调、统一数据访问
 *   不创建独立任务, 由 robot_control 轮询调用 Module_PS2_Poll()
 */
#ifndef _MODULE_PS2_H_
#define _MODULE_PS2_H_

#include "ps2.h"
#include "module_offline.h"
#include <stdint.h>

/* 按钮回调函数类型 */
typedef void (*PS2_ButtonCallback_t)(void);

/**
 * @brief 初始化 PS2 模块
 *   - GPIO 配置
 *   - PS2 进入模拟模式
 *   - 注册离线检测
 */
void Module_PS2_Init(void);

/**
 * @brief PS2 轮询 (由 robot_control 每 10ms 调用一次)
 *   - 收发数据
 *   - 按钮回调分发
 *   - 离线检测更新 (摇杆值逻辑)
 */
void Module_PS2_Poll(void);

/**
 * @brief 获取 PS2 数据句柄
 * @return PS2_Handle_t 指针
 */
PS2_Handle_t *Module_PS2_GetHandle(void);

/**
 * @brief 获取在线状态
 * @return 1=在线, 0=离线
 */
uint8_t Module_PS2_IsOnline(void);

/**
 * @brief 绑定按钮事件
 * @param bit_index 按键 bit 索引 (0-15)
 * @param mode 触发模式
 * @param cb 回调函数
 */
void Module_PS2_BindButton(uint8_t bit_index, PS2_TriggerMode_e mode, PS2_ButtonCallback_t cb);

#endif /* _MODULE_PS2_H_ */
