/*
 * @Author: AdoreElysia
 * @Date: 2026-07-15
 * @Description: Target 机器人配置
 *   单板模式, 启用 OFFLINE + MOTOR + PS2
 */
# 先加载默认模板, 再覆盖差异
include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关 (单板)
set(MODULES_SINGLE OFFLINE MOTOR PS2)

# 不需要 BMI088 / INS / REFEREE / SUPERCAP / VISION / BOARDCOMM / REMOTE
# PS2 作为独立模块, 通过 MODULE_PS2 开关控制
set(MODULE_PS2 1)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE 0)

# MOTOR 参数
set(MOTOR_TASK_STACK_SIZE 1024)
set(MOTOR_TASK_PRIORITY   12)

# PS2 参数
set(PS2_OFFLINE_TIMEOUT_MS 200)
