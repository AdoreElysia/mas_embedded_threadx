/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2026-04-14 23:19:28
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2026-04-14 23:25:29
 * @FilePath: \mas\utils\utils_init.c
 * @Description:
 */
#include "utils_init.h"

#include "ulog.h"
#include "SEGGER_SYSVIEW.h"

void UTILS_Init(void)
{
    ulog_init();           // RTT must be initialized first
    SEGGER_SYSVIEW_Conf(); // SystemView configures RTT channel 1
}
