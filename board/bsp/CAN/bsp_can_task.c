#include "tx_api.h"
#include "tx_port.h"
#include "bsp_can.h"
#include "bsp_can_task.h"
#include "bsp_def.h"

#define LOG_TAG "bsp_can_task"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define BSP_CAN_RX_TASK_PRIORITY 3
#define BSP_CAN_TX_TASK_PRIORITY 4
#define BSP_CAN_TASK_STACK_SIZE  1024U

TX_EVENT_FLAGS_GROUP g_can_event_flags;

static TX_THREAD                  g_can_rx_thread;
static TX_THREAD                  g_can_tx_thread;
APPS_STACK_SECTION static uint8_t g_can_rx_stack[BSP_CAN_TASK_STACK_SIZE];
APPS_STACK_SECTION static uint8_t g_can_tx_stack[BSP_CAN_TASK_STACK_SIZE];

/* RX 任务 */
static void can_rx_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        tx_event_flags_get(&g_can_event_flags, CAN_EVENT_RX, TX_OR_CLEAR, NULL, TX_WAIT_FOREVER);

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
        {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].rx_fifo, &msg))
            {
                Can_Device *dev = can_dev_find(&g_can_bus[i], msg.id);
                if (dev != NULL && dev->rx_callback != NULL) dev->rx_callback(dev, msg.data, msg.len);
            }
        }
    }
}

/* TX 任务 */
static void can_tx_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        tx_event_flags_get(&g_can_event_flags, CAN_EVENT_TX, TX_OR_CLEAR, NULL, TX_WAIT_FOREVER);

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
        {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].tx_fifo, &msg))
            {
#if defined(STM32H723xx)
                FDCAN_TxHeaderTypeDef TxHeader = {
                    .Identifier          = msg.id,
                    .IdType              = FDCAN_STANDARD_ID,
                    .TxFrameType         = FDCAN_DATA_FRAME,
                    .DataLength          = (msg.len > 8) ? FDCAN_DLC_BYTES_8 : msg.len,
                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                    .BitRateSwitch       = FDCAN_BRS_OFF,
                    .FDFormat            = FDCAN_CLASSIC_CAN,
                };
                while (HAL_FDCAN_AddMessageToTxFifoQ(msg.hcan, &TxHeader, msg.data) != HAL_OK) tx_thread_sleep(1);
#elif defined(STM32F407xx)
                CAN_TxHeaderTypeDef TxHeader = {.StdId = msg.id, .DLC = msg.len, .RTR = CAN_RTR_DATA, .IDE = CAN_ID_STD};
                uint32_t            mailbox;
                while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK) tx_thread_sleep(1);
#endif
            }
        }
    }
}

/* 对外函数 */

void BSP_CAN_TaskInit(void)
{
    if (tx_event_flags_create(&g_can_event_flags, "can_evt") != TX_SUCCESS)
    {
        LOG_E("event flags create failed");
        return;
    }

    if (tx_thread_create(&g_can_rx_thread, "CAN RX Task", can_rx_task_entry, 0, g_can_rx_stack, BSP_CAN_TASK_STACK_SIZE, BSP_CAN_RX_TASK_PRIORITY,
                         BSP_CAN_RX_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("rx task create failed");
        return;
    }

    if (tx_thread_create(&g_can_tx_thread, "CAN TX Task", can_tx_task_entry, 0, g_can_tx_stack, BSP_CAN_TASK_STACK_SIZE, BSP_CAN_TX_TASK_PRIORITY,
                         BSP_CAN_TX_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("tx task create failed");
        return;
    }

    LOG_I("CAN tasks ready");
}
