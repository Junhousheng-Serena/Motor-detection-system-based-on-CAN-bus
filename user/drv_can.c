#include "drv_can.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "drv_print.h"

static TaskHandle_t can_receive_task_handle = NULL;
static CAN_RxHeaderTypeDef can_receive_header;
static uint8_t can_receive_data[8];


//发送CAN报文
HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId = id;              // 标准标识符（11位）
    tx_header.ExtId = 0x00;            // 扩展标识符（29位，不用）
    tx_header.RTR = CAN_RTR_DATA;      // 数据帧（非远程帧）
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.DLC = length;               // 数据长度（0~8字节）
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox);
}

// CAN 接收中断：只取帧并通知任务
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (can_handle != &hcan || can_receive_task_handle == NULL) {
        return;
    }

    if (HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0,
                             &can_receive_header, can_receive_data) == HAL_OK) {
        vTaskNotifyGiveFromISR(can_receive_task_handle, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

// CAN 接收任务：被中断通知后，在任务上下文中打印。
void Can_ReceiveTask(void *argument)
{
    (void)argument;
    can_receive_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 0U) {
            uint8_t length = (can_receive_header.DLC <= 8U) ?
                             (uint8_t)can_receive_header.DLC : 8U;

            my_printf("CAN Received: ID=0x%03X, Len=%u, Data=",
                      (unsigned int)can_receive_header.StdId,
                      (unsigned int)length);
            for (uint8_t i = 0U; i < length; i++) {
                my_printf("%02X ", (unsigned int)can_receive_data[i]);
            }
            my_printf("\r\n");
        }
    }
}
