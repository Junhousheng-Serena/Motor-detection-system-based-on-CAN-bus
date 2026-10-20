#ifndef USER_DRV_CAN_H
#define USER_DRV_CAN_H

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"


HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length);

/* CAN 接收任务：由 CAN 中断通知，任务中完成串口打印。 */
void Can_ReceiveTask(void *argument);




#endif /* USER_DRV_CAN_H */
