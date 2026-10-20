#ifndef TEST_COMM_CAN_H
#define TEST_COMM_CAN_H
#include "main.h"
typedef struct { unsigned unused; } CAN_HandleTypeDef;
typedef struct
{
    uint32_t StdId, ExtId, RTR, IDE, DLC, TransmitGlobalTime;
} CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
extern CAN_HandleTypeDef hcan;
#define CAN_RX_FIFO0 0U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_ID_STD 0U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 2U
#define DISABLE 0U
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *);
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, const CAN_TxHeaderTypeDef *,
                                      const uint8_t *, uint32_t *);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t,
                                      CAN_RxHeaderTypeDef *, uint8_t *);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
#endif
