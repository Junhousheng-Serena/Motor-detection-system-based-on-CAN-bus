#ifndef USER_DRV_CAN_H
#define USER_DRV_CAN_H

#include "main.h"
#include "FreeRTOS.h"
#include "can_protocol.h"

typedef struct
{
    uint32_t rxDropped;
    uint32_t txFailed;
    uint8_t peerOnline;
    uint8_t commandSeen, commandFresh, commandSequence;
} CanDiagnostics;

/* Create the RX queue before the scheduler; CAN starts inside the task. */
HAL_StatusTypeDef Can_Init(void);
HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length);
void Can_GetDiagnostics(CanDiagnostics *diagnostics);

/* Bounded RX processing and periodic telemetry, with no UART in this task. */
void Can_ReceiveTask(void *argument);




#endif /* USER_DRV_CAN_H */
