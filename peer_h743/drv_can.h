#ifndef USER_DRV_CAN_H
#define USER_DRV_CAN_H

#include "main.h"
#include "FreeRTOS.h"
#include "can_protocol.h"

typedef struct
{
    CanMeasurements measurements;
    CanMotorStatus status;
    uint32_t ageMs;
    uint32_t rxDropped;
    uint32_t txFailed;
    uint8_t available;
    uint8_t online;
} CanRemoteSnapshot;

/* Prepare filters and queues before starting the scheduler. */
void FDCAN_Init(void);
HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length);
void Can_CommunicationTask(void *argument);
void Can_ReportTask(void *argument);
void Can_GetRemoteSnapshot(CanRemoteSnapshot *snapshot);

#endif /* USER_DRV_CAN_H */
