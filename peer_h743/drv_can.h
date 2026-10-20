#ifndef USER_DRV_CAN_H
#define USER_DRV_CAN_H

#include "main.h"
#include "FreeRTOS.h"
#include "can_protocol.h"

#define CAN_DEFAULT_ENABLE 1U
#define CAN_DEFAULT_DUTY_PERCENT 50U
#define CAN_DEFAULT_CURRENT_LIMIT_MA 160U

typedef struct
{
    CanTelemetry telemetry;
    CanHeartbeat heartbeat;
    CanFaultEvent fault;
    uint32_t ageMs, heartbeatAgeMs;
    uint32_t rxDropped, txFailed;
    uint8_t available, heartbeatAvailable, faultAvailable;
    uint8_t telemetryFresh, heartbeatOnline, online;
} CanRemoteSnapshot;

/* Prepare filters and queues before starting the scheduler. */
void FDCAN_Init(void);
HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length);
/* Thread-safe desired command. Periodically sent even when values do not change.
 * Current limit means trip-and-latch, not current regulation. No remote reset. */
HAL_StatusTypeDef Can_SetMotorCommand(uint8_t enable, uint8_t dutyPercent, uint16_t currentLimitMa);
void Can_GetMotorCommand(CanMotorCommand *command);
void Can_CommunicationTask(void *argument);
void Can_ReportTask(void *argument);
void Can_GetRemoteSnapshot(CanRemoteSnapshot *snapshot);

#endif /* USER_DRV_CAN_H */
