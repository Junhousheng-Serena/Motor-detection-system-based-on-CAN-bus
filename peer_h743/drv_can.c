#include "drv_can.h"
#include "fdcan.h"
#include "queue.h"
#include "task.h"
#include "drv_print.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CAN_RX_QUEUE_LENGTH 16U
#define CAN_TASK_PERIOD_MS 10U
#define CAN_PAIR_TIMEOUT_MS 500U

typedef struct
{
    TickType_t receivedAt;
    uint16_t id;
    uint8_t data[8];
} CanRxFrame;

static QueueHandle_t receiveQueue;
static StaticQueue_t receiveQueueControl;
static uint8_t receiveQueueStorage[CAN_RX_QUEUE_LENGTH * sizeof(CanRxFrame)];
static volatile uint32_t rxDropped;
static volatile uint32_t txFailed;
static CanRemoteSnapshot latest;
static TickType_t lastSample;
static CanMeasurements pendingMeasurements;
static CanMotorStatus pendingStatus;
static TickType_t measurementsTick, statusTick;
static uint8_t haveMeasurements, haveStatus;

void FDCAN_Init(void)
{
    receiveQueue = xQueueCreateStatic(CAN_RX_QUEUE_LENGTH, sizeof(CanRxFrame),
                                     receiveQueueStorage, &receiveQueueControl);
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_ID_MOTOR_MEASUREMENTS;
    filter.FilterID2 = CAN_ID_MOTOR_STATUS;
    if (receiveQueue == NULL ||
        HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t payload[8] = {0};
    if (id > 0x7FFU || length > 8U || (length != 0U && data == NULL)) return HAL_ERROR;
    /* This HAL reads full words, including the last word of a short payload. */
    if (length != 0U) memcpy(payload, data, length);
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = length; /* This project's H7 HAL uses unshifted DLC 0..8. */
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    HAL_StatusTypeDef result = HAL_BUSY;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0U)
    {
        result = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, payload);
    }
    __set_PRIMASK(irqState);
    return result;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t flags)
{
    if (handle != &hfdcan1 || (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
    BaseType_t taskWoken = pdFALSE;
    uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0);
    while (pending-- != 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t payload[64]; /* HAL copies by DLC; validate after the bounded copy. */
        if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, payload) != HAL_OK)
        {
            rxDropped++;
            break;
        }
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8)
            continue;
        if (header.Identifier != CAN_ID_MOTOR_MEASUREMENTS &&
            header.Identifier != CAN_ID_MOTOR_STATUS) continue;
        CanRxFrame frame = {.id = (uint16_t)header.Identifier};
        frame.receivedAt = xTaskGetTickCountFromISR();
        memcpy(frame.data, payload, 8);
        if (receiveQueue == NULL ||
            xQueueSendFromISR(receiveQueue, &frame, &taskWoken) != pdPASS) rxDropped++;
    }
    portYIELD_FROM_ISR(taskWoken);
}

static void Can_ProcessFrame(const CanRxFrame *frame, TickType_t now)
{
    if (frame->id == CAN_ID_MOTOR_MEASUREMENTS)
    {
        CanMeasurements decoded;
        if (!CanProtocol_DecodeMeasurements(frame->data, &decoded)) return;
        pendingMeasurements = decoded;
        measurementsTick = frame->receivedAt;
        haveMeasurements = 1U;
    }
    else if (frame->id == CAN_ID_MOTOR_STATUS)
    {
        CanMotorStatus decoded;
        if (!CanProtocol_DecodeStatus(frame->data, &decoded)) return;
        pendingStatus = decoded;
        statusTick = frame->receivedAt;
        haveStatus = 1U;
    }
    if (haveMeasurements && haveStatus &&
        pendingMeasurements.sequence == pendingStatus.sequence &&
        (TickType_t)(now - measurementsTick) < pdMS_TO_TICKS(CAN_PAIR_TIMEOUT_MS) &&
        (TickType_t)(now - statusTick) < pdMS_TO_TICKS(CAN_PAIR_TIMEOUT_MS))
    {
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        latest.measurements = pendingMeasurements;
        latest.status = pendingStatus;
        latest.available = 1U;
        /* Age uses the older half, not the later time at which a queue drains. */
        lastSample = (TickType_t)(now - measurementsTick) >
                     (TickType_t)(now - statusTick) ? measurementsTick : statusTick;
        __set_PRIMASK(irqState);
        haveMeasurements = haveStatus = 0U;
    }
}

void Can_GetRemoteSnapshot(CanRemoteSnapshot *snapshot)
{
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    *snapshot = latest;
    TickType_t age = xTaskGetTickCount() - lastSample;
    snapshot->online = latest.available && age < pdMS_TO_TICKS(CAN_PEER_TIMEOUT_MS);
    snapshot->ageMs = latest.available ? (uint32_t)age * portTICK_PERIOD_MS : UINT32_MAX;
    snapshot->rxDropped = rxDropped;
    snapshot->txFailed = txFailed;
    __set_PRIMASK(irqState);
}

void Can_CommunicationTask(void *argument)
{
    (void)argument;
    if (receiveQueue == NULL || HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        txFailed++;
        vTaskDelete(NULL);
        return;
    }
    uint8_t sequence = 0U;
    TickType_t previousHeartbeat = xTaskGetTickCount() - pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS);
    for (;;)
    {
        CanRxFrame frame;
        for (unsigned i = 0; i < CAN_RX_QUEUE_LENGTH; i++)
        {
            if (xQueueReceive(receiveQueue, &frame, 0) != pdPASS) break;
            Can_ProcessFrame(&frame, xTaskGetTickCount());
        }
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - previousHeartbeat) >= pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS))
        {
            previousHeartbeat = now;
            uint8_t payload[8];
            CanProtocol_EncodeHeartbeat(sequence++, now / pdMS_TO_TICKS(1000), payload);
            if (Can_Send(CAN_ID_HOST_HEARTBEAT, payload, 8) != HAL_OK) txFailed++;
        }
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_PERIOD_MS));
    }
}

static void Can_FormatCurrent(char text[16], uint16_t value)
{
    if (value == CAN_VALUE_INVALID) snprintf(text, 16, "INVALID");
    else snprintf(text, 16, "%u.%u", (unsigned)(value / 10U), (unsigned)(value % 10U));
}

void Can_ReportTask(void *argument)
{
    (void)argument;
    static const char *states[] = {
        "WAIT_SENSORS", "READY", "OVERHEAT_LATCHED", "SENSOR_FAULT_LATCHED",
        "OVERCURRENT_LATCHED"
    };
    for (;;)
    {
        CanRemoteSnapshot sample;
        Can_GetRemoteSnapshot(&sample);
        if (!sample.available)
        {
            my_printf("F103: WAIT_DATA, rx_drop=%lu, tx_fail=%lu\r\n",
                      (unsigned long)sample.rxDropped, (unsigned long)sample.txFailed);
        }
        else
        {
            char temperature[16], current[16], peak[16];
            int32_t raw = sample.measurements.temperatureDeciC;
            uint32_t magnitude = (uint32_t)(raw < 0 ? -raw : raw);
            if (raw == CAN_TEMPERATURE_INVALID) snprintf(temperature, 16, "INVALID");
            else snprintf(temperature, 16, "%s%lu.%lu", raw < 0 ? "-" : "",
                          (unsigned long)(magnitude / 10), (unsigned long)(magnitude % 10));
            Can_FormatCurrent(current, sample.measurements.currentDeciMa);
            Can_FormatCurrent(peak, sample.measurements.peakDeciMa);
            my_printf("F103: %s, seq=%u, temp=%s C, current=%s mA, peak=%s mA, "
                      "motor=%s, duty=%u%%, age=%lu ms\r\n",
                      sample.online ? "ONLINE" : "OFFLINE",
                      (unsigned)sample.measurements.sequence, temperature, current, peak,
                      states[sample.measurements.protection], (unsigned)sample.status.appliedDuty,
                      (unsigned long)sample.ageMs);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
