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

_Static_assert(CAN_DEFAULT_ENABLE <= 1U && CAN_DEFAULT_DUTY_PERCENT <= 100U &&
               CAN_DEFAULT_CURRENT_LIMIT_MA > 0U &&
               CAN_DEFAULT_CURRENT_LIMIT_MA <= CAN_CURRENT_LIMIT_MAX_MA,
               "Invalid default motor command");

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
static TickType_t lastSample, lastHeartbeat;
static CanMotorCommand desired = {.enable = CAN_DEFAULT_ENABLE,
    .dutyPercent = CAN_DEFAULT_DUTY_PERCENT, .currentLimitMa = CAN_DEFAULT_CURRENT_LIMIT_MA};

HAL_StatusTypeDef Can_SetMotorCommand(uint8_t enable, uint8_t dutyPercent, uint16_t currentLimitMa)
{
    CanMotorCommand command = {.enable = enable, .dutyPercent = dutyPercent,
                               .currentLimitMa = currentLimitMa};
    if (!CanProtocol_CommandValid(&command)) return HAL_ERROR;
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    desired = command;
    __set_PRIMASK(irqState);
    return HAL_OK;
}

void Can_GetMotorCommand(CanMotorCommand *command)
{
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    *command = desired;
    __set_PRIMASK(irqState);
}

void FDCAN_Init(void)
{
    receiveQueue = xQueueCreateStatic(CAN_RX_QUEUE_LENGTH, sizeof(CanRxFrame),
                                     receiveQueueStorage, &receiveQueueControl);
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_ID_MOTOR_TELEMETRY;
    filter.FilterID2 = CAN_ID_MOTOR_FAULT; /* Exact IDs are checked in the ISR. */
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
        if (header.Identifier != CAN_ID_MOTOR_TELEMETRY &&
            header.Identifier != CAN_ID_MOTOR_HEARTBEAT &&
            header.Identifier != CAN_ID_MOTOR_FAULT) continue;
        CanRxFrame frame = {.id = (uint16_t)header.Identifier};
        frame.receivedAt = xTaskGetTickCountFromISR();
        memcpy(frame.data, payload, 8);
        if (receiveQueue == NULL ||
            xQueueSendFromISR(receiveQueue, &frame, &taskWoken) != pdPASS) rxDropped++;
    }
    portYIELD_FROM_ISR(taskWoken);
}

static void Can_ProcessFrame(const CanRxFrame *frame)
{
    /* This task is the only writer. Mask interrupts while publishing to readers. */
    CanTelemetry telemetry;
    CanHeartbeat heartbeat;
    CanFaultEvent fault;
    if (frame->id == CAN_ID_MOTOR_TELEMETRY &&
        CanProtocol_DecodeTelemetry(frame->data, &telemetry))
    {
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        latest.telemetry = telemetry;
        latest.available = 1U;
        lastSample = frame->receivedAt;
        __set_PRIMASK(irqState);
    }
    else if (frame->id == CAN_ID_MOTOR_HEARTBEAT &&
             CanProtocol_DecodeHeartbeat(frame->data, &heartbeat))
    {
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        latest.heartbeat = heartbeat;
        latest.heartbeatAvailable = 1U;
        lastHeartbeat = frame->receivedAt;
        __set_PRIMASK(irqState);
    }
    else if (frame->id == CAN_ID_MOTOR_FAULT &&
             CanProtocol_DecodeFault(frame->data, &fault))
    {
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        latest.fault = fault; /* Retries overwrite the same event, not a new fault. */
        latest.faultAvailable = 1U;
        __set_PRIMASK(irqState);
    }
}

void Can_GetRemoteSnapshot(CanRemoteSnapshot *snapshot)
{
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    *snapshot = latest;
    TickType_t now = xTaskGetTickCount();
    TickType_t age = now - lastSample, heartbeatAge = now - lastHeartbeat;
    snapshot->telemetryFresh = latest.available && age < pdMS_TO_TICKS(CAN_TELEMETRY_TIMEOUT_MS);
    snapshot->heartbeatOnline = latest.heartbeatAvailable &&
        heartbeatAge < pdMS_TO_TICKS(CAN_PEER_TIMEOUT_MS);
    snapshot->online = snapshot->telemetryFresh && snapshot->heartbeatOnline;
    snapshot->ageMs = latest.available ? (uint32_t)age * portTICK_PERIOD_MS : UINT32_MAX;
    snapshot->heartbeatAgeMs = latest.heartbeatAvailable ?
        (uint32_t)heartbeatAge * portTICK_PERIOD_MS : UINT32_MAX;
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
    uint8_t heartbeatSequence = 0U, commandSequence = 0U;
    TickType_t previousHeartbeat = xTaskGetTickCount() - pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS);
    TickType_t previousCommand = xTaskGetTickCount() - pdMS_TO_TICKS(CAN_COMMAND_PERIOD_MS);
    for (;;)
    {
        CanRxFrame frame;
        for (unsigned i = 0; i < CAN_RX_QUEUE_LENGTH; i++)
        {
            if (xQueueReceive(receiveQueue, &frame, 0) != pdPASS) break;
            Can_ProcessFrame(&frame);
        }
        TickType_t now = xTaskGetTickCount();
        /* Send heartbeat first so an arriving startup command can be accepted. */
        if ((TickType_t)(now - previousHeartbeat) >= pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS))
        {
            previousHeartbeat = now;
            uint8_t payload[8];
            CanHeartbeat heartbeat = {.sequence = heartbeatSequence++,
                .uptimeSeconds = now / pdMS_TO_TICKS(1000)};
            CanProtocol_EncodeHeartbeat(&heartbeat, payload);
            if (Can_Send(CAN_ID_HOST_HEARTBEAT, payload, 8) != HAL_OK) txFailed++;
        }
        if ((TickType_t)(now - previousCommand) >= pdMS_TO_TICKS(CAN_COMMAND_PERIOD_MS))
        {
            previousCommand = now;
            CanMotorCommand command;
            Can_GetMotorCommand(&command);
            command.sequence = commandSequence;
            uint8_t payload[8];
            CanProtocol_EncodeCommand(&command, payload);
            if (Can_Send(CAN_ID_MOTOR_COMMAND, payload, 8) == HAL_OK) commandSequence++;
            else txFailed++;
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
        "OVERCURRENT_LATCHED", "COMM_FAULT_LATCHED"
    };
    for (;;)
    {
        CanRemoteSnapshot sample;
        Can_GetRemoteSnapshot(&sample);
        if (!sample.available)
        {
            my_printf("F103: WAIT_DATA, hb=%u, rx_drop=%lu, tx_fail=%lu\r\n",
                      (unsigned)sample.heartbeatOnline,
                      (unsigned long)sample.rxDropped, (unsigned long)sample.txFailed);
        }
        else
        {
            char temperature[16], current[16];
            int32_t raw = sample.telemetry.temperatureDeciC;
            uint32_t magnitude = (uint32_t)(raw < 0 ? -raw : raw);
            if (raw == CAN_TEMPERATURE_INVALID) snprintf(temperature, 16, "INVALID");
            else snprintf(temperature, 16, "%s%lu.%lu", raw < 0 ? "-" : "",
                          (unsigned long)(magnitude / 10), (unsigned long)(magnitude % 10));
            Can_FormatCurrent(current, sample.telemetry.currentDeciMa);
            my_printf("F103: %s, ack=%u, temp=%s C, current=%s mA, "
                      "motor=%s, duty=%u%%, flags=0x%02X, hb=%u, age=%lu ms, last_fault=%u\r\n",
                      sample.online ? "ONLINE" : "OFFLINE",
                      (unsigned)sample.telemetry.commandSequence, temperature, current,
                      states[sample.telemetry.status & CAN_STATUS_STATE_MASK],
                      (unsigned)sample.telemetry.appliedDuty, (unsigned)sample.telemetry.status,
                      (unsigned)sample.heartbeatOnline, (unsigned long)sample.ageMs,
                      sample.faultAvailable ? (unsigned)sample.fault.code : 0U);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
