#include "drv_can.h"
#include "can.h"
#include "queue.h"
#include "task.h"
#include "drv_motor.h"
#include "drv_ntc.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CAN_RX_QUEUE_LENGTH 16U
#define CAN_TASK_PERIOD_MS  10U

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
static TickType_t lastHeartbeat;
static uint8_t heartbeatSeen;
static TickType_t lastCommand;
static uint8_t commandSeen, commandSequence, heartbeatSequence;
static uint8_t faultCaptured, faultCopies;
static CanFaultEvent pendingFault;
static TickType_t previousFault;

_Static_assert(MOTOR_WAITING_SENSORS == 0 && MOTOR_READY == 1 &&
               MOTOR_OVERHEAT_LATCHED == 2 && MOTOR_SENSOR_FAULT_LATCHED == 3 &&
               MOTOR_OVERCURRENT_LATCHED == 4 && MOTOR_COMM_FAULT_LATCHED == 5, "Update the CAN wire state mapping");
_Static_assert(CAN_CURRENT_LIMIT_MAX_MA == MOTOR_OVERCURRENT_STOP_MA,
               "Protocol and motor current limits must agree");

HAL_StatusTypeDef Can_Init(void)
{
    Motor_RequireRemoteControl();
    if (receiveQueue == NULL)
    {
        receiveQueue = xQueueCreateStatic(CAN_RX_QUEUE_LENGTH, sizeof(CanRxFrame),
                                         receiveQueueStorage, &receiveQueueControl);
    }
    return receiveQueue != NULL ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Can_Send(uint16_t id, const uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint8_t payload[8] = {0};
    if (id > 0x7FFU || length > 8U || (length != 0U && data == NULL))
    {
        return HAL_ERROR;
    }
    if (length != 0U) memcpy(payload, data, length);
    header.StdId = id;
    header.RTR = CAN_RTR_DATA;
    header.IDE = CAN_ID_STD;
    header.DLC = length;
    header.TransmitGlobalTime = DISABLE;

    /* Serialize mailbox selection without waiting for bus transmission. */
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    HAL_StatusTypeDef result = HAL_BUSY;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0U)
    {
        result = HAL_CAN_AddTxMessage(&hcan, &header, payload, &mailbox);
    }
    __set_PRIMASK(irqState);
    return result;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    if (handle != &hcan) return;
    BaseType_t taskWoken = pdFALSE;
    /* Drain only the FIFO contents present on entry, bounding ISR work. */
    uint32_t pending = HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0);
    while (pending-- != 0U)
    {
        CAN_RxHeaderTypeDef header;
        CanRxFrame frame;
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, frame.data) != HAL_OK)
        {
            rxDropped++;
            break;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
            header.DLC != CAN_PROTOCOL_LENGTH) continue;
        frame.id = (uint16_t)header.StdId;
        if (frame.id != CAN_ID_HOST_HEARTBEAT && frame.id != CAN_ID_MOTOR_COMMAND) continue;
        frame.receivedAt = xTaskGetTickCountFromISR();
        if (receiveQueue == NULL ||
            xQueueSendFromISR(receiveQueue, &frame, &taskWoken) != pdPASS)
        {
            rxDropped++;
        }
    }
    portYIELD_FROM_ISR(taskWoken);
}

void Can_GetDiagnostics(CanDiagnostics *diagnostics)
{
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    diagnostics->rxDropped = rxDropped;
    diagnostics->txFailed = txFailed;
    diagnostics->peerOnline = heartbeatSeen &&
        (TickType_t)(xTaskGetTickCount() - lastHeartbeat) < pdMS_TO_TICKS(CAN_PEER_TIMEOUT_MS);
    diagnostics->commandSeen = commandSeen;
    diagnostics->commandFresh = commandSeen &&
        (TickType_t)(xTaskGetTickCount() - lastCommand) < pdMS_TO_TICKS(CAN_COMMAND_TIMEOUT_MS);
    diagnostics->commandSequence = commandSequence;
    __set_PRIMASK(irqState);
}

static uint16_t Can_UnsignedValue(float value, float scale)
{
    if (!isfinite(value) || value < 0.0f) return CAN_VALUE_INVALID;
    float scaled = value * scale;
    if (scaled >= 65534.0f) return 65534U;
    return (uint16_t)(scaled + 0.5f);
}

static int16_t Can_TemperatureValue(float value)
{
    if (!isfinite(value)) return CAN_TEMPERATURE_INVALID;
    float scaled = value * 10.0f;
    if (scaled >= 32767.0f) return 32767;
    if (scaled <= -32767.0f) return -32767;
    return (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

static void Can_Report(void)
{
    CanTelemetry sample = {0};
    CanDiagnostics diagnostics;
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    float temperature = ntc_res;
    float current = current_res_ma;
    sample.status = (uint8_t)Motor_GetProtectionState();
    sample.appliedDuty = Motor_GetAppliedDuty();
    if (Motor_GetRemoteEnabled()) sample.status |= CAN_STATUS_ENABLED;
    Can_GetDiagnostics(&diagnostics);
    sample.commandSequence = commandSequence;
    __set_PRIMASK(irqState);
    if (diagnostics.commandSeen) sample.status |= CAN_STATUS_COMMAND_SEEN;
    if (diagnostics.commandFresh) sample.status |= CAN_STATUS_COMMAND_FRESH;
    if (diagnostics.peerOnline) sample.status |= CAN_STATUS_HOST_ONLINE;
    sample.temperatureDeciC = Can_TemperatureValue(temperature);
    sample.currentDeciMa = Can_UnsignedValue(current, 10.0f);
    uint8_t payload[8];
    CanProtocol_EncodeTelemetry(&sample, payload);
    if (Can_Send(CAN_ID_MOTOR_TELEMETRY, payload, 8U) != HAL_OK) txFailed++;
}

static void Can_SendHeartbeat(TickType_t now)
{
    CanHeartbeat heartbeat = {.sequence = heartbeatSequence++,
        .state = (uint8_t)Motor_GetProtectionState(),
        .uptimeSeconds = now / pdMS_TO_TICKS(1000)};
    heartbeat.flags = (rxDropped ? CAN_HEARTBEAT_RX_DROPPED : 0U) |
                      (txFailed ? CAN_HEARTBEAT_TX_FAILED : 0U) |
                      (commandSeen ? CAN_HEARTBEAT_COMMAND_SEEN : 0U) |
                      (Motor_GetRemoteEnabled() ? CAN_HEARTBEAT_ENABLED : 0U);
    uint8_t payload[8];
    CanProtocol_EncodeHeartbeat(&heartbeat, payload);
    if (Can_Send(CAN_ID_MOTOR_HEARTBEAT, payload, 8U) != HAL_OK) txFailed++;
}

static void Can_ReportFault(TickType_t now)
{
    MotorFaultSnapshot fault;
    Motor_GetFaultSnapshot(&fault);
    if (!faultCaptured && fault.state >= MOTOR_OVERHEAT_LATCHED)
    {
        static const uint8_t codes[] = {0, 0, CAN_FAULT_OVERHEAT, CAN_FAULT_SENSOR,
            CAN_FAULT_OVERCURRENT, CAN_FAULT_COMMUNICATION};
        pendingFault = (CanFaultEvent){.sequence = 0, .code = codes[fault.state],
            .commandSequence = commandSequence,
            .peakDeciMa = Can_UnsignedValue(fault.currentPeakMa, 10.0f),
            .temperatureDeciC = Can_TemperatureValue(fault.temperatureC)};
        faultCaptured = 1U;
        faultCopies = 3U;
        previousFault = now - pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS);
    }
    if (faultCopies && (TickType_t)(now - previousFault) >= pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS))
    {
        uint8_t payload[8];
        CanProtocol_EncodeFault(&pendingFault, payload);
        previousFault = now;
        if (Can_Send(CAN_ID_MOTOR_FAULT, payload, 8U) == HAL_OK) faultCopies--;
        else txFailed++; /* Keep pending if no mailbox was available. */
    }
}

static void Can_ProcessFrame(const CanRxFrame *frame, TickType_t now)
{
    if (frame->id == CAN_ID_HOST_HEARTBEAT)
    {
        CanHeartbeat heartbeat;
        if (CanProtocol_DecodeHeartbeat(frame->data, &heartbeat) &&
            heartbeat.state == 0U && heartbeat.flags == 0U &&
            (TickType_t)(now - frame->receivedAt) < pdMS_TO_TICKS(CAN_PEER_TIMEOUT_MS))
        {
            lastHeartbeat = frame->receivedAt;
            heartbeatSeen = 1U;
        }
        return;
    }
    CanMotorCommand command;
    if (!CanProtocol_DecodeCommand(frame->data, &command) ||
        (TickType_t)(now - frame->receivedAt) >= pdMS_TO_TICKS(CAN_COMMAND_TIMEOUT_MS)) return;
    /* A well-formed disable always stops, even if its sequence is duplicated.
     * It does not renew the watchdog unless it is a new accepted command. */
    if (!command.enable) Motor_Stop();
    if (!heartbeatSeen ||
        (TickType_t)(now - lastHeartbeat) >= pdMS_TO_TICKS(CAN_PEER_TIMEOUT_MS) ||
        (commandSeen && !CanProtocol_SequenceNewer(command.sequence, commandSequence))) return;
    commandSequence = command.sequence;
    commandSeen = 1U;
    lastCommand = frame->receivedAt;
    (void)Motor_ApplyRemoteCommand(command.enable, command.dutyPercent, command.currentLimitMa);
}

void Can_ReceiveTask(void *argument)
{
    (void)argument;
    if (receiveQueue == NULL || HAL_CAN_Start(&hcan) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Motor_LatchCommunicationFault();
        vTaskDelete(NULL);
        return;
    }
    TickType_t startedAt = xTaskGetTickCount();
    TickType_t previousReport = startedAt - pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS);
    TickType_t previousHeartbeat = startedAt - pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS);
    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        /* Check before consuming queued frames: a real gap must stay latched. */
        if ((commandSeen && (TickType_t)(now - lastCommand) >= pdMS_TO_TICKS(CAN_COMMAND_TIMEOUT_MS)) ||
            (heartbeatSeen && (TickType_t)(now - lastHeartbeat) >= pdMS_TO_TICKS(CAN_PEER_TIMEOUT_MS)) ||
            (!commandSeen && (TickType_t)(now - startedAt) >= pdMS_TO_TICKS(CAN_STARTUP_TIMEOUT_MS)))
            Motor_LatchCommunicationFault();
        CanRxFrame frame;
        for (unsigned i = 0; i < CAN_RX_QUEUE_LENGTH; i++)
        {
            if (xQueueReceive(receiveQueue, &frame, 0) != pdPASS) break;
            Can_ProcessFrame(&frame, xTaskGetTickCount());
        }
        now = xTaskGetTickCount();
        Can_ReportFault(now);
        if ((TickType_t)(now - previousReport) >= pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS))
        {
            previousReport = now;
            Can_Report();
        }
        if ((TickType_t)(now - previousHeartbeat) >= pdMS_TO_TICKS(CAN_HEARTBEAT_PERIOD_MS))
        {
            previousHeartbeat = now;
            Can_SendHeartbeat(now);
        }
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_PERIOD_MS));
    }
}
