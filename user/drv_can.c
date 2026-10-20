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
static uint8_t reportSequence;

_Static_assert(MOTOR_WAITING_SENSORS == 0 && MOTOR_READY == 1 &&
               MOTOR_OVERHEAT_LATCHED == 2 && MOTOR_SENSOR_FAULT_LATCHED == 3 &&
               MOTOR_OVERCURRENT_LATCHED == 4, "Update the CAN wire state mapping");

HAL_StatusTypeDef Can_Init(void)
{
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
        if (frame.id != CAN_ID_HOST_HEARTBEAT) continue;
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
    CanMeasurements sample = {0};
    CanMotorStatus status = {0};
    CanDiagnostics diagnostics;
    /* Capture temperature, current and PWM state from one complete batch. */
    uint32_t irqState = __get_PRIMASK();
    __disable_irq();
    float temperature = ntc_res;
    float current = current_res_ma;
    float peak = current_peak_ma;
    float voltage = voltage_res;
    sample.protection = (uint8_t)Motor_GetProtectionState();
    status.appliedDuty = Motor_GetAppliedDuty();
    status.requestedDuty = Motor_GetRequestedDuty();
    Can_GetDiagnostics(&diagnostics);
    __set_PRIMASK(irqState);

    sample.sequence = status.sequence = reportSequence++;
    sample.temperatureDeciC = Can_TemperatureValue(temperature);
    sample.currentDeciMa = Can_UnsignedValue(current, 10.0f);
    sample.peakDeciMa = Can_UnsignedValue(peak, 10.0f);
    status.voltageMv = Can_UnsignedValue(voltage, 1000.0f);
    status.linkFlags = (diagnostics.peerOnline ? CAN_LINK_HOST_ONLINE : 0U) |
                      (diagnostics.rxDropped ? CAN_LINK_RX_DROPPED : 0U) |
                      (diagnostics.txFailed ? CAN_LINK_TX_FAILED : 0U);

    uint8_t measurements[8], motorStatus[8];
    CanProtocol_EncodeMeasurements(&sample, measurements);
    CanProtocol_EncodeStatus(&status, motorStatus);
    /* Do not spin if the bus is unavailable; the next period sends fresh data. */
    if (Can_Send(CAN_ID_MOTOR_MEASUREMENTS, measurements, 8U) != HAL_OK) txFailed++;
    if (Can_Send(CAN_ID_MOTOR_STATUS, motorStatus, 8U) != HAL_OK) txFailed++;
}

void Can_ReceiveTask(void *argument)
{
    (void)argument;
    /* The queue and scheduler are ready before an ISR can call FreeRTOS. */
    if (receiveQueue == NULL || HAL_CAN_Start(&hcan) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Motor_Stop();
        vTaskDelete(NULL);
        return;
    }

    TickType_t previousReport = xTaskGetTickCount() - pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS);
    for (;;)
    {
        CanRxFrame frame;
        for (unsigned i = 0; i < CAN_RX_QUEUE_LENGTH; i++)
        {
            if (xQueueReceive(receiveQueue, &frame, 0) != pdPASS) break;
            if (frame.id == CAN_ID_HOST_HEARTBEAT && CanProtocol_IsHeartbeat(frame.data))
            {
                lastHeartbeat = frame.receivedAt;
                heartbeatSeen = 1U;
            }
        }
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - previousReport) >= pdMS_TO_TICKS(CAN_TELEMETRY_PERIOD_MS))
        {
            previousReport = now;
            Can_Report();
        }
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_PERIOD_MS));
    }
}
