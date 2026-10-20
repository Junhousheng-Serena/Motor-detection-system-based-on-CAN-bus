#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "drv_can.h"
#include "queue.h"
#include "task.h"
#ifdef TEST_H743
#include "fdcan.h"
FDCAN_HandleTypeDef hfdcan1;
#else
#include "can.h"
#include "tim.h"
#include "drv_motor.h"
#include "drv_ntc.h"
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim3 = {3599, {0, 0}};
volatile float ntc_res = 25.0f, voltage_res = 0.6f;
volatile float current_res_ma = 30.0f, current_peak_ma = 40.0f;
#endif

uint32_t testIrqMask;
static TickType_t now;
static QueueHandle_t queue;
static jmp_buf done;
static unsigned iterations, maxIterations = 1, started, prints;
static unsigned txCount, txFree = 3;
static uint16_t txIds[8];
static uint8_t txData[8][8];
static uint16_t rxId;
static uint8_t rxData[64];
static unsigned rxPending, rxLength = 8, rxExtended, rxRemote, rxFd;
static uint8_t values[8] = {0x11, 7, 0xFA, 0, 0x2C, 1, 0x90, 1};
static uint8_t statusData[8] = {1, 7, 0x58, 2, 50, 50, 1, 0};

TickType_t xTaskGetTickCount(void) { return now; }
TickType_t xTaskGetTickCountFromISR(void) { return now; }
void vTaskDelete(void *task) { (void)task; assert(!"unexpected task deletion"); }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks != 0);
    if (++iterations >= maxIterations) longjmp(done, 1);
    now += ticks;
}
QueueHandle_t xQueueCreateStatic(unsigned length, unsigned itemSize,
                                 uint8_t *storage, StaticQueue_t *control)
{
    *control = (StaticQueue_t){.storage = storage, .length = length, .itemSize = itemSize};
    queue = control;
    return queue;
}
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken)
{
    (void)woken;
    if (q->count == q->length) return pdFALSE;
    memcpy(q->storage + q->tail * q->itemSize, item, q->itemSize);
    q->tail = (q->tail + 1) % q->length;
    q->count++;
    return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t timeout)
{
    assert(timeout == 0);
    if (!q->count) return pdFALSE;
    memcpy(item, q->storage + q->head * q->itemSize, q->itemSize);
    q->head = (q->head + 1) % q->length;
    q->count--;
    return pdPASS;
}
void my_printf(const char *format, ...)
{
    (void)format;
    prints++;
}
static HAL_StatusTypeDef recordTx(uint16_t id, const uint8_t *data)
{
    assert(txCount < 8);
    txIds[txCount] = id;
    memcpy(txData[txCount++], data, 8);
    return HAL_OK;
}

#ifdef TEST_H743
void Error_Handler(void) { assert(!"unexpected error handler"); }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, const FDCAN_FilterTypeDef *f)
{
    assert(h == &hfdcan1 && f->FilterType == FDCAN_FILTER_DUAL);
    assert(f->FilterID1 == 0x201 && f->FilterID2 == 0x202);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a,
                                              uint32_t b, uint32_t c, uint32_t d)
{
    (void)h;
    assert(a == FDCAN_REJECT && b == FDCAN_REJECT);
    assert(c == FDCAN_REJECT_REMOTE && d == FDCAN_REJECT_REMOTE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{
    (void)h;
    assert(queue != NULL);
    started++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t flags,
                                                uint32_t index)
{
    (void)h; (void)index;
    assert(started && flags == FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return rxPending; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h)
{ (void)h; return txFree; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    *header = (FDCAN_RxHeaderTypeDef){.Identifier = rxId, .DataLength = rxLength,
        .IdType = rxExtended, .RxFrameType = rxRemote, .FDFormat = rxFd};
    memcpy(data, rxData, 8);
    assert(rxPending-- > 0);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
                                              const FDCAN_TxHeaderTypeDef *header,
                                              const uint8_t *data)
{
    (void)h;
    assert(header->FDFormat == FDCAN_CLASSIC_CAN && header->BitRateSwitch == FDCAN_BRS_OFF);
    assert(header->DataLength <= 8 && header->IdType == FDCAN_STANDARD_ID);
    return recordTx((uint16_t)header->Identifier, data);
}
#else
void testSetCompare(TIM_HandleTypeDef *h, uint32_t channel, uint32_t value)
{ h->ccr[channel] = value; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel)
{ assert(h->ccr[channel] == 0); return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{ (void)h; assert(queue != NULL); started++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t flags)
{ (void)h; assert(started && flags == CAN_IT_RX_FIFO0_MSG_PENDING); return HAL_OK; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h)
{ (void)h; return txFree; }
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return rxPending; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *header,
                                      const uint8_t *data, uint32_t *mailbox)
{
    (void)h; (void)mailbox;
    assert(header->IDE == CAN_ID_STD && header->RTR == CAN_RTR_DATA && header->DLC <= 8);
    return recordTx((uint16_t)header->StdId, data);
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                      CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo;
    *header = (CAN_RxHeaderTypeDef){.StdId = rxId, .DLC = rxLength,
                                   .IDE = rxExtended, .RTR = rxRemote};
    memcpy(data, rxData, 8);
    assert(rxPending-- > 0);
    return HAL_OK;
}
#endif

static void inject(uint16_t id, const uint8_t data[8])
{
    rxId = id;
    memcpy(rxData, data, 8);
    rxPending = 1;
#ifdef TEST_H743
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#else
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan);
#endif
    assert(rxPending == 0 && prints == 0); /* No UART inside RX ISR. */
}
static void runTask(void)
{
    if (setjmp(done) == 0)
    {
#ifdef TEST_H743
        Can_CommunicationTask(NULL);
#else
        Can_ReceiveTask(NULL);
#endif
    }
    assert(prints == 0 && testIrqMask == 0);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *name = argv[1];
    /* Verify known wire bytes and signed decoding, not just a round trip. */
    CanMeasurements sample;
    CanMotorStatus status;
    assert(CanProtocol_DecodeMeasurements(values, &sample));
    assert(sample.temperatureDeciC == 250 && sample.currentDeciMa == 300 &&
           sample.peakDeciMa == 400 && sample.protection == 1 && sample.sequence == 7);
    assert(CanProtocol_DecodeStatus(statusData, &status) && status.voltageMv == 600);
    uint8_t encoded[8];
    CanProtocol_EncodeMeasurements(&sample, encoded);
    assert(memcmp(encoded, values, 8) == 0);
    CanProtocol_EncodeStatus(&status, encoded);
    assert(memcmp(encoded, statusData, 8) == 0);
    encoded[0] = 0x11;
    encoded[2] = 0x9B; encoded[3] = 0xFF; /* -10.1 C */
    assert(CanProtocol_DecodeMeasurements(encoded, &sample) && sample.temperatureDeciC == -101);
    encoded[0] = 0x21;
    assert(!CanProtocol_DecodeMeasurements(encoded, &sample));
    encoded[0] = 0x15;
    assert(!CanProtocol_DecodeMeasurements(encoded, &sample));

#ifdef TEST_H743
    FDCAN_Init();
    assert(started == 0);
    if (strcmp(name, "pair") == 0 || strcmp(name, "timeout_wrap") == 0)
    {
        now = UINT32_MAX - 20U;
        inject(0x202, statusData); /* Frame arrival order must not matter. */
        inject(0x201, values);
        runTask();
        CanRemoteSnapshot result;
        Can_GetRemoteSnapshot(&result);
        assert(result.available && result.online && result.ageMs == 0);
        assert(result.measurements.temperatureDeciC == 250 && result.status.appliedDuty == 50);
        now += 150U;
        Can_GetRemoteSnapshot(&result);
        assert(!result.online && result.ageMs == 1500U);
    }
    else if (strcmp(name, "mismatch") == 0 || strcmp(name, "stale_queue") == 0)
    {
        inject(0x201, values);
        if (strcmp(name, "mismatch") == 0) statusData[1] = 8;
        else now += 60U; /* Both queued frames are older than the pairing window. */
        inject(0x202, statusData);
        runTask();
        CanRemoteSnapshot result;
        Can_GetRemoteSnapshot(&result);
        assert(!result.available && !result.online);
    }
    else if (strcmp(name, "bad_frames") == 0)
    {
        rxLength = 7; inject(0x201, values); rxLength = 8;
        rxExtended = 1; inject(0x201, values); rxExtended = 0;
        rxRemote = 1; inject(0x201, values); rxRemote = 0;
        rxFd = 1; inject(0x201, values); rxFd = 0;
        inject(0x555, values);
        values[0] = 0x21; inject(0x201, values);
        inject(0x202, statusData);
        runTask();
        CanRemoteSnapshot result;
        Can_GetRemoteSnapshot(&result);
        assert(!result.available);
    }
    else if (strcmp(name, "overflow") == 0)
    {
        for (unsigned i = 0; i < 20; i++) inject(0x201, values);
        CanRemoteSnapshot result;
        Can_GetRemoteSnapshot(&result);
        assert(result.rxDropped == 4);
        runTask();
    }
    else if (strcmp(name, "tx_busy") == 0)
    {
        txFree = 0;
        runTask();
        CanRemoteSnapshot result;
        Can_GetRemoteSnapshot(&result);
        assert(txCount == 0 && result.txFailed == 1);
    }
    else assert(!"unknown H743 case");
#else
    (void)rxFd;
    assert(Motor_Init() == HAL_OK);
    Motor_UpdateProtection(25, 40);
    assert(Can_Init() == HAL_OK && started == 0);
    if (strcmp(name, "telemetry") == 0)
    {
        runTask();
        assert(txCount == 2 && txIds[0] == 0x201 && txIds[1] == 0x202);
        values[1] = 0; statusData[1] = 0; statusData[6] = 0;
        assert(memcmp(txData[0], values, 8) == 0 && memcmp(txData[1], statusData, 8) == 0);
    }
    else if (strcmp(name, "heartbeat") == 0 || strcmp(name, "timeout_wrap") == 0 ||
             strcmp(name, "stale_queue") == 0)
    {
        now = UINT32_MAX - 20U;
        CanProtocol_EncodeHeartbeat(2, 100, encoded);
        inject(0x101, encoded);
        if (strcmp(name, "stale_queue") == 0) now += 150U;
        runTask();
        CanDiagnostics diagnostics;
        Can_GetDiagnostics(&diagnostics);
        assert(diagnostics.peerOnline == (strcmp(name, "stale_queue") != 0));
        now += 150U;
        Can_GetDiagnostics(&diagnostics);
        assert(!diagnostics.peerOnline);
        assert(Motor_GetAppliedDuty() == 50); /* Heartbeat loss does not control motor. */
    }
    else if (strcmp(name, "invalid_samples") == 0)
    {
        ntc_res = current_res_ma = current_peak_ma = voltage_res = NAN;
        Motor_UpdateProtection(NAN, NAN);
        runTask();
        assert(CanProtocol_DecodeMeasurements(txData[0], &sample));
        assert(sample.temperatureDeciC == INT16_MIN && sample.currentDeciMa == UINT16_MAX &&
               sample.peakDeciMa == UINT16_MAX && sample.protection == 3);
        assert(CanProtocol_DecodeStatus(txData[1], &status));
        assert(status.voltageMv == UINT16_MAX && status.appliedDuty == 0);
    }
    else if (strcmp(name, "overflow") == 0)
    {
        CanProtocol_EncodeHeartbeat(2, 100, encoded);
        for (unsigned i = 0; i < 20; i++) inject(0x101, encoded);
        CanDiagnostics diagnostics;
        Can_GetDiagnostics(&diagnostics);
        assert(diagnostics.rxDropped == 4);
        runTask();
    }
    else if (strcmp(name, "bad_frames") == 0)
    {
        CanProtocol_EncodeHeartbeat(2, 100, encoded);
        rxLength = 7; inject(0x101, encoded); rxLength = 8;
        rxExtended = 1; inject(0x101, encoded); rxExtended = 0;
        rxRemote = 1; inject(0x101, encoded); rxRemote = 0;
        inject(0x555, encoded);
        encoded[0] = 2; inject(0x101, encoded);
        runTask();
        CanDiagnostics diagnostics;
        Can_GetDiagnostics(&diagnostics);
        assert(!diagnostics.peerOnline);
    }
    else if (strcmp(name, "tx_busy") == 0)
    {
        txFree = 0;
        runTask();
        CanDiagnostics diagnostics;
        Can_GetDiagnostics(&diagnostics);
        assert(txCount == 0 && diagnostics.txFailed == 2);
    }
    else assert(!"unknown F103 case");
#endif
    assert(Can_Send(0x800, values, 8) == HAL_ERROR);
    assert(Can_Send(0x123, values, 9) == HAL_ERROR);
    assert(Can_Send(0x123, NULL, 1) == HAL_ERROR);
    puts(name);
    return 0;
}
