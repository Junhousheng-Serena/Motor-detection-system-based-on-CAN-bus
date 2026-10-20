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
static uint16_t txIds[1024];
static uint8_t txData[1024][8];
static TickType_t txTicks[1024];
static void (*stepHook)(void);
static uint16_t rxId;
static uint8_t rxData[64];
static unsigned rxPending, rxLength = 8, rxExtended, rxRemote, rxFd;
static uint8_t values[8] = {2, 7, 0x2C, 1, 0xFA, 0, 50, 0x79};
static uint8_t heartbeatData[8] = {2, 9, 0, 0, 100, 0, 0, 0};
static uint8_t commandData[8] = {2, 7, 1, 50, 160, 0, 0, 0};

TickType_t xTaskGetTickCount(void) { return now; }
TickType_t xTaskGetTickCountFromISR(void) { return now; }
void vTaskDelete(void *task) { (void)task; assert(!"unexpected task deletion"); }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks != 0);
    if (++iterations >= maxIterations) longjmp(done, 1);
    now += ticks;
    if (stepHook) stepHook();
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
    assert(txCount < 1024);
    txTicks[txCount] = now;
    txIds[txCount] = id;
    memcpy(txData[txCount++], data, 8);
    return HAL_OK;
}

#ifdef TEST_H743
void Error_Handler(void) { assert(!"unexpected error handler"); }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, const FDCAN_FilterTypeDef *f)
{
    assert(h == &hfdcan1 && f->FilterType == FDCAN_FILTER_RANGE);
    assert(f->FilterID1 == 0x301 && f->FilterID2 == 0x3F1);
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

static unsigned countId(uint16_t id)
{
    unsigned n = 0;
    for (unsigned i = 0; i < txCount; i++) if (txIds[i] == id) n++;
    return n;
}
static const uint8_t *lastTx(uint16_t id)
{
    for (unsigned i = txCount; i > 0; i--) if (txIds[i-1] == id) return txData[i-1];
    assert(!"missing frame");
    return NULL;
}
static void wireTests(void)
{
    CanTelemetry t;
    CanMotorCommand c;
    CanHeartbeat h;
    CanFaultEvent f;
    uint8_t encoded[8];
    assert(CanProtocol_DecodeTelemetry(values, &t));
    assert(t.commandSequence == 7 && t.currentDeciMa == 300 &&
           t.temperatureDeciC == 250 && t.appliedDuty == 50 && t.status == 0x79);
    CanProtocol_EncodeTelemetry(&t, encoded);
    assert(memcmp(encoded, values, 8) == 0);
    encoded[4] = 0x9B; encoded[5] = 0xFF;
    assert(CanProtocol_DecodeTelemetry(encoded, &t) && t.temperatureDeciC == -101);
    encoded[7] = 0x80; assert(!CanProtocol_DecodeTelemetry(encoded, &t));
    assert(CanProtocol_DecodeCommand(commandData, &c));
    assert(c.sequence == 7 && c.enable == 1 && c.dutyPercent == 50 && c.currentLimitMa == 160);
    CanProtocol_EncodeCommand(&c, encoded);
    assert(memcmp(encoded, commandData, 8) == 0);
    encoded[6] = 1; assert(!CanProtocol_DecodeCommand(encoded, &c));
    encoded[6] = 0; encoded[4] = 161; assert(!CanProtocol_DecodeCommand(encoded, &c));
    encoded[4] = 0; assert(!CanProtocol_DecodeCommand(encoded, &c));
    encoded[4] = 160; encoded[2] = 2; assert(!CanProtocol_DecodeCommand(encoded, &c));
    encoded[2] = 1; encoded[3] = 101; assert(!CanProtocol_DecodeCommand(encoded, &c));
    assert(CanProtocol_DecodeHeartbeat(heartbeatData, &h) && h.uptimeSeconds == 100);
    CanProtocol_EncodeHeartbeat(&h, encoded);
    assert(memcmp(encoded, heartbeatData, 8) == 0);
    const uint8_t event[8] = {2, 0, 1, 7, 0x4A, 0x06, 0xFA, 0};
    assert(CanProtocol_DecodeFault(event, &f) && f.peakDeciMa == 1610 && f.temperatureDeciC == 250);
    CanProtocol_EncodeFault(&f, encoded); assert(memcmp(event, encoded, 8) == 0);
    assert(CanProtocol_SequenceNewer(0, 255));
    assert(!CanProtocol_SequenceNewer(7, 7) && !CanProtocol_SequenceNewer(6, 7));
}

#ifndef TEST_H743
static const char *scenario;
static void controlHook(void)
{
    if (strcmp(scenario, "sequence_wrap") == 0 && iterations == 1)
    { commandData[1] = 0; commandData[3] = 75; inject(0x201, commandData); }
    else if (strcmp(scenario, "disable") == 0 && iterations == 1)
    { commandData[1]++; commandData[2] = 0; inject(0x201, commandData); }
    else if (strcmp(scenario, "duplicate_disable") == 0 && iterations == 1)
    { commandData[2] = 0; inject(0x201, commandData); }
    else if (strcmp(scenario, "out_of_order") == 0 && iterations == 1)
    { commandData[1]--; commandData[3] = 90; inject(0x201, commandData); }
    else if (strcmp(scenario, "duplicates_timeout") == 0)
    { inject(0x100, heartbeatData); inject(0x201, commandData); }
    else if (strcmp(scenario, "command_timeout") == 0 || strcmp(scenario, "tick_wrap") == 0)
        inject(0x100, heartbeatData);
    else if (strcmp(scenario, "heartbeat_timeout") == 0 && iterations % 5 == 0)
    { commandData[1]++; inject(0x201, commandData); }
    else if (strcmp(scenario, "recovery_latched") == 0 && iterations == 21)
    { commandData[1]++; inject(0x100, heartbeatData); inject(0x201, commandData); }
    else if (strcmp(scenario, "fault_busy_retry") == 0 && iterations == 1) txFree = 3;
    else if (strcmp(scenario, "lower_limit") == 0 && iterations == 1)
    { commandData[1]++; commandData[4] = 35; inject(0x201, commandData); }
}
#endif

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *name = argv[1];
    wireTests();
#ifdef TEST_H743
    FDCAN_Init();
    assert(started == 0);
    if (strcmp(name, "defaults_periods") == 0)
    {
        maxIterations = 51;
        runTask();
        assert(txIds[0] == 0x100 && txIds[1] == 0x201);
        assert(countId(0x100) == 2 && countId(0x201) == 11);
        uint8_t seq = 0;
        for (unsigned i = 0; i < txCount; i++) if (txIds[i] == 0x201)
        {
            const uint8_t expected[8] = {2, seq, 1, 50, 160, 0, 0, 0};
            assert(memcmp(txData[i], expected, 8) == 0 && txTicks[i] == seq * 5U);
            seq++;
        }
    }
    else if (strcmp(name, "api") == 0)
    {
        assert(Can_SetMotorCommand(1, 60, 140) == HAL_OK);
        assert(Can_SetMotorCommand(2, 50, 160) == HAL_ERROR);
        assert(Can_SetMotorCommand(1, 101, 160) == HAL_ERROR);
        assert(Can_SetMotorCommand(1, 50, 161) == HAL_ERROR);
        assert(Can_SetMotorCommand(1, 50, 0) == HAL_ERROR);
        CanMotorCommand c; Can_GetMotorCommand(&c);
        assert(c.enable && c.dutyPercent == 60 && c.currentLimitMa == 140);
        assert(Can_SetMotorCommand(0, 0, 140) == HAL_OK);
        runTask();
        assert(CanProtocol_DecodeCommand(lastTx(0x201), &c) && !c.enable && c.currentLimitMa == 140);
    }
    else if (strcmp(name, "receive") == 0 || strcmp(name, "tick_wrap") == 0)
    {
        now = UINT32_MAX - 20U;
        inject(0x301, values); inject(0x302, heartbeatData);
        runTask();
        CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(result.online && result.ageMs == 0 && result.telemetry.currentDeciMa == 300);
        now += 25U; Can_GetRemoteSnapshot(&result);
        assert(!result.online && !result.telemetryFresh && result.heartbeatOnline && result.ageMs == 250);
        now += 125U; Can_GetRemoteSnapshot(&result);
        assert(!result.heartbeatOnline);
    }
    else if (strcmp(name, "no_heartbeat") == 0)
    {
        inject(0x301, values); runTask();
        CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(result.telemetryFresh && !result.heartbeatOnline && !result.online);
    }
    else if (strcmp(name, "stale_queue") == 0)
    {
        inject(0x301, values); inject(0x302, heartbeatData); now = 150;
        runTask(); CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(result.available && !result.telemetryFresh && !result.heartbeatOnline);
        assert(result.ageMs == 1500);
    }
    else if (strcmp(name, "fault") == 0)
    {
        const uint8_t event[8] = {2, 0, 1, 7, 0x4A, 6, 0xFA, 0};
        inject(0x3F1, event); inject(0x3F1, event); runTask();
        CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(result.faultAvailable && result.fault.code == CAN_FAULT_OVERCURRENT &&
               result.fault.peakDeciMa == 1610 && !result.online);
    }
    else if (strcmp(name, "bad_frames") == 0)
    {
        rxLength = 7; inject(0x301, values); rxLength = 8;
        rxExtended = 1; inject(0x301, values); rxExtended = 0;
        rxRemote = 1; inject(0x301, values); rxRemote = 0;
        rxFd = 1; inject(0x301, values); rxFd = 0;
        inject(0x305, values); values[0] = 1; inject(0x301, values);
        runTask(); CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(!result.available && !result.online);
    }
    else if (strcmp(name, "overflow") == 0)
    {
        for (unsigned i = 0; i < 20; i++) inject(0x301, values);
        runTask(); CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(result.rxDropped == 4);
    }
    else if (strcmp(name, "tx_busy") == 0)
    {
        txFree = 0; runTask(); CanRemoteSnapshot result; Can_GetRemoteSnapshot(&result);
        assert(txCount == 0 && result.txFailed == 2);
    }
    else assert(!"unknown H743 case");
#else
    (void)rxFd;
    assert(Motor_Init() == HAL_OK);
    assert(Can_Init() == HAL_OK && started == 0);
    if (strcmp(name, "command_before_sensors") == 0)
    {
        inject(0x100, heartbeatData); inject(0x201, commandData); runTask();
        assert(Motor_GetRemoteEnabled() && Motor_GetAppliedDuty() == 0 && htim3.ccr[0] == 0);
        assert(Motor_GetProtectionState() == MOTOR_WAITING_SENSORS);
        Motor_UpdateProtection(55, 0);
        assert(Motor_GetAppliedDuty() == 0);
        Motor_UpdateProtection(49, 30);
        assert(Motor_GetAppliedDuty() == 50 && htim3.ccr[0] == 1800);
        puts(name);
        return 0;
    }
    Motor_UpdateProtection(25, 40);
    assert(Motor_GetAppliedDuty() == 0 && htim3.ccr[0] == 0);
    assert(Motor_SetForwardDuty(50) == HAL_BUSY); /* Local API cannot bypass CAN gate. */
    scenario = name;
    stepHook = controlHook;
    if (strcmp(name, "boot_wait") == 0)
    {
        runTask(); assert(Motor_GetAppliedDuty() == 0 && Motor_GetProtectionState() == MOTOR_READY);
        CanTelemetry t; assert(CanProtocol_DecodeTelemetry(lastTx(0x301), &t));
        assert(t.currentDeciMa == 300 && t.temperatureDeciC == 250 && t.appliedDuty == 0);
        assert(t.status == 1 && countId(0x302) == 1);
    }
    else if (strcmp(name, "startup_timeout") == 0)
    {
        maxIterations = 151; runTask();
        assert(Motor_GetProtectionState() == MOTOR_COMM_FAULT_LATCHED && Motor_GetAppliedDuty() == 0);
    }
    else if (strcmp(name, "no_heartbeat") == 0 || strcmp(name, "stale_queue") == 0)
    {
        if (strcmp(name, "stale_queue") == 0) inject(0x100, heartbeatData);
        inject(0x201, commandData);
        if (strcmp(name, "stale_queue") == 0) now += 20;
        runTask(); CanDiagnostics d; Can_GetDiagnostics(&d);
        assert(!d.commandSeen && Motor_GetAppliedDuty() == 0);
    }
    else if (strcmp(name, "bad_frames") == 0)
    {
        inject(0x100, heartbeatData);
        rxLength = 7; inject(0x201, commandData); rxLength = 8;
        rxExtended = 1; inject(0x201, commandData); rxExtended = 0;
        rxRemote = 1; inject(0x201, commandData); rxRemote = 0;
        inject(0x555, commandData); commandData[0] = 1; inject(0x201, commandData);
        commandData[0] = 2; commandData[6] = 1; inject(0x201, commandData);
        commandData[6] = 0; commandData[4] = 161; inject(0x201, commandData);
        runTask(); CanDiagnostics d; Can_GetDiagnostics(&d);
        assert(!d.commandSeen && Motor_GetAppliedDuty() == 0);
    }
    else if (strcmp(name, "overflow") == 0)
    {
        for (unsigned i = 0; i < 20; i++) inject(0x100, heartbeatData);
        runTask(); CanDiagnostics d; Can_GetDiagnostics(&d); assert(d.rxDropped == 4);
    }
    else if (strcmp(name, "tx_busy") == 0)
    {
        txFree = 0; runTask(); CanDiagnostics d; Can_GetDiagnostics(&d);
        assert(txCount == 0 && d.txFailed == 2);
    }
    else if (strcmp(name, "remote_current_boundary") == 0)
    {
        commandData[4] = 100;
        inject(0x100, heartbeatData); inject(0x201, commandData); runTask();
        assert(Motor_GetCurrentLimitMa() == 100 && Motor_GetAppliedDuty() == 50);
        Motor_UpdateProtection(25, 99.9f);
        assert(Motor_GetAppliedDuty() == 50);
        Motor_UpdateProtection(25, 100);
        assert(Motor_GetProtectionState() == MOTOR_OVERCURRENT_LATCHED && Motor_GetAppliedDuty() == 0);
    }
    else if (strcmp(name, "fault_snapshot") == 0 || strcmp(name, "fault_busy_retry") == 0 ||
             strcmp(name, "invalid_samples") == 0)
    {
        if (strcmp(name, "invalid_samples") == 0)
        { ntc_res = current_res_ma = NAN; Motor_UpdateProtection(NAN, NAN); }
        else
        { Motor_UpdateProtection(25, 161); Motor_UpdateProtection(26, 0); }
        if (strcmp(name, "fault_busy_retry") == 0) txFree = 0;
        maxIterations = 16; runTask();
        CanFaultEvent f; assert(CanProtocol_DecodeFault(lastTx(0x3F1), &f));
        assert(countId(0x3F1) == 3 && Motor_GetAppliedDuty() == 0);
        if (strcmp(name, "invalid_samples") == 0)
        {
            assert(f.code == CAN_FAULT_SENSOR && f.peakDeciMa == UINT16_MAX);
            CanTelemetry t; assert(CanProtocol_DecodeTelemetry(lastTx(0x301), &t));
            assert(t.currentDeciMa == UINT16_MAX && t.temperatureDeciC == INT16_MIN);
        }
        else assert(f.code == CAN_FAULT_OVERCURRENT && f.peakDeciMa == 1610 && f.temperatureDeciC == 250);
    }
    else
    {
        if (strcmp(name, "sequence_wrap") == 0) commandData[1] = 255;
        if (strcmp(name, "tick_wrap") == 0) now = UINT32_MAX - 10U;
        inject(0x100, heartbeatData); inject(0x201, commandData);
        if (strcmp(name, "enable") == 0) maxIterations = 1;
        else if (strcmp(name, "command_timeout") == 0 || strcmp(name, "duplicates_timeout") == 0 ||
                 strcmp(name, "tick_wrap") == 0) maxIterations = 21;
        else if (strcmp(name, "heartbeat_timeout") == 0) maxIterations = 151;
        else if (strcmp(name, "recovery_latched") == 0) maxIterations = 22;
        else if (strcmp(name, "disable") == 0 || strcmp(name, "duplicate_disable") == 0 ||
                 strcmp(name, "out_of_order") == 0 || strcmp(name, "sequence_wrap") == 0 ||
                 strcmp(name, "lower_limit") == 0) maxIterations = 2;
        else assert(!"unknown F103 case");
        runTask();
        CanDiagnostics d; Can_GetDiagnostics(&d);
        if (strcmp(name, "enable") == 0 || strcmp(name, "out_of_order") == 0)
        { assert(Motor_GetAppliedDuty() == 50 && htim3.ccr[0] == 1800 && d.commandSequence == 7); }
        else if (strcmp(name, "sequence_wrap") == 0)
        { assert(Motor_GetAppliedDuty() == 75 && d.commandSequence == 0); }
        else if (strcmp(name, "disable") == 0 || strcmp(name, "duplicate_disable") == 0)
        { assert(!Motor_GetRemoteEnabled() && Motor_GetAppliedDuty() == 0 && Motor_GetProtectionState() == MOTOR_READY); }
        else
        {
            MotorProtectionState expected = strcmp(name, "lower_limit") == 0 ?
                MOTOR_OVERCURRENT_LATCHED : MOTOR_COMM_FAULT_LATCHED;
            assert(Motor_GetAppliedDuty() == 0 && Motor_GetProtectionState() == expected);
            Motor_UpdateProtection(25, 0);
            assert(Motor_ApplyRemoteCommand(1, 100, 160) == HAL_BUSY && Motor_GetAppliedDuty() == 0);
            CanFaultEvent f; assert(CanProtocol_DecodeFault(lastTx(0x3F1), &f));
            assert(f.code == (expected == MOTOR_OVERCURRENT_LATCHED ? CAN_FAULT_OVERCURRENT : CAN_FAULT_COMMUNICATION));
        }
    }
#endif
    assert(Can_Send(0x800, values, 8) == HAL_ERROR);
    assert(Can_Send(0x123, values, 9) == HAL_ERROR);
    assert(Can_Send(0x123, NULL, 1) == HAL_ERROR);
    puts(name);
    return 0;
}
