#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "tim.h"
#include "adc.h"
#include "task.h"
#include "drv_motor.h"
#include "drv_ntc.h"
#include "drv_current.h"

uint32_t testIrqMask;
TIM_HandleTypeDef htim3 = {3599, {0, 0}};
ADC_HandleTypeDef hadc1 = {ADC1};
static TaskFunction_t sampleTask;
static TaskFunction_t reportTask;
static jmp_buf nextIteration;
static uint16_t adcSample = 2048;
static uint16_t voltageSample = 1000;
static int useNoisySamples;
static int injectCurrentSpike;
static unsigned nonzeroPwmWrites;
static TickType_t lastDelay;
static const uint16_t ntcPattern[15] = {
    2048, 4095, 2044, 2050, 2046, 0, 2049, 2047,
    2051, 2048, 2045, 2052, 2048, 2043, 2053
};
static const uint16_t voltagePattern[15] = {
    3100, 0, 3104, 3098, 3102, 4095, 3099, 3101,
    3097, 3100, 3103, 3096, 3100, 3105, 3095
};
static char lastLog[128];
static uint32_t notification = 1;
static unsigned dmaStarts, printCalls;
static HAL_StatusTypeDef dmaStartStatus = HAL_OK, dmaStopStatus = HAL_OK;

void testSetCompare(TIM_HandleTypeDef *h, uint32_t channel, uint32_t value)
{
    h->ccr[channel] = value;
    if (value != 0) nonzeroPwmWrites++;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel)
{
    assert(h->ccr[channel] == 0);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *data, uint32_t count)
{
    (void)h;
    dmaStarts++;
    assert(count == 30); /* 15 complete pairs, not 15 interleaved values. */
    for (uint32_t i = 0; i < 15; i++)
    {
        ((uint16_t *)data)[2 * i] = useNoisySamples ? ntcPattern[i] : adcSample;
        ((uint16_t *)data)[2 * i + 1] =
            useNoisySamples ? voltagePattern[i] : voltageSample;
    }
    if (injectCurrentSpike) ((uint16_t *)data)[17] = 4095;
    return dmaStartStatus;
}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *h)
{
    (void)h;
    return dmaStopStatus;
}
BaseType_t xTaskCreate(TaskFunction_t task, const char *name, size_t stack,
                      void *arg, UBaseType_t priority, TaskHandle_t *handle)
{
    (void)stack; (void)arg;
    if (strcmp(name, "NTC") == 0)
    {
        sampleTask = task;
        assert(priority > 2);
    }
    else
    {
        assert(priority == 1);
        reportTask = task;
    }
    if (handle) *handle = (void *)1;
    return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { (void)task; }
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{
    (void)task; (void)woken;
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t timeout)
{
    (void)clear;
    return timeout ? notification : 0;
}
void vTaskDelay(TickType_t ticks)
{
    assert(ticks > 0);
    lastDelay = ticks;
    longjmp(nextIteration, 1);
}
void my_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(lastLog, sizeof(lastLog), format, args);
    va_end(args);
    printCalls++;
}

static void expectOff(void)
{
    assert(htim3.ccr[0] == 0 && htim3.ccr[1] == 0);
}
static void expectRunning(void)
{
    assert(htim3.ccr[0] == 1800 && htim3.ccr[1] == 0);
}
static void sampleOnce(void)
{
    if (setjmp(nextIteration) == 0) sampleTask(NULL);
    assert(printCalls == 0); /* UART is absent from the protection path. */
    assert(lastDelay == pdMS_TO_TICKS(10));
}
static void checkFaultCannotRestart(MotorProtectionState expected)
{
    expectOff();
    assert(Motor_GetProtectionState() == expected);
    Motor_UpdateProtection(25.0f, 0.0f);
    assert(Motor_SetForwardDuty(100) == HAL_BUSY);
    assert(Motor_Init() == HAL_OK);
    expectOff();
    assert(Motor_GetProtectionState() == expected);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    assert(Motor_Init() == HAL_OK);
    expectOff();
    if (strcmp(argv[1], "startup") == 0)
    {
        assert(Motor_SetForwardDuty(50) == HAL_BUSY);
        expectOff();
        Motor_UpdateProtection(55, 0);
        expectOff();
        Motor_UpdateProtection(50, 0);
        expectOff();
        Motor_UpdateProtection(49.9f, 0);
        expectRunning();
        Motor_UpdateProtection(59.9f, 0);
        expectRunning();
        assert(Motor_SetForwardDuty(101) == HAL_ERROR);
        expectRunning();
    }
    else if (strcmp(argv[1], "overheat") == 0)
    {
        Motor_UpdateProtection(25, 0);
        expectRunning();
        Motor_UpdateProtection(60, 0);
        checkFaultCannotRestart(MOTOR_OVERHEAT_LATCHED);
    }
    else if (strcmp(argv[1], "hot_boot") == 0)
    {
        Motor_UpdateProtection(60, 0);
        checkFaultCannotRestart(MOTOR_OVERHEAT_LATCHED);
    }
    else if (strcmp(argv[1], "invalid_temperature") == 0)
    {
        Motor_UpdateProtection(25, 0);
        Motor_UpdateProtection(NAN, 0);
        checkFaultCannotRestart(MOTOR_SENSOR_FAULT_LATCHED);
    }
    else if (strcmp(argv[1], "infinite_temperature") == 0)
    {
        Motor_UpdateProtection(INFINITY, 0);
        checkFaultCannotRestart(MOTOR_SENSOR_FAULT_LATCHED);
    }
    else if (strcmp(argv[1], "manual_stop") == 0)
    {
        Motor_UpdateProtection(25, 0);
        Motor_Stop();
        Motor_UpdateProtection(26, 0);
        expectOff();
        assert(Motor_SetForwardDuty(75) == HAL_OK);
        assert(htim3.ccr[0] == 2700);
        testIrqMask = 1;
        Motor_Stop();
        assert(testIrqMask == 1);
        testIrqMask = 0;
    }
    else if (strcmp(argv[1], "overcurrent") == 0)
    {
        Motor_UpdateProtection(25, MOTOR_OVERCURRENT_STOP_MA - 0.1f);
        expectRunning();
        Motor_UpdateProtection(25, MOTOR_OVERCURRENT_STOP_MA);
        checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
        Motor_UpdateProtection(70, 0); /* Preserve the original fault reason. */
        assert(Motor_GetProtectionState() == MOTOR_OVERCURRENT_LATCHED);
        Motor_Stop();
        checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
    }
    else if (strcmp(argv[1], "overcurrent_boot") == 0)
    {
        Motor_UpdateProtection(25, MOTOR_OVERCURRENT_STOP_MA + 10.0f);
        assert(nonzeroPwmWrites == 0);
        checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
    }
    else if (strcmp(argv[1], "invalid_current") == 0 ||
             strcmp(argv[1], "infinite_current") == 0 ||
             strcmp(argv[1], "negative_current") == 0)
    {
        float current = strcmp(argv[1], "invalid_current") == 0 ? NAN :
                        strcmp(argv[1], "infinite_current") == 0 ? INFINITY : -1.0f;
        Motor_UpdateProtection(25, current);
        assert(nonzeroPwmWrites == 0);
        checkFaultCannotRestart(MOTOR_SENSOR_FAULT_LATCHED);
    }
    else if (strcmp(argv[1], "current_conversion") == 0)
    {
        assert(Current_VoltageToMilliamps(0) == 0);
        assert(fabsf(Current_VoltageToMilliamps(0.6f) - 30.0f) < 0.0001f);
        assert(fabsf(Current_VoltageToMilliamps(3.0f) - 150.0f) < 0.0001f);
        assert(fabsf(Current_VoltageToMilliamps(3.2f) - 160.0f) < 0.0001f);
        assert(isnan(Current_VoltageToMilliamps(NAN)));
        assert(isnan(Current_VoltageToMilliamps(INFINITY)));
        assert(isnan(Current_VoltageToMilliamps(-0.1f)));
    }
    else if (strcmp(argv[1], "adc_overcurrent_boot") == 0)
    {
        assert(Ntc_Init() == HAL_OK);
        voltageSample = 4095;
        sampleOnce();
        assert(nonzeroPwmWrites == 0); /* No PWM pulse before checking current. */
        checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
    }
    else
    {
        assert(Ntc_Init() == HAL_OK);
        assert(dmaStarts == 0); /* No ISR notification before scheduler startup. */
        sampleOnce();
        expectRunning();
        assert(fabsf(ntc_res - 25.0f) < 0.1f);
        assert(fabsf(voltage_res - 0.8058608f) < 0.00001f);
        assert(fabsf(current_res_ma - 40.29304f) < 0.0001f);
        assert(fabsf(current_peak_ma - 40.29304f) < 0.0001f);
        if (strcmp(argv[1], "adc_split_filter") == 0)
        {
            useNoisySamples = 1;
            sampleOnce();
            /* The median remains useful, but the 4095 spike must stop PWM. */
            checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
            assert(fabsf(ntc_res - 25.0f) < 0.1f);
            assert(fabsf(voltage_res - 2.4981685f) < 0.00001f);
            assert(fabsf(current_res_ma - 124.90843f) < 0.0001f);
            assert(fabsf(current_peak_ma - 165.0f) < 0.0001f);
            assert(ntc_samples[7] == 2048);
            assert(voltage_samples[7] == 3100);
            for (unsigned i = 0; i < 15; i++)
            {
                /* Sorting channel copies must not scramble the DMA buffer. */
                assert(adc_buf[2 * i] == ntcPattern[i]);
                assert(adc_buf[2 * i + 1] == voltagePattern[i]);
            }
            goto finished;
        }
        if (strcmp(argv[1], "adc_voltage_endpoints") == 0)
        {
            voltageSample = 0;
            sampleOnce();
            expectRunning();
            assert(voltage_res == 0.0f);
            assert(current_res_ma == 0.0f && current_peak_ma == 0.0f);
            voltageSample = 4095;
            sampleOnce();
            checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
            assert(fabsf(voltage_res - 3.3f) < 0.00001f);
            assert(fabsf(current_res_ma - 165.0f) < 0.0001f);
            goto finished;
        }
        if (strcmp(argv[1], "adc_report") == 0)
        {
            if (setjmp(nextIteration) == 0) reportTask(NULL);
            assert(printCalls == 1);
            assert(strstr(lastLog, "ntc_res: 25.0 C") != NULL);
            assert(strstr(lastLog, "voltage: 0.806 V") != NULL);
            assert(strstr(lastLog, "current: 40.3 mA") != NULL);
            assert(strstr(lastLog, "peak: 40.3 mA") != NULL);
            assert(strstr(lastLog, "motor: READY") != NULL);
            assert(lastDelay == pdMS_TO_TICKS(200));
            goto finished;
        }
        if (strcmp(argv[1], "adc_current_boundary") == 0)
        {
            voltageSample = 3970; /* 159.963 mA, immediately below the limit. */
            sampleOnce();
            expectRunning();
            voltageSample = 3971; /* 160.004 mA, first code above the limit. */
            sampleOnce();
            assert(current_peak_ma > 160.0f && current_peak_ma < 160.1f);
            checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
            voltageSample = 0;
            sampleOnce();
            expectOff();
            goto finished;
        }
        if (strcmp(argv[1], "adc_current_spike") == 0 ||
            strcmp(argv[1], "adc_current_fault_report") == 0)
        {
            injectCurrentSpike = 1;
            sampleOnce();
            /* 14 normal samples and one overcurrent: median is still normal. */
            assert(fabsf(current_res_ma - 40.29304f) < 0.0001f);
            assert(fabsf(current_peak_ma - 165.0f) < 0.0001f);
            checkFaultCannotRestart(MOTOR_OVERCURRENT_LATCHED);
            injectCurrentSpike = 0;
            voltageSample = 0;
            sampleOnce();
            expectOff();
            assert(Motor_GetProtectionState() == MOTOR_OVERCURRENT_LATCHED);
            if (strcmp(argv[1], "adc_current_fault_report") == 0)
            {
                if (setjmp(nextIteration) == 0) reportTask(NULL);
                assert(strstr(lastLog, "motor: OVERCURRENT_LATCHED") != NULL);
                assert(printCalls == 1);
            }
            goto finished;
        }
        if (strcmp(argv[1], "adc_overheat") == 0) adcSample = 700;
        else if (strcmp(argv[1], "adc_open") == 0) adcSample = 4094;
        else if (strcmp(argv[1], "adc_short") == 0) adcSample = 1;
        else if (strcmp(argv[1], "adc_timeout") == 0) notification = 0;
        else if (strcmp(argv[1], "adc_start_error") == 0) dmaStartStatus = HAL_ERROR;
        else if (strcmp(argv[1], "adc_stop_error") == 0) dmaStopStatus = HAL_ERROR;
        else assert(!"unknown test case");
        sampleOnce();
        if (strcmp(argv[1], "adc_timeout") == 0 ||
            strcmp(argv[1], "adc_start_error") == 0 ||
            strcmp(argv[1], "adc_stop_error") == 0)
        {
            assert(isnan(ntc_res) && isnan(voltage_res));
            assert(isnan(current_res_ma) && isnan(current_peak_ma));
        }
        else
        {
            /* Invalid NTC data must not invalidate the other channel. */
            assert(fabsf(voltage_res - 0.8058608f) < 0.00001f);
            assert(fabsf(current_res_ma - 40.29304f) < 0.0001f);
        }
        MotorProtectionState expected = strcmp(argv[1], "adc_overheat") == 0 ?
            MOTOR_OVERHEAT_LATCHED : MOTOR_SENSOR_FAULT_LATCHED;
        checkFaultCannotRestart(expected);
        adcSample = 2048;
        notification = 1;
        dmaStartStatus = dmaStopStatus = HAL_OK;
        sampleOnce();
        expectOff();
        assert(fabsf(ntc_res - 25.0f) < 0.1f);
        assert(fabsf(voltage_res - 0.8058608f) < 0.00001f);
    }
finished:
    assert(testIrqMask == 0);
    puts(argv[1]);
    return 0;
}
