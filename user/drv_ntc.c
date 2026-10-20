#include "drv_ntc.h"
#include "adc.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "drv_print.h"
#include "drv_motor.h"

static TaskHandle_t ntcTaskHandle = NULL;
uint16_t adc_buf[ADC_BUF_LEN];
uint16_t ntc_samples[ADC_SAMPLES_PER_CHANNEL];
uint16_t voltage_samples[ADC_SAMPLES_PER_CHANNEL];
volatile float ntc_res = NAN;
volatile float voltage_res = NAN;

static void vNtcMonitorTask(void *pvParameters);
static void vNtcReportTask(void *pvParameters);

HAL_StatusTypeDef Ntc_Init(void) {
    HAL_StatusTypeDef status = HAL_ADCEx_Calibration_Start(&hadc1);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Protection must run above CanRx (priority 2), without serial output. */
    if (xTaskCreate(vNtcMonitorTask, "NTC", 384, NULL, 3,
                  &ntcTaskHandle) != pdPASS)
    {
        return HAL_ERROR;
    }
    if (xTaskCreate(vNtcReportTask, "NtcLog", 512, NULL, 1, NULL) != pdPASS)
    {
        vTaskDelete(ntcTaskHandle);
        ntcTaskHandle = NULL;
        return HAL_ERROR;
    }

    /* DMA starts inside the task, after the scheduler is running. */
    return HAL_OK;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && ntcTaskHandle != NULL)
    {
        BaseType_t taskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(ntcTaskHandle, &taskWoken);
        portYIELD_FROM_ISR(taskWoken);
    }
}

static int cmp(const void *a, const void *b)
{
    uint16_t x = *(const uint16_t *)a;
    uint16_t y = *(const uint16_t *)b;

    return x - y;
}
static float Ntc_AdcToTemperature(float adc)
{
    if (adc <= NTC_ADC_FAULT_MARGIN || adc >= ADC_FULL_SCALE - NTC_ADC_FAULT_MARGIN)
    {
        return NAN;  // 无效采样值
    }

    float resistance = 10000.0f * adc / (ADC_FULL_SCALE - adc);

    float temperatureK =
        1.0f / (1.0f / 298.15f
              + logf(resistance / 10000.0f) / 3950.0f);

    return temperatureK - 273.15f;
}
static void vNtcMonitorTask(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        float temperature = NAN;
        float voltage = NAN;
        /* Discard any old notification before arming a new one-shot batch. */
        (void)ulTaskNotifyTake(pdTRUE, 0);
        HAL_StatusTypeDef startStatus =
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
        uint32_t completed = 0U;
        if (startStatus == HAL_OK)
        {
            completed = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(NTC_SAMPLE_TIMEOUT_MS));
        }
        HAL_StatusTypeDef stopStatus = HAL_ADC_Stop_DMA(&hadc1);

        if (startStatus == HAL_OK && completed != 0U && stopStatus == HAL_OK)
        {
            /* Rank order: NTC0, voltage0, NTC1, voltage1, ... */
            for (uint32_t i = 0U; i < ADC_SAMPLES_PER_CHANNEL; i++)
            {
                ntc_samples[i] = adc_buf[ADC_CHANNEL_COUNT * i + ADC_NTC_INDEX];
                voltage_samples[i] = adc_buf[ADC_CHANNEL_COUNT * i + ADC_VOLTAGE_INDEX];
            }
            qsort(ntc_samples, ADC_SAMPLES_PER_CHANNEL, sizeof(uint16_t), cmp);
            qsort(voltage_samples, ADC_SAMPLES_PER_CHANNEL, sizeof(uint16_t), cmp);
            temperature = Ntc_AdcToTemperature(
                (float)ntc_samples[ADC_SAMPLES_PER_CHANNEL / 2U]);
            voltage = (float)voltage_samples[ADC_SAMPLES_PER_CHANNEL / 2U] *
                      ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE;
        }

        /* Invalid data / DMA errors / timeout all latch the motor off. */
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        Motor_UpdateTemperature(temperature);
        ntc_res = temperature;
        voltage_res = voltage;
        __set_PRIMASK(irqState);
        vTaskDelay(pdMS_TO_TICKS(NTC_SAMPLE_INTERVAL_MS));
    }
}

static void vNtcReportTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        /* Snapshot one complete batch before formatting can be preempted. */
        uint32_t irqState = __get_PRIMASK();
        __disable_irq();
        float temperature = ntc_res;
        float voltage = voltage_res;
        MotorProtectionState protection = Motor_GetProtectionState();
        __set_PRIMASK(irqState);
        const char *state;
        switch (protection)
        {
        case MOTOR_TEMPERATURE_READY:
            state = "READY";
            break;
        case MOTOR_OVERHEAT_LATCHED:
            state = "OVERHEAT_LATCHED";
            break;
        case MOTOR_SENSOR_FAULT_LATCHED:
            state = "SENSOR_FAULT_LATCHED";
            break;
        default:
            state = "WAIT_TEMP";
            break;
        }
        my_printf("ntc_res: %.1f C, voltage: %.3f V, motor: %s\r\n",
                  (double)temperature, (double)voltage, state);
        vTaskDelay(pdMS_TO_TICKS(NTC_SAMPLE_INTERVAL_MS));
    }
}
