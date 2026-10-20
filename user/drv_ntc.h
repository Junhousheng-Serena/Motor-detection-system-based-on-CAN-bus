#ifndef USER_DRV_NTC_H
#define USER_DRV_NTC_H

#include "main.h"

#define ADC_CHANNEL_COUNT       2U
#define ADC_SAMPLES_PER_CHANNEL 15U
#define ADC_BUF_LEN             (ADC_CHANNEL_COUNT * ADC_SAMPLES_PER_CHANNEL)
#define ADC_NTC_INDEX           0U  /* Rank 1: PA4 / ADC1_IN4 */
#define ADC_VOLTAGE_INDEX       1U  /* Rank 2: PA3 / ADC1_IN3 */
#define ADC_FULL_SCALE          4095U
#define ADC_REFERENCE_VOLTAGE   3.3f /* Set to the measured VDDA voltage. */
#define NTC_SAMPLE_INTERVAL_MS 10U  /* One tick at configTICK_RATE_HZ = 100. */
#define NTC_SAMPLE_TIMEOUT_MS  20U
#define NTC_REPORT_INTERVAL_MS 200U
#define NTC_ADC_FAULT_MARGIN   8U
/* Raw interleaved DMA data is kept intact; channel buffers are sorted. */
extern uint16_t adc_buf[ADC_BUF_LEN];
extern uint16_t ntc_samples[ADC_SAMPLES_PER_CHANNEL];
extern uint16_t voltage_samples[ADC_SAMPLES_PER_CHANNEL];
extern volatile float ntc_res;
extern volatile float voltage_res; /* ADC pin voltage in volts, not current. */
extern volatile float current_res_ma;  /* Median-based display, not PWM average. */
extern volatile float current_peak_ma; /* Largest sampled current in this batch. */

/* Call after Motor_Init() and printf_init(), before starting the scheduler.
 * Calibrate ADC1 and create separate sampling/protection and logging tasks. */
HAL_StatusTypeDef Ntc_Init(void);


#endif /* USER_DRV_NTC_H */
