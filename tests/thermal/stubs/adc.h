#ifndef TEST_ADC_H
#define TEST_ADC_H
#include "main.h"
#define ADC1 ((void *)1)
typedef struct { void *Instance; } ADC_HandleTypeDef;
extern ADC_HandleTypeDef hadc1;
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *data, uint32_t count);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *h);
#endif
