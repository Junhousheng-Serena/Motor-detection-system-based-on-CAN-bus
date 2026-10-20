#ifndef TEST_TIM_H
#define TEST_TIM_H
#include "main.h"
typedef struct { uint32_t arr, ccr[2]; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim3;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->arr)
#define __HAL_TIM_SET_COMPARE(h, ch, value) ((h)->ccr[ch] = (value))
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t channel);
#endif
