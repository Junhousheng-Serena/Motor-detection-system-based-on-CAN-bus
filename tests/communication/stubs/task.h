#ifndef TEST_COMM_TASK_H
#define TEST_COMM_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);
void vTaskDelay(TickType_t ticks);
void vTaskDelete(void *task);
#endif
