#ifndef TEST_COMM_FREERTOS_H
#define TEST_COMM_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
#define pdPASS 1
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((ms) / 10U)
#define portTICK_PERIOD_MS 10U
#define portYIELD_FROM_ISR(woken) ((void)(woken))
#endif
