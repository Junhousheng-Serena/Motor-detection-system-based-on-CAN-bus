#ifndef TEST_COMM_QUEUE_H
#define TEST_COMM_QUEUE_H
#include "FreeRTOS.h"
typedef struct
{
    uint8_t *storage;
    unsigned length, itemSize, count, head, tail;
} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
QueueHandle_t xQueueCreateStatic(unsigned length, unsigned itemSize,
                                 uint8_t *storage, StaticQueue_t *control);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *item, BaseType_t *woken);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout);
#endif
