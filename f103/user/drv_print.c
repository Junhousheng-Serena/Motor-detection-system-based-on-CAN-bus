//
// Created by liu on 2026/9/30.
//
#include "drv_print.h"
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdarg.h>

SemaphoreHandle_t xMutex;

int __io_putchar(int ch)
{
    uint8_t data = (uint8_t)ch;
    (void)HAL_UART_Transmit(&huart1, &data, 1U, HAL_MAX_DELAY);
    return ch;
}
void printf_init(void) {
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL) {
        Error_Handler();
    }
}

void my_printf(const char *format, ...)
{
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
        va_list args;           // 保存可变参数的访问状态
        va_start(args, format); // 从 format 后面开始读取参数

        vprintf(format, args);  // 按格式打印这些参数

        va_end(args);          // 结束参数访问
        xSemaphoreGive(xMutex);
    }
}
