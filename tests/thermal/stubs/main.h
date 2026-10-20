#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
extern uint32_t testIrqMask;
static inline uint32_t __get_PRIMASK(void) { return testIrqMask; }
static inline void __disable_irq(void) { testIrqMask = 1; }
static inline void __set_PRIMASK(uint32_t mask) { testIrqMask = mask; }
#endif
