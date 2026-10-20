#ifndef USER_DRV_MOTOR_H
#define USER_DRV_MOTOR_H

#include "main.h"

#define MOTOR_OVERHEAT_STOP_C       60U
#define MOTOR_START_MAX_TEMP_C      50U
#define MOTOR_DEFAULT_DUTY_PERCENT  50U

#if MOTOR_START_MAX_TEMP_C >= MOTOR_OVERHEAT_STOP_C
#error "Motor startup temperature must be below the overheat threshold"
#endif

typedef enum
{
  MOTOR_WAITING_TEMPERATURE = 0,
  MOTOR_TEMPERATURE_READY,
  MOTOR_OVERHEAT_LATCHED,
  MOTOR_SENSOR_FAULT_LATCHED
} MotorProtectionState;



/* DRV8833: PA6/TIM3_CH1 -> AIN1, PA7/TIM3_CH2 -> AIN2.
 * Start both PWM channels at zero duty. A valid, cool sample enables the
 * default duty. Calling this again does not clear a latched fault. */
HAL_StatusTypeDef Motor_Init(void);

/* Remember the requested 0..100% duty. HAL_BUSY means temperature protection
 * currently inhibits output; a speed command cannot clear a fault. Task only. */
HAL_StatusTypeDef Motor_SetForwardDuty(uint32_t dutyPercent);

/* Manual coast stop: also clear the requested duty, without clearing faults. */
void Motor_Stop(void);

/* Called for each sample, or with NAN on acquisition failure. Faults remain
 * latched until MCU reset; cooling alone never restarts the motor. Task only. */
void Motor_UpdateTemperature(float temperatureC);
MotorProtectionState Motor_GetProtectionState(void);


#endif /* USER_DRV_MOTOR_H */
