#ifndef USER_DRV_MOTOR_H
#define USER_DRV_MOTOR_H

#include "main.h"

#define MOTOR_OVERHEAT_STOP_C       60U
#define MOTOR_START_MAX_TEMP_C      50U
#define MOTOR_DEFAULT_DUTY_PERCENT  50U
/* Provisional bench limit, not a motor rating. No startup blanking. */
#define MOTOR_OVERCURRENT_STOP_MA   160U

#if MOTOR_START_MAX_TEMP_C >= MOTOR_OVERHEAT_STOP_C
#error "Motor startup temperature must be below the overheat threshold"
#endif

#if MOTOR_OVERCURRENT_STOP_MA == 0
#error "Motor overcurrent threshold must be positive"
#endif

typedef enum
{
  MOTOR_WAITING_SENSORS = 0,
  MOTOR_READY,
  MOTOR_OVERHEAT_LATCHED,
  MOTOR_SENSOR_FAULT_LATCHED,
  MOTOR_OVERCURRENT_LATCHED
} MotorProtectionState;



/* DRV8833: PA6/TIM3_CH1 -> AIN1, PA7/TIM3_CH2 -> AIN2.
 * Start both PWM channels at zero duty. A valid temperature/current pair
 * enables default duty only within startup limits. Re-init cannot clear faults. */
HAL_StatusTypeDef Motor_Init(void);

/* Remember the requested 0..100% duty. HAL_BUSY means sensor protection
 * currently inhibits output; a speed command cannot clear a fault. Task only. */
HAL_StatusTypeDef Motor_SetForwardDuty(uint32_t dutyPercent);

/* Manual coast stop: also clear the requested duty, without clearing faults. */
void Motor_Stop(void);

/* Check one complete batch atomically, before enabling PWM. currentPeakMa is
 * the largest current sample in the batch, not the filtered display value.
 * Pass NAN on acquisition failure. Faults remain latched until MCU reset.
 * Task only. */
void Motor_UpdateProtection(float temperatureC, float currentPeakMa);
MotorProtectionState Motor_GetProtectionState(void);


#endif /* USER_DRV_MOTOR_H */
