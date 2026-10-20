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
  MOTOR_OVERCURRENT_LATCHED,
  MOTOR_COMM_FAULT_LATCHED
} MotorProtectionState;

typedef struct
{
  MotorProtectionState state;
  float temperatureC;
  float currentPeakMa;
} MotorFaultSnapshot;


/* DRV8833: PA6/TIM3_CH1 -> AIN1, PA7/TIM3_CH2 -> AIN2.
 * Start both PWM channels at zero duty. Output requires valid sensors within
 * startup limits and, in CAN mode, remote enable. Re-init cannot clear faults. */
HAL_StatusTypeDef Motor_Init(void);

/* Remember the requested 0..100% duty. HAL_BUSY means protection or remote
 * enable currently inhibits output; duty commands cannot clear faults. Task only. */
HAL_StatusTypeDef Motor_SetForwardDuty(uint32_t dutyPercent);

/* Manual coast stop: also clear the requested duty, without clearing faults. */
void Motor_Stop(void);

/* Check one complete batch atomically, before enabling PWM. currentPeakMa is
 * the largest current sample in the batch, not the filtered display value.
 * Pass NAN on acquisition failure. Faults remain latched until MCU reset.
 * Task only. */
void Motor_UpdateProtection(float temperatureC, float currentPeakMa);
MotorProtectionState Motor_GetProtectionState(void);
uint8_t Motor_GetAppliedDuty(void);
uint8_t Motor_GetRequestedDuty(void);
uint8_t Motor_GetRemoteEnabled(void);
uint16_t Motor_GetCurrentLimitMa(void);

/* Called before scheduling by Can_Init(). Local duty requests cannot bypass
 * this gate. Only a validated remote command can grant enable. */
void Motor_RequireRemoteControl(void);
HAL_StatusTypeDef Motor_ApplyRemoteCommand(uint8_t enable, uint8_t dutyPercent,
                                         uint16_t currentLimitMa);
void Motor_LatchCommunicationFault(void);
void Motor_GetFaultSnapshot(MotorFaultSnapshot *snapshot);


#endif /* USER_DRV_MOTOR_H */
