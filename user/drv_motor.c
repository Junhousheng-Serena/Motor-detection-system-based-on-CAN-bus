#include "drv_motor.h"
#include "tim.h"
#include <math.h>

static uint32_t requestedDutyPercent = MOTOR_DEFAULT_DUTY_PERCENT;
static volatile MotorProtectionState protectionState = MOTOR_WAITING_SENSORS;
static uint8_t motorInitialized;

/* Caller protects the state and PWM update together from task preemption. */
static void Motor_ApplyDuty(void)
{
  uint32_t duty = (protectionState == MOTOR_READY) ?
                   requestedDutyPercent : 0U;
  uint32_t pulse = ((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U) * duty) / 100U;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}


HAL_StatusTypeDef Motor_Init(void)
{
  if (motorInitialized)
  {
    return HAL_OK;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);

  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  if (status == HAL_OK)
  {
    status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  }
  if (status == HAL_OK)
  {
    motorInitialized = 1U;
  }
  return status;
}

HAL_StatusTypeDef Motor_SetForwardDuty(uint32_t dutyPercent)
{
  if (!motorInitialized || dutyPercent > 100U)
  {
    return HAL_ERROR;
  }

  uint32_t irqState = __get_PRIMASK();
  __disable_irq();
  requestedDutyPercent = dutyPercent;
  Motor_ApplyDuty();
  HAL_StatusTypeDef status = (protectionState == MOTOR_READY) ?
                              HAL_OK : HAL_BUSY;
  __set_PRIMASK(irqState);
  return status;
}

void Motor_Stop(void)
{
  uint32_t irqState = __get_PRIMASK();
  __disable_irq();
  requestedDutyPercent = 0U;
  if (motorInitialized)
  {
    Motor_ApplyDuty();
  }
  __set_PRIMASK(irqState);
}

void Motor_UpdateProtection(float temperatureC, float currentPeakMa)
{
  uint32_t irqState = __get_PRIMASK();
  __disable_irq();
  if (motorInitialized &&
      protectionState != MOTOR_OVERHEAT_LATCHED &&
      protectionState != MOTOR_SENSOR_FAULT_LATCHED &&
      protectionState != MOTOR_OVERCURRENT_LATCHED)
  {
    if (!isfinite(temperatureC) || !isfinite(currentPeakMa) || currentPeakMa < 0.0f)
    {
      protectionState = MOTOR_SENSOR_FAULT_LATCHED;
    }
    else if (temperatureC >= MOTOR_OVERHEAT_STOP_C)
    {
      protectionState = MOTOR_OVERHEAT_LATCHED;
    }
    else if (currentPeakMa >= MOTOR_OVERCURRENT_STOP_MA)
    {
      protectionState = MOTOR_OVERCURRENT_LATCHED;
    }
    else if (protectionState == MOTOR_WAITING_SENSORS &&
             temperatureC < MOTOR_START_MAX_TEMP_C)
    {
      protectionState = MOTOR_READY;
    }
    Motor_ApplyDuty();
  }
  __set_PRIMASK(irqState);
}

MotorProtectionState Motor_GetProtectionState(void)
{
  return protectionState;
}

uint8_t Motor_GetAppliedDuty(void)
{
  return (motorInitialized && protectionState == MOTOR_READY) ?
      (uint8_t)requestedDutyPercent : 0U;
}

uint8_t Motor_GetRequestedDuty(void)
{
  return (uint8_t)requestedDutyPercent;
}
