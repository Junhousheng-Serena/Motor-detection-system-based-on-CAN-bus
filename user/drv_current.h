#ifndef USER_DRV_CURRENT_H
#define USER_DRV_CURRENT_H

/* INA180A1IDBVR: 20 V/V, with a 1 ohm shunt. */
#define CURRENT_AMPLIFIER_GAIN       20U
#define CURRENT_SHUNT_MILLIOHM       1000U

#if CURRENT_AMPLIFIER_GAIN == 0 || CURRENT_SHUNT_MILLIOHM == 0
#error "Current sense gain and shunt resistance must be positive"
#endif

/* Positive shunt current in mA; invalid voltage returns NAN.
 * This unidirectional circuit cannot distinguish zero current from a
 * disconnected sensor or reversed shunt polarity. */
float Current_VoltageToMilliamps(float voltage);

#endif /* USER_DRV_CURRENT_H */
