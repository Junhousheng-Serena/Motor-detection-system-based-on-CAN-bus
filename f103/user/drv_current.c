#include "drv_current.h"
#include <math.h>

float Current_VoltageToMilliamps(float voltage)
{
    if (!isfinite(voltage) || voltage < 0.0f)
    {
        return NAN;
    }

    return voltage * 1000000.0f /
           ((float)CURRENT_AMPLIFIER_GAIN * CURRENT_SHUNT_MILLIOHM);
}
